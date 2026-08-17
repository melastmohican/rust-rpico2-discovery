//! # Pervasive Displays E2154QS0F1 E-Paper Example (epdsi)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! Pervasive Displays E2154QS0F1 (1.54" Spectra-4 BWRY, 152x152, no RAM overscan) using the `epdsi` library.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Pervasive Displays E2154QS0F1 1.54" Quad-Color E-Paper Display (EPDK / Driver F)
//!
//! ### Hardware Note for EXT3-1 Extension Boards:
//! - **J3 Jumper Setting**: Ensure the J3 jumper is OPEN (selecting the 10 µH inductor path for panels <= 3.7", e.g. 1.54" E2154QS0F1).
//!   - If J3 is closed (47 µH path for large screens), the DC-DC booster chokes during current bursts,
//!     causing voltage sags and busy-wait hangs.
//!
//! ## Wiring (EXT3/EPD connection)
//!
//! Connection using the **10-way rainbow bridging cable** provided with the EPDK.
//!
//! | Pico Pin       | Cable Color | EXT3 Pin / Function  |
//! |----------------|-------------|----------------------|
//! | 3V3 (Pin 36)   | **Black**   | 1 / VCC              |
//! | GPIO18 (Pin 24)| **Brown**   | 2 / SCK (SPI Clock)  |
//! | GPIO13 (Pin 17)| **Red**     | 3 / BUSY             |
//! | GPIO12 (Pin 16)| **Orange**  | 4 / DC (Data/Cmd)    |
//! | GPIO11 (Pin 15)| **Yellow**  | 5 / RST (Reset)      |
//! | GPIO16 (Pin 21)| **Green**   | 6 / MISO             |
//! | GPIO19 (Pin 25)| **Blue**    | 7 / MOSI             |
//! | NC             | **Violet**  | 8 / FCSM (Flash CS)  |
//! | GPIO17 (Pin 22)| **Grey**    | 9 / ECSM (Display CS)|
//! | GND (Pin 38)   | **White**   | 10 / GND             |
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pdi_e2154qs0f1
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{
    ErrorType as DigitalErrorType, InputPin, OutputPin, StatefulOutputPin,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use epdsi::prelude::*;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{
    FunctionSio, FunctionSpi, Pin, PinId, PullNone, SioInput, SioOutput, ValidFunction,
};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;

/// Adapts an rp235x-hal GPIO pin to [`epdsi::bus3::DynamicPin`] by switching between the pin's
/// `FunctionSio<SioOutput>` and `FunctionSio<SioInput>` type states at runtime.
///
/// Required because the Pervasive BWRY OTP read handshake bit-bangs a single MOSI wire in both
/// directions (see `Spi3Bus`) — rp235x-hal (like most Rust HALs) represents input vs. output as
/// distinct pin types, so switching direction means converting and re-storing the pin, not just
/// flipping a register bit through a fixed type.
enum FlexPinInner<I: PinId> {
    In(Pin<I, FunctionSio<SioInput>, PullNone>),
    Out(Pin<I, FunctionSio<SioOutput>, PullNone>),
}

struct FlexPin<I: PinId>(Option<FlexPinInner<I>>);

impl<I> FlexPin<I>
where
    I: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
{
    fn new_output<P: hal::gpio::PullType>(pin: Pin<I, FunctionSio<SioOutput>, P>) -> Self {
        Self(Some(FlexPinInner::Out(pin.into_pull_type::<PullNone>())))
    }

    /// Consumes self, returning the pin configured as a floating input. Panics if the pin is
    /// currently in output mode — the OTP handshake always leaves the data pin in input mode
    /// (its last operation is always a bit-banged read), so this is safe to call afterward.
    fn release_as_input(mut self) -> Pin<I, FunctionSio<SioInput>, PullNone> {
        match self.0.take().expect("FlexPin poisoned") {
            FlexPinInner::In(p) => p,
            FlexPinInner::Out(_) => panic!("FlexPin::release_as_input called while in output mode"),
        }
    }
}

impl<I: PinId> DigitalErrorType for FlexPin<I> {
    type Error = core::convert::Infallible;
}

impl<I> epdsi::bus3::DynamicPin for FlexPin<I>
where
    I: PinId + ValidFunction<FunctionSio<SioInput>> + ValidFunction<FunctionSio<SioOutput>>,
{
    type Error = core::convert::Infallible;

    fn set_as_output(&mut self) -> Result<(), Self::Error> {
        let inner = self.0.take().expect("FlexPin poisoned");
        self.0 = Some(match inner {
            FlexPinInner::In(p) => FlexPinInner::Out(p.into_push_pull_output()),
            out @ FlexPinInner::Out(_) => out,
        });
        Ok(())
    }

    fn set_as_input(&mut self) -> Result<(), Self::Error> {
        let inner = self.0.take().expect("FlexPin poisoned");
        self.0 = Some(match inner {
            FlexPinInner::Out(p) => FlexPinInner::In(p.into_floating_input()),
            input @ FlexPinInner::In(_) => input,
        });
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self.0.as_mut().expect("FlexPin poisoned") {
            FlexPinInner::Out(p) => p.set_high(),
            FlexPinInner::In(_) => panic!("FlexPin::set_high called while in input mode"),
        }
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self.0.as_mut().expect("FlexPin poisoned") {
            FlexPinInner::Out(p) => p.set_low(),
            FlexPinInner::In(_) => panic!("FlexPin::set_low called while in input mode"),
        }
    }

    fn is_high(&mut self) -> Result<bool, Self::Error> {
        match self.0.as_mut().expect("FlexPin poisoned") {
            FlexPinInner::In(p) => p.is_high(),
            FlexPinInner::Out(_) => panic!("FlexPin::is_high called while in output mode"),
        }
    }
}

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Converts a 1bpp PageBuffer slice (152x152 bits, 2,888 bytes) into a 2bpp BWRY frame buffer (152x152 2bpp, 5,776 bytes).
/// 1bpp White (bit=1) -> 2bpp `0b01` (White)
/// 1bpp Black (bit=0) -> 2bpp `0b00` (Black)
fn convert_1bpp_to_2bpp_bwry(src_1bpp: &[u8], dst_2bpp: &mut [u8]) {
    for (i, &byte) in src_1bpp.iter().enumerate() {
        let mut high_2bpp = 0u8;
        let mut low_2bpp = 0u8;

        // High nibble (pixels 0..3)
        for bit in 0..4 {
            let is_set = (byte & (1 << (7 - bit))) != 0;
            let val = if is_set { 0b01 } else { 0b00 };
            high_2bpp |= val << ((3 - bit) * 2);
        }

        // Low nibble (pixels 4..7)
        for bit in 0..4 {
            let is_set = (byte & (1 << (3 - bit))) != 0;
            let val = if is_set { 0b01 } else { 0b00 };
            low_2bpp |= val << ((3 - bit) * 2);
        }

        dst_2bpp[i * 2] = high_2bpp;
        dst_2bpp[i * 2 + 1] = low_2bpp;
    }
}

/// Helper to draw a colored rectangle directly onto the 2bpp BWRY frame buffer.
/// `color_code`: 0b00 = Black, 0b01 = White, 0b10 = Yellow, 0b11 = Red
fn fill_rect_2bpp(dst_2bpp: &mut [u8], width: u32, x: u32, y: u32, w: u32, h: u32, color_code: u8) {
    let color_2bit = color_code & 0x03;
    for py in y..(y + h) {
        for px in x..(x + w) {
            let pixel_idx = (py * width + px) as usize;
            let byte_idx = pixel_idx / 4;
            let pixel_in_byte = pixel_idx % 4;
            let bit_shift = (3 - pixel_in_byte) * 2;
            let mask = !(0x03 << bit_shift);
            dst_2bpp[byte_idx] = (dst_2bpp[byte_idx] & mask) | (color_2bit << bit_shift);
        }
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Create a delay using the system timer
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED pin for status indication
    let mut led_pin: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio25.into_push_pull_output();

    // Hardware Note for EXT3-1 extension boards:
    // Ensure the J3 jumper is OPEN (selecting the 10 µH inductor path for panels <= 3.7", e.g. 1.54" E2154QS0F1).
    // If J3 is closed (47 µH path for large screens), the DC-DC booster chokes during current bursts,
    // causing voltage sags and busy-wait hangs.

    // Control pins for e-paper
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    // BUSY pin is active-LOW for Pervasive Displays COG (low = busy).
    let busy = pins.gpio13.into_pull_up_input();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // SCK/MOSI start as plain GPIO: the Pervasive BWRY COG's OTP register read is a bit-banged
    // 3-wire handshake (SCK + a single bidirectional DATA line) done entirely in software, NOT
    // over the hardware SPI peripheral — the panel drives its response back on MOSI, and MISO is
    // never used for this handshake. See epdsi::bus3::Spi3Bus for why SpiBusWrapper can't do this.
    let sck_gpio = pins.gpio18.into_push_pull_output();
    let mosi_flex = FlexPin::new_output(pins.gpio19.into_push_pull_output());

    let mut controller = PervasiveBwryController::new(E2154QS0F1::WIDTH, E2154QS0F1::HEIGHT)
        .with_variant(PervasiveBwryVariant::DriverF)
        .with_temperature(25);

    defmt::info!("Reading OTP register data from panel (bit-banged 3-wire handshake)...");
    let mut bus3 = Spi3Bus::new(cs, sck_gpio, mosi_flex, dc, rst, busy);
    controller
        .read_otp(&mut bus3, &mut timer)
        .expect("Pervasive BWRY OTP read failed");
    let (cs, sck_gpio, mosi_flex, dc, rst, busy) = bus3.release();
    defmt::info!("OTP register data read OK");

    // Reconfigure SCK/MOSI into the hardware SPI peripheral's function for normal 4-wire operation
    // (the OTP handshake always leaves the data pin in input mode, since its last step is a read).
    let sck: Pin<_, FunctionSpi, _> = sck_gpio.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = mosi_flex.release_as_input().into_function::<FunctionSpi>();

    // Create SPI bus with 16 MHz clock speed
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    defmt::info!("SPI configured at 16 MHz");

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Wrap SPI device and control pins using epdsi SpiBusWrapper
    let bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let mut driver = EpdBuilder::<_, E2154QS0F1>::new(controller).build(bus);

    defmt::info!("Initializing E2154QS0F1 display (BWRY Driver F) via epdsi driver...");
    // init() no longer touches OTP data (already read above); it does the non-OTP init steps.
    driver.init(&mut timer).unwrap();
    defmt::info!("E-Paper display initialized");

    // 1bpp monochrome drawing buffer (152 x 152 / 8 = 2,888 bytes)
    let mut buffer_1bpp = [0u8; (E2154QS0F1::WIDTH as usize * E2154QS0F1::HEIGHT as usize) / 8];
    // 2bpp BWRY packed frame buffer (152 x 152 * 2 / 8 = 5,776 bytes)
    let mut buffer_2bpp = [0u8; (E2154QS0F1::WIDTH as usize * E2154QS0F1::HEIGHT as usize) / 4];

    let mut display = PageBuffer::new(&mut buffer_1bpp, E2154QS0F1::WIDTH, E2154QS0F1::HEIGHT, 0);

    // Clear display buffer to White (0xFF in PageBuffer)
    display.clear_byte(0xFF);

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    defmt::info!("Drawing shapes, text, and logos onto frame buffer...");

    // Note: the 1.54" E2154QS0F1 panel is 152x152 with no RAM overscan — the whole canvas is visible.
    Text::new("E2154QS0F1 1.54\"", Point::new(5, 15), text_style)
        .draw(&mut display)
        .unwrap();

    Line::new(Point::new(5, 22), Point::new(145, 22))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    Text::new("Spectra-4 BWRY", Point::new(5, 38), text_style)
        .draw(&mut display)
        .unwrap();

    // Bounding box within 152x152 active region
    Rectangle::new(Point::new(5, 45), Size::new(140, 12))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // Draw Ferris logo (horizontally centered under the red banner: banner spans x=[7,72), so
    // centered at x=39; ferris_bmp is 64px wide, so its origin is 39 - 64/2 = 7)
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + Point::new(7, 60), BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Draw Rust logo (horizontally centered under the yellow banner: banner spans x=[75,140), so
    // centered at x=107; rust_bmp is 64px wide, so its origin is 107 - 64/2 = 75)
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + Point::new(75, 60), BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Draw text labels
    Text::new("RP2350 Pico 2", Point::new(5, 125), text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("epdsi Driver F", Point::new(5, 145), text_style)
        .draw(&mut display)
        .unwrap();

    // Convert 1bpp monochrome layout to 2bpp BWRY format
    convert_1bpp_to_2bpp_bwry(display.as_slice(), &mut buffer_2bpp);

    // Overlay Red (0b11) and Yellow (0b10) accent color blocks to demonstrate Quad-Color capability
    // Red banner inside bounding box
    fill_rect_2bpp(&mut buffer_2bpp, E2154QS0F1::WIDTH, 7, 47, 65, 8, 0b11);
    // Yellow banner inside bounding box
    fill_rect_2bpp(&mut buffer_2bpp, E2154QS0F1::WIDTH, 75, 47, 65, 8, 0b10);

    defmt::info!("Sending BWRY frame buffer data (5,776 bytes) to display...");
    driver
        .write_frame(ColorChannel::BlackWhite, &buffer_2bpp)
        .unwrap();

    defmt::info!("Refreshing display hardware (Full OTP-driven update)...");
    driver.refresh(&mut timer).unwrap();

    defmt::info!("Powering off DC/DC...");
    driver.sleep(&mut timer).unwrap();

    defmt::info!("Display update complete!");

    // Blink LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
