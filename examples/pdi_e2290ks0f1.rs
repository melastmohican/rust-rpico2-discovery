//! # Pervasive Displays E2290KS0F1 E-Paper Example (epdsi)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! Pervasive Displays E2290KS0F1 (2.90" Monochrome 384x168 EPD) using the `epdsi` library.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Pervasive Displays E2290KS0F1 2.90" Monochrome E-Paper Display (EPDK / Driver F)
//!
//! ### Hardware Note for EXT3-1 Extension Boards:
//! - **J3 Jumper Setting**: Ensure the J3 jumper is OPEN (selecting the 10 µH inductor path for panels <= 3.7", e.g. 2.9" E2290KS0F1).
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
//! cargo run --example pdi_e2290ks0f1
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
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use epdsi::prelude::*;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

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

    // Configure SPI pins for e-paper display
    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // Hardware Note for EXT3-1 extension boards:
    // Ensure the J3 jumper is OPEN (selecting the 10 µH inductor path for panels <= 3.7", e.g. 2.9" E2290KS0F1).
    // If J3 is closed (47 µH path for large screens), the DC-DC booster chokes during current bursts,
    // causing voltage sags and busy-wait hangs.

    // Control pins for e-paper
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    // BUSY pin is active-LOW for Pervasive Displays COG (low = busy).
    let busy = pins.gpio13.into_pull_up_input();

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

    // Create Pervasive Displays controller configured for Driver F and E2290KS0F1 panel (168x384)
    let controller = PervasiveDisplaysController::new(E2290KS0F1::WIDTH, E2290KS0F1::HEIGHT)
        .with_driver_variant(PervasiveDriverVariant::DriverF);
    let mut driver = EpdBuilder::<_, E2290KS0F1>::new(controller).build(bus);

    defmt::info!("Initializing E2290KS0F1 display (Driver F) via epdsi driver...");
    driver.init(&mut timer).unwrap();
    defmt::info!("E-Paper display initialized");

    // Clear Red frame buffer (DTM2) to 0x00 (no red pixels, preventing controller RAM noise)
    driver.clear_frame(ColorChannel::RedYellow, 0x00).unwrap();

    // Full frame buffer (168 width x 384 height / 8 = 8,064 bytes RAM)
    let mut buffer = [0u8; (E2290KS0F1::WIDTH as usize * E2290KS0F1::HEIGHT as usize) / 8];
    let mut prev_buffer = [0u8; (E2290KS0F1::WIDTH as usize * E2290KS0F1::HEIGHT as usize) / 8];
    let mut display = PageBuffer::new(&mut buffer, E2290KS0F1::WIDTH, E2290KS0F1::HEIGHT, 0);

    // Clear display buffer to White (0xFF in PageBuffer)
    display.clear_byte(0xFF);

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    defmt::info!("--- Phase 1: Normal Full Refresh ---");
    defmt::info!("Drawing initial shapes and text onto frame buffer...");

    // Draw header text for E2290KS0F1 2.90" panel
    Text::new("E2290KS0F1 2.90\"", Point::new(10, 15), text_style)
        .draw(&mut display)
        .unwrap();

    // Draw separator line
    Line::new(Point::new(10, 25), Point::new(155, 25))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    Text::new("Monochrome 1bpp", Point::new(10, 42), text_style)
        .draw(&mut display)
        .unwrap();

    // Draw bounding box
    Rectangle::new(Point::new(10, 65), Size::new(148, 15))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    // Draw Ferris logo
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + Point::new(10, 85), BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Draw Rust logo
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + Point::new(80, 85), BinaryColor::On)
                .draw(&mut display)
                .unwrap();
        }
    }

    // Draw text labels
    Text::new("RP2350 Pico 2", Point::new(10, 170), text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("epdsi Driver F", Point::new(10, 195), text_style)
        .draw(&mut display)
        .unwrap();

    defmt::info!("Initializing display registers before normal update...");
    driver.init(&mut timer).unwrap();

    defmt::info!("Sending frame buffer data to display...");
    driver
        .write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();

    defmt::info!("Refreshing display hardware (Normal full refresh)...");
    driver.refresh(&mut timer).unwrap();

    defmt::info!("Powering off DC/DC...");
    driver.sleep(&mut timer).unwrap();

    // Save initial frame to prev_buffer for differential update base
    prev_buffer.copy_from_slice(display.as_slice());
    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Fast Differential Refresh ---");
    defmt::info!("Switching PervasiveDisplaysController to Fast refresh mode...");
    driver
        .controller_mut()
        .set_refresh_mode(PervasiveRefreshMode::Fast);
    driver.init(&mut timer).unwrap();

    for count in 1..=5 {
        display.clear_byte(0xFF);

        Text::new("Driver F Fast", Point::new(10, 15), text_style)
            .draw(&mut display)
            .unwrap();

        Line::new(Point::new(10, 25), Point::new(155, 25))
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        let mut count_buf = [0u8; 32];
        let count_str =
            format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
        Text::new(count_str, Point::new(10, 42), text_style)
            .draw(&mut display)
            .unwrap();

        let bar_width = count * 28;
        Rectangle::new(Point::new(10, 65), Size::new(bar_width, 10))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut display)
            .unwrap();

        // Alternate positions of Ferris and Rust logos on each iteration to make differential refresh clear
        let (ferris_pos, rust_pos) = if count % 2 == 1 {
            (Point::new(80, 85), Point::new(10, 85))
        } else {
            (Point::new(10, 85), Point::new(80, 85))
        };

        // Draw Ferris logo
        for pixel in ferris_bmp.pixels() {
            if pixel.1 == BinaryColor::Off {
                Pixel(pixel.0 + ferris_pos, BinaryColor::On)
                    .draw(&mut display)
                    .unwrap();
            }
        }

        // Draw Rust logo
        for pixel in rust_bmp.pixels() {
            if pixel.1 == BinaryColor::On {
                Pixel(pixel.0 + rust_pos, BinaryColor::On)
                    .draw(&mut display)
                    .unwrap();
            }
        }

        Text::new("RP2350 Pico 2", Point::new(10, 170), text_style)
            .draw(&mut display)
            .unwrap();

        Text::new("epdsi fast mode", Point::new(10, 195), text_style)
            .draw(&mut display)
            .unwrap();

        let (bus, controller) = driver.split_mut();
        controller
            .write_fast_frame(bus, &prev_buffer, display.as_slice())
            .unwrap();

        defmt::info!("Refreshing display in fast mode...");
        driver.refresh(&mut timer).unwrap();
        driver.sleep(&mut timer).unwrap();

        // Update prev_buffer for next differential step
        prev_buffer.copy_from_slice(display.as_slice());
        timer.delay_ms(1000);
    }

    defmt::info!("Powering off / sleeping display...");
    driver.sleep(&mut timer).unwrap();
    defmt::info!("Display complete!");

    // Blink LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
