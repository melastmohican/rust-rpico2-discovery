//! # Good Display GDEM0154Z90 1.54" Tri-Color E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! **GDEM0154Z90** 1.54" Tri-Color (Black/White/Red, 200x200) E-Paper Display using the
//! `Ssd1681Controller` from the `epdsi` library.
//!
//! Demonstrates:
//! 1. **Phase 1**: Full Tri-Color (Black/White/Red) refresh displaying header, colored accent banners,
//!    Ferris logo (Red), Rust logo (Black), and text labels.
//! 2. **Phase 2**: Partial *window* refresh loop that repaints only the bottom status band with an
//!    animated Black/Red progress bar, leaving the header and logos untouched.
//!
//! ## Note on refresh speed
//!
//! Tri-color (BWR) panels such as the GDEM0154Z90 have **no fast/differential waveform**: the red
//! pigment needs the long OTP waveform, so *every* update takes ~14 s. The SSD1681 "display mode 2"
//! trigger (`0x22 = 0xFC`) that enables sub-second updates on monochrome panels is not supported
//! here — using it just runs a slow update and, because only the Black/White RAM gets rewritten,
//! drops all red content. Phase 2 therefore keeps [`Ssd1681RefreshMode::Full`] (`0x22 = 0xF7`) and
//! narrows the RAM window instead, writing **both** the B/W and Red RAM for the updated region.
//! This mirrors GxEPD2's `GxEPD2_154_Z90c`, where `partial_refresh_time == full_refresh_time`.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Dalian Good Display GDEM0154Z90 1.54" Tri-Color E-Paper Display (200x200)
//! - **Adapter Board:** [Good Display DESPI-C02](https://www.good-display.com/product/516.html)
//!
//! ## Wiring Connection
//!
//! | Pico 2 Pin    | DESPI-C02 / Breakout Pin | Function              |
//! |---------------|--------------------------|-----------------------|
//! | 3V3 (Pin 36)  | 3.3V / VCC               | 3.3V Power Supply     |
//! | GND (Pin 38)  | GND                      | Ground                |
//! | GPIO11 (Pin 15)| RES / RST               | Reset                 |
//! | GPIO12 (Pin 16)| D/C                     | Data / Command Control|
//! | GPIO13 (Pin 17)| BUSY                    | Busy Status Signal    |
//! | GPIO16 (Pin 21)| MISO                    | SPI MISO              |
//! | GPIO17 (Pin 22)| CS                      | Display Chip Select   |
//! | GPIO18 (Pin 24)| SCK / CLK               | SPI Clock             |
//! | GPIO19 (Pin 25)| SDI / MOSI              | SPI MOSI Data         |
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ssd1681_gdem0154z90_epd
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
    defmt::info!("Starting GDEM0154Z90 1.54\" Tri-Color EPD example (epdsi SSD1681)");
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

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio25.into_push_pull_output();

    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    let busy = pins.gpio13.into_pull_down_input();

    // Create SPI driver instance (16 MHz clock)
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1681 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1681Controller::new(GDEM0154Z90::WIDTH, GDEM0154Z90::HEIGHT);

    // Build EPD Driver using epdsi with GDEM0154Z90 panel specification (200x200)
    let mut epd = EpdBuilder::<_, GDEM0154Z90>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1681 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Clear display controller RAM
    epd.clear_frame(ColorChannel::BlackWhite, 0xFF).unwrap();
    epd.clear_frame(ColorChannel::RedYellow, 0x00).unwrap();

    // Frame buffers: 200 x 200 / 8 = 5,000 bytes each
    let mut bw_buf = [0xFFu8; (GDEM0154Z90::WIDTH as usize * GDEM0154Z90::HEIGHT as usize) / 8];
    let mut red_buf = [0x00u8; (GDEM0154Z90::WIDTH as usize * GDEM0154Z90::HEIGHT as usize) / 8];

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    defmt::info!("--- Phase 1: Normal Full Tri-Color Refresh ---");
    defmt::info!("Drawing shapes, text, and logos onto frame buffers...");

    // Scoped so the full-frame borrows of `bw_buf` / `red_buf` end before Phase 2 re-borrows them
    // as smaller sub-region buffers.
    {
        let mut display_bw =
            PageBuffer::new(&mut bw_buf, GDEM0154Z90::WIDTH, GDEM0154Z90::HEIGHT, 0);
        let mut display_red =
            PageBuffer::new(&mut red_buf, GDEM0154Z90::WIDTH, GDEM0154Z90::HEIGHT, 0);

        // Outer border (Black)
        Rectangle::new(
            Point::new(0, 0),
            Size::new(GDEM0154Z90::WIDTH, GDEM0154Z90::HEIGHT),
        )
        .into_styled(style)
        .draw(&mut display_bw)
        .unwrap();

        // Header text (Black)
        Text::new("GDEM0154Z90 1.54\"", Point::new(10, 18), text_style)
            .draw(&mut display_bw)
            .unwrap();

        // Separator line (Black)
        Line::new(Point::new(10, 25), Point::new(190, 25))
            .into_styled(style)
            .draw(&mut display_bw)
            .unwrap();

        // Subtitle: "Tri-Color " in Black, "BWR" in Red
        Text::new("Tri-Color ", Point::new(10, 42), text_style)
            .draw(&mut display_bw)
            .unwrap();
        Text::new("BWR", Point::new(110, 42), text_style)
            .draw(&mut display_red)
            .unwrap();

        // Bounding box for color swatches
        Rectangle::new(Point::new(10, 50), Size::new(180, 16))
            .into_styled(style)
            .draw(&mut display_bw)
            .unwrap();

        // Black swatch banner inside bounding box (on Black/White frame)
        Rectangle::new(Point::new(12, 52), Size::new(84, 12))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut display_bw)
            .unwrap();

        // Red swatch banner inside bounding box (on Red frame, BinaryColor::Off sets bit=1 in 0x00-base buffer)
        Rectangle::new(Point::new(104, 52), Size::new(84, 12))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(&mut display_red)
            .unwrap();

        // Draw Ferris logo in Red (left side: x=20, y=75)
        let ferris_pos = Point::new(20, 75);
        for pixel in ferris_bmp.pixels() {
            if pixel.1 == BinaryColor::Off {
                Pixel(pixel.0 + ferris_pos, BinaryColor::Off)
                    .draw(&mut display_red)
                    .unwrap();
            }
        }

        // Draw Rust logo in Black (right side: x=115, y=75)
        let rust_pos = Point::new(115, 75);
        for pixel in rust_bmp.pixels() {
            if pixel.1 == BinaryColor::On {
                Pixel(pixel.0 + rust_pos, BinaryColor::On)
                    .draw(&mut display_bw)
                    .unwrap();
            }
        }

        // Draw text labels (Black)
        Text::new("RP2350 Pico 2", Point::new(10, 165), text_style)
            .draw(&mut display_bw)
            .unwrap();

        Text::new("epdsi SSD1681", Point::new(10, 185), text_style)
            .draw(&mut display_bw)
            .unwrap();

        // Each RAM write starts from the window origin, so reset window + cursor before both channels.
        defmt::info!("Sending Black/White frame (5,000 bytes)...");
        epd.set_window(0, 0, GDEM0154Z90::WIDTH - 1, GDEM0154Z90::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, display_bw.as_slice())
            .unwrap();

        defmt::info!("Sending Red frame (5,000 bytes)...");
        epd.set_window(0, 0, GDEM0154Z90::WIDTH - 1, GDEM0154Z90::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        epd.write_frame(ColorChannel::RedYellow, display_red.as_slice())
            .unwrap();

        defmt::info!("Refreshing display hardware (Full refresh, ~14 s)...");
        epd.refresh(&mut timer).unwrap();
    }

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Partial Window Tri-Color Refresh ---");

    // The refresh mode deliberately stays `Full` (0x22 = 0xF7). Trigger 0xFC selects the SSD1681
    // built-in fast LUT, which only exists for monochrome panels — on a BWR panel it is slow *and*
    // discards red. What makes this phase "partial" is the narrowed RAM window below.
    debug_assert_eq!(epd.controller().refresh_mode(), Ssd1681RefreshMode::Full);

    // Bottom status band, updated in place. The Rust logo ends at y = 139 (75 + 64), so the band
    // starts at y = 140 and the header/logos painted in Phase 1 are never touched.
    const BAND_Y: u32 = 140;
    const BAND_H: u32 = 60;
    const BAND_BYTES: usize = (GDEM0154Z90::WIDTH as usize * BAND_H as usize) / 8;

    for count in 1..=5u32 {
        // Sub-region buffers: 200 x 60 / 8 = 1,500 bytes of the full-frame arrays.
        let mut band_bw = PageBuffer::new(
            &mut bw_buf[..BAND_BYTES],
            GDEM0154Z90::WIDTH,
            BAND_H,
            BAND_Y,
        );
        let mut band_red = PageBuffer::new(
            &mut red_buf[..BAND_BYTES],
            GDEM0154Z90::WIDTH,
            BAND_H,
            BAND_Y,
        );

        band_bw.clear_byte(0xFF);
        band_red.clear_byte(0x00);

        // Update counter label (Black)
        let mut count_buf = [0u8; 32];
        let count_str =
            format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
        Text::new(count_str, Point::new(10, 157), text_style)
            .draw(&mut band_bw)
            .unwrap();

        // Progress bar outline (Black)
        Rectangle::new(Point::new(10, 164), Size::new(180, 14))
            .into_styled(style)
            .draw(&mut band_bw)
            .unwrap();

        // Progress bar fill (Red) — proves the Red channel survives a partial window update
        Rectangle::new(Point::new(12, 166), Size::new(count * 35, 10))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(&mut band_red)
            .unwrap();

        Text::new("Partial window", Point::new(10, 195), text_style)
            .draw(&mut band_bw)
            .unwrap();

        // Restrict controller RAM to the band, then write BOTH channels for that region. Writing
        // only Black/White would leave stale Red RAM behind for the band.
        epd.set_window(0, BAND_Y, GDEM0154Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, band_bw.as_slice())
            .unwrap();

        epd.set_window(0, BAND_Y, GDEM0154Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::RedYellow, band_red.as_slice())
            .unwrap();

        defmt::info!(
            "Refreshing band y={}..{} (Update #{}, ~14 s)...",
            BAND_Y,
            BAND_Y + BAND_H - 1,
            count
        );
        epd.refresh(&mut timer).unwrap();

        timer.delay_ms(1000);
    }

    // Restore the full-frame RAM window for any subsequent updates.
    epd.set_window(0, 0, GDEM0154Z90::WIDTH - 1, GDEM0154Z90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();

    defmt::info!("Display complete!");

    // Blink status LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
