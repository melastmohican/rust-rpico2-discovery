//! # Good Display GDEY037T03 3.7" Monochrome E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! **GDEY037T03** 3.7" Monochrome (Black/White, 240x416) E-Paper Display using the
//! `Uc8253Controller` from the `epdsi` library.
//!
//! Demonstrates:
//! 1. **Phase 1**: Full monochrome refresh showing a header, separator, the Ferris and Rust logos
//!    side by side, and text labels.
//! 2. **Phase 2**: Partial-window refresh loop that repaints only the content band
//!    (y = 66..415), swapping the two logos on every pass and advancing a progress bar, while the
//!    header above the band is never touched.
//! 3. **Phase 3**: Full-panel, full-waveform cleanup pass restoring the ink density that the
//!    shortened partial waveform leaves behind.
//!
//! ## Note on the UC8253 command model
//!
//! The UC8253 differs from the SSD16xx family used by the other `epdsi` examples:
//!
//! - The RAM area must be set before writing image data, and the partial window is re-opened
//!   around *every* RAM write and again around the refresh
//!   (`PARTIAL_IN` -> `PARTIAL_WINDOW` -> operation -> `PARTIAL_OUT`). `set_window` records the
//!   area; `epdsi` emits the commands per operation.
//! - The two RAM banks are **old/new planes**, not colors: [`ColorChannel::BlackWhite`] maps to
//!   `WRITE_NEW_DATA` (`0x13`) and [`ColorChannel::RedYellow`] to `WRITE_OLD_DATA` (`0x10`).
//!   The old plane is primed to white before the first write.
//! - BUSY is active-**LOW** on this panel, the opposite of the SSD16xx panels, so the GPIO takes
//!   a pull-**up** rather than a pull-down.
//! - Orientation: buffer coordinates map directly to the panel, with `(0,0)` at the top-left
//!   when the **FPC ribbon is at the bottom**. No rotation or mirroring is applied.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Dalian Good Display GDEY037T03 3.7" Monochrome E-Paper Display (240x416)
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
//! cargo run --example uc8253_gdey037t03_epd
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

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Row stride in bytes: 240 / 8 = 30. This panel is already byte-aligned.
const STRIDE: usize = GDEY037T03::WIDTH.div_ceil(8) as usize;

/// Full frame buffer size: 30 x 416 = 12,480 bytes.
const FRAME_BYTES: usize = STRIDE * GDEY037T03::HEIGHT as usize;

/// Top Y coordinate of the content band repainted in Phase 2.
const BAND_Y: u32 = 66;

/// Height of the content band in pixels (y = 66..415).
const BAND_H: u32 = 350;

/// Last row of the band.
const BAND_END: u32 = BAND_Y + BAND_H - 1;

/// Byte offset of the band's first row within the frame buffer.
const BAND_START_BYTE: usize = BAND_Y as usize * STRIDE;

/// Byte offset one past the band's last row.
const BAND_END_BYTE: usize = (BAND_END as usize + 1) * STRIDE;

/// X coordinate of the left logo slot.
const LOGO_LEFT_X: i32 = 30;

/// X coordinate of the right logo slot.
const LOGO_RIGHT_X: i32 = 146;

/// Draws the static chrome above the Phase 2 band: border, header, subtitle and separator.
fn draw_chrome(display: &mut PageBuffer) {
    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEY037T03::WIDTH, GDEY037T03::HEIGHT),
    )
    .into_styled(style)
    .draw(display)
    .unwrap();

    Text::new("GDEY037T03", Point::new(10, 24), text_style)
        .draw(display)
        .unwrap();

    Text::new("3.7\" Mono", Point::new(10, 48), text_style)
        .draw(display)
        .unwrap();

    Line::new(Point::new(10, 58), Point::new(229, 58))
        .into_styled(style)
        .draw(display)
        .unwrap();
}

/// Draws everything inside the Phase 2 band: the two logos, the footer labels and the progress
/// indicator.
///
/// `swapped` exchanges the two logo slots — Phase 2 flips it on every pass so the partial update
/// is obvious at a glance. Ferris is 64x42 and Rust is 64x64, and their Y positions differ so
/// their bottoms line up. `count` of 0 renders the Phase 1 state instead of an update counter.
fn draw_content(
    display: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
    swapped: bool,
    count: u32,
) {
    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    let (ferris_x, rust_x) = if swapped {
        (LOGO_RIGHT_X, LOGO_LEFT_X)
    } else {
        (LOGO_LEFT_X, LOGO_RIGHT_X)
    };

    // The Ferris BMP has the opposite polarity to the Rust BMP, hence the `Off` test here.
    let ferris_pos = Point::new(ferris_x, 112);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    let rust_pos = Point::new(rust_x, 90);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    Text::new("RP2350 Pico 2", Point::new(10, 200), text_style)
        .draw(display)
        .unwrap();

    Text::new("epdsi UC8253", Point::new(10, 225), text_style)
        .draw(display)
        .unwrap();

    Line::new(Point::new(10, 240), Point::new(229, 240))
        .into_styled(style)
        .draw(display)
        .unwrap();

    let mut count_buf = [0u8; 32];
    let label = if count == 0 {
        "Full refresh"
    } else {
        format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap()
    };
    Text::new(label, Point::new(10, 285), text_style)
        .draw(display)
        .unwrap();

    Rectangle::new(Point::new(10, 300), Size::new(220, 22))
        .into_styled(style)
        .draw(display)
        .unwrap();

    // Progress bar fill: 6 steps of 35 px stay inside the 216 px interior
    if count > 0 {
        Rectangle::new(Point::new(12, 303), Size::new(count * 35, 16))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display)
            .unwrap();
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEY037T03 3.7\" Monochrome EPD example (epdsi UC8253)");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        12_000_000u32,
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
    // UC8253 BUSY is active-LOW, unlike the SSD16xx panels in this repo
    let busy = pins.gpio13.into_pull_up_input();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Uc8253Controller::new(GDEY037T03::WIDTH, GDEY037T03::HEIGHT);
    let mut epd = EpdBuilder::<_, GDEY037T03>::new(controller).build(epd_bus);

    defmt::info!("Initializing UC8253 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Prime the old plane to white so the update has a clean base.
    epd.clear_frame(ColorChannel::RedYellow, 0xFF).unwrap();

    let mut bw_buf = [0xFFu8; FRAME_BYTES];

    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    // Buffer coordinates map straight to the panel with the FPC ribbon at the bottom: (0,0) is
    // the top-left of the visible image. No rotation or mirroring is needed.
    let mut display = PageBuffer::new(&mut bw_buf, GDEY037T03::WIDTH, GDEY037T03::HEIGHT, 0);

    defmt::info!("--- Phase 1: Full Monochrome Refresh ---");

    draw_chrome(&mut display);
    draw_content(&mut display, &ferris_bmp, &rust_bmp, false, 0);

    defmt::info!("Sending Black/White frame (12,480 bytes)...");
    epd.set_window(0, 0, GDEY037T03::WIDTH - 1, GDEY037T03::HEIGHT - 1)
        .unwrap();
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();

    defmt::info!("Refreshing display hardware (Full refresh)...");
    epd.refresh(&mut timer).unwrap();

    // Sync the old plane with what is now on the panel.
    epd.set_window(0, 0, GDEY037T03::WIDTH - 1, GDEY037T03::HEIGHT - 1)
        .unwrap();
    epd.write_frame(ColorChannel::RedYellow, display.as_slice())
        .unwrap();

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Partial Window Refresh (logo swap) ---");

    // GxEPD2 declares `hasFastPartialUpdate = true` for this panel, so its partial path always
    // applies the CCSET/TSSET temperature override — that is `FastPartial` here, not `Partial`.
    epd.controller_mut()
        .set_refresh_mode(Uc8253RefreshMode::FastPartial);

    for count in 1..=6u32 {
        // Flip the logo slots on every pass so the partial update is unmistakable.
        let swapped = count % 2 == 1;

        // The whole buffer is redrawn, but only the band's rows are sent, so the chrome above the
        // band is never repainted.
        display.clear_byte(0xFF);
        draw_chrome(&mut display);
        draw_content(&mut display, &ferris_bmp, &rust_bmp, swapped, count);

        epd.set_window(0, BAND_Y, GDEY037T03::WIDTH - 1, BAND_END)
            .unwrap();
        epd.write_frame(
            ColorChannel::BlackWhite,
            &display.as_slice()[BAND_START_BYTE..BAND_END_BYTE],
        )
        .unwrap();

        defmt::info!(
            "Refreshing band y={}..{} (Update #{}, logos {})...",
            BAND_Y,
            BAND_END,
            count,
            if swapped { "swapped" } else { "normal" }
        );
        epd.refresh(&mut timer).unwrap();

        // Keep the old plane in step with the panel for the next differential pass.
        epd.set_window(0, BAND_Y, GDEY037T03::WIDTH - 1, BAND_END)
            .unwrap();
        epd.write_frame(
            ColorChannel::RedYellow,
            &display.as_slice()[BAND_START_BYTE..BAND_END_BYTE],
        )
        .unwrap();

        timer.delay_ms(1000);
    }

    defmt::info!("--- Phase 3: Full-Waveform Cleanup Pass ---");

    // The partial waveform settles pixels at a dark grey rather than a deep black. Redraw the
    // whole frame and run it through the full waveform to restore even ink density.
    epd.controller_mut()
        .set_refresh_mode(Uc8253RefreshMode::Full);

    display.clear_byte(0xFF);
    draw_chrome(&mut display);
    draw_content(&mut display, &ferris_bmp, &rust_bmp, false, 6);

    epd.set_window(0, 0, GDEY037T03::WIDTH - 1, GDEY037T03::HEIGHT - 1)
        .unwrap();
    epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
        .unwrap();

    defmt::info!("Refreshing full panel with the full waveform...");
    epd.refresh(&mut timer).unwrap();

    defmt::info!("Display complete!");

    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
