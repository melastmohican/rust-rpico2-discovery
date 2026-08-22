//! # Good Display GDEM0213B74 2.13" Monochrome E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the
//! **GDEM0213B74** 2.13" Monochrome (Black/White, 122x250) E-Paper Display using the
//! `Ssd1680Controller` from the `epdsi` library.
//!
//! Demonstrates:
//! 1. **Phase 1**: Full monochrome refresh displaying a header, separator, Ferris logo (top),
//!    Rust logo (bottom), and text labels.
//! 2. **Phase 2**: Fast *differential* partial-window refresh loop that repaints only the
//!    content band (y = 50..249), swapping the Ferris and Rust logos on every pass and advancing
//!    a progress bar, while the header above the band is never touched.
//! 3. **Phase 3**: Full-waveform cleanup pass over the same band, restoring the ink density that
//!    the shortened differential waveform leaves behind.
//!
//! ## Note on the 122 pixel panel width
//!
//! The panel is 122 pixels wide, which is not a byte multiple. The SSD1680 addresses RAM in
//! whole bytes, so `set_window(0, .., 121, ..)` selects RAM bytes 0..=15 and the controller
//! expects **16 bytes per row**. [`PageBuffer`] rounds its row stride up to a whole byte, so
//! it is constructed with the real visible width and the frame buffer is sized
//! `WIDTH.div_ceil(8) * HEIGHT` (4,000 bytes) rather than `WIDTH * HEIGHT / 8`. Pixels at
//! x = 122..127 fall in the off-panel padding and are clipped.
//!
//! ## Note on refresh speed
//!
//! Unlike the tri-color GDEM0154Z90, this panel is monochrome, so the SSD1680 built-in fast
//! LUT (`0x22 = 0xFC`, selected by [`Ssd168xRefreshMode::Partial`]) gives a genuine sub-second
//! differential update. Differential mode diffs the new image in Black/White RAM (`0x24`)
//! against the *previous* image in the secondary RAM (`0x26`, reached through
//! [`ColorChannel::RedYellow`]), so this example keeps `0x26` in sync after every refresh.
//! If you still see ghosting, note that `epdsi` pins `BORDER_WAVEFORM_CONTROL` to `0x05` at
//! init where GxEPD2 switches to `0x80` for partial updates; that register is not reachable
//! through the public API and may produce a faint border flash.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Dalian Good Display GDEM0213B74 2.13" Monochrome E-Paper Display (122x250)
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
//! cargo run --example ssd1680_gdem0213b74_epd
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_10X20};
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

/// Row stride in bytes. 122 px rounds up to 16 bytes; see the module docs.
const STRIDE: usize = GDEM0213B74::WIDTH.div_ceil(8) as usize;

/// Full frame buffer size: 16 x 250 = 4,000 bytes.
const FRAME_BYTES: usize = STRIDE * GDEM0213B74::HEIGHT as usize;

/// Top Y coordinate of the content band repainted in Phase 2. The header and the separator
/// above it (y = 0..49) are painted once in Phase 1 and never touched again.
const BAND_Y: u32 = 50;

/// Height of the content band in pixels (y = 50..249).
const BAND_H: u32 = 200;

/// Content band buffer size: 16 x 200 = 3,200 bytes.
const BAND_BYTES: usize = STRIDE * BAND_H as usize;

/// All-white fill for the content band, used to blank the secondary RAM before the Phase 3
/// cleanup pass. Lives in flash rather than on the stack.
static WHITE_BAND: [u8; BAND_BYTES] = [0xFFu8; BAND_BYTES];

/// X coordinate that horizontally centres a 64 px logo on the 122 px panel.
const LOGO_X: i32 = (GDEM0213B74::WIDTH as i32 - 64) / 2;

/// Top Y coordinate of the upper logo slot.
const LOGO_TOP_Y: i32 = 52;

/// Draws the Ferris and Rust logos stacked vertically, horizontally centred on the panel.
///
/// The two 64 px-wide logos cannot sit side by side on a 122 px panel, so they are stacked.
/// `swapped` exchanges which logo occupies the upper slot: Phase 2 flips it on every partial
/// update so the differential refresh is obvious at a glance. Ferris is 64x42 and Rust is
/// 64x64, and the offsets are chosen so both arrangements end at y = 161.
fn draw_logos(
    display: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
    swapped: bool,
) {
    let (ferris_y, rust_y) = if swapped {
        (LOGO_TOP_Y + 68, LOGO_TOP_Y)
    } else {
        (LOGO_TOP_Y, LOGO_TOP_Y + 46)
    };

    // The Ferris BMP has the opposite polarity to the Rust BMP, hence the `Off` test here.
    let ferris_pos = Point::new(LOGO_X, ferris_y);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    let rust_pos = Point::new(LOGO_X, rust_y);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEM0213B74 2.13\" Monochrome EPD example (epdsi SSD1680)");
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
    // SSD1680 BUSY is active-HIGH
    let busy = pins.gpio13.into_pull_down_input();

    // Create SPI driver instance (16 MHz clock)
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1680 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1680Controller::new(GDEM0213B74::WIDTH, GDEM0213B74::HEIGHT);

    // Build EPD Driver using epdsi with GDEM0213B74 panel specification (122x250)
    let mut epd = EpdBuilder::<_, GDEM0213B74>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1680 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Clear both display controller RAM banks to white. On this monochrome panel the secondary
    // RAM (0x26) is not a color plane but the "previous image" used by differential updates.
    epd.clear_frame(ColorChannel::BlackWhite, 0xFF).unwrap();
    epd.clear_frame(ColorChannel::RedYellow, 0xFF).unwrap();

    // Frame buffer: 128 x 250 / 8 = 4,000 bytes (0xFF = white)
    let mut bw_buf = [0xFFu8; FRAME_BYTES];

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    // The panel is only 122 px wide, so the footer labels use the smaller 6x10 font
    let small_text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    defmt::info!("--- Phase 1: Full Monochrome Refresh ---");
    defmt::info!("Drawing shapes, text, and logos onto frame buffer...");

    // Scoped so the full-frame borrow of `bw_buf` ends before Phase 2 re-borrows it as a
    // smaller sub-region buffer.
    {
        let mut display = PageBuffer::new(&mut bw_buf, GDEM0213B74::WIDTH, GDEM0213B74::HEIGHT, 0);

        // Outer border (visible area only)
        Rectangle::new(
            Point::new(0, 0),
            Size::new(GDEM0213B74::WIDTH, GDEM0213B74::HEIGHT),
        )
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

        // Header text
        Text::new("GDEM0213B74", Point::new(6, 18), text_style)
            .draw(&mut display)
            .unwrap();

        // Subtitle
        Text::new("2.13\" Mono", Point::new(6, 38), text_style)
            .draw(&mut display)
            .unwrap();

        // Separator line
        Line::new(Point::new(6, 45), Point::new(115, 45))
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        // Ferris on top, Rust below. Phase 2 swaps them on every partial update.
        draw_logos(&mut display, &ferris_bmp, &rust_bmp, false);

        // Draw text labels
        Text::new("RP2350 Pico 2", Point::new(6, 180), small_text_style)
            .draw(&mut display)
            .unwrap();

        Text::new("epdsi SSD1680", Point::new(6, 195), small_text_style)
            .draw(&mut display)
            .unwrap();

        // Separator above the Phase 2 status band
        Line::new(Point::new(6, 203), Point::new(115, 203))
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        // Each RAM write starts from the window origin, so reset window + cursor first.
        defmt::info!("Sending Black/White frame (4,000 bytes)...");
        epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, display.as_slice())
            .unwrap();

        defmt::info!("Refreshing display hardware (Full refresh)...");
        epd.refresh(&mut timer).unwrap();

        // Seed the "previous image" RAM (0x26) with what is now physically on the panel, so the
        // Phase 2 differential updates have a correct base to diff against.
        epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        epd.write_frame(ColorChannel::RedYellow, display.as_slice())
            .unwrap();
    }

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Fast Partial Window Refresh (logo swap) ---");

    // Select the SSD1680 built-in fast LUT (0x22 = 0xFC). On this monochrome panel that is a
    // real differential update and completes in well under a second.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Partial);

    for count in 1..=6u32 {
        // Flip the logo order on every pass. This is the whole point of the phase: a swap of two
        // 64 px logos is impossible to miss, where a progress bar alone is easy to overlook.
        let swapped = count % 2 == 1;

        // Sub-region buffer: 16 x 200 = 3,200 bytes of the full-frame array. The y_offset
        // argument keeps embedded-graphics coordinates in full-panel space.
        let mut band = PageBuffer::new(
            &mut bw_buf[..BAND_BYTES],
            GDEM0213B74::WIDTH,
            BAND_H,
            BAND_Y,
        );
        band.clear_byte(0xFF);

        // Swap Ferris and Rust
        draw_logos(&mut band, &ferris_bmp, &rust_bmp, swapped);

        // Footer labels and separator, redrawn identically every pass. Differential mode sees no
        // change here, so they stay rock steady while the logos above them swap.
        Text::new("RP2350 Pico 2", Point::new(6, 180), small_text_style)
            .draw(&mut band)
            .unwrap();

        Text::new("epdsi SSD1680", Point::new(6, 195), small_text_style)
            .draw(&mut band)
            .unwrap();

        Line::new(Point::new(6, 203), Point::new(115, 203))
            .into_styled(style)
            .draw(&mut band)
            .unwrap();

        // Update counter label
        let mut count_buf = [0u8; 32];
        let count_str =
            format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
        Text::new(count_str, Point::new(6, 224), text_style)
            .draw(&mut band)
            .unwrap();

        // Progress bar outline
        Rectangle::new(Point::new(6, 230), Size::new(110, 14))
            .into_styled(style)
            .draw(&mut band)
            .unwrap();

        // Progress bar fill
        Rectangle::new(Point::new(8, 232), Size::new(count * 17, 10))
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(&mut band)
            .unwrap();

        // Restrict controller RAM to the band, then write the new image to Black/White RAM.
        epd.set_window(0, BAND_Y, GDEM0213B74::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, band.as_slice())
            .unwrap();

        defmt::info!(
            "Refreshing band y={}..{} (Update #{}, logos {})...",
            BAND_Y,
            BAND_Y + BAND_H - 1,
            count,
            if swapped { "swapped" } else { "normal" }
        );
        epd.refresh(&mut timer).unwrap();

        // Copy the band we just displayed into the "previous image" RAM so the next iteration
        // diffs against what is actually on the panel rather than the Phase 1 content.
        epd.set_window(0, BAND_Y, GDEM0213B74::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::RedYellow, band.as_slice())
            .unwrap();

        timer.delay_ms(1000);
    }

    defmt::info!("--- Phase 3: Full-Waveform Cleanup Pass ---");

    // Differential updates drive the pixels with a much shorter waveform than the OTP full-refresh
    // LUT, so pixels that flip white -> black during Phase 2 settle at a dark grey rather than a
    // deep black, while pixels already black from Phase 1 keep their full density. Re-running the
    // final band content through the full waveform restores even ink density across the band.
    // Blanking the secondary RAM first also stops it being interpreted as a second color plane.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Full);

    epd.set_window(0, BAND_Y, GDEM0213B74::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &WHITE_BAND)
        .unwrap();

    // `bw_buf` still holds the last band drawn in Phase 2, so re-send it unchanged.
    epd.set_window(0, BAND_Y, GDEM0213B74::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &bw_buf[..BAND_BYTES])
        .unwrap();

    defmt::info!("Refreshing band with the full OTP waveform...");
    epd.refresh(&mut timer).unwrap();

    // Restore the full-frame RAM window for any subsequent updates.
    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();

    defmt::info!("Display complete!");

    // Blink status LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
