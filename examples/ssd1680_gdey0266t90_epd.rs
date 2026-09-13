//! # Good Display GDEY0266T90 2.66" Monochrome E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the **GDEY0266T90** 2.66"
//! Monochrome (Black/White, 152x296) E-Paper Display using the `Ssd1680Controller` from the
//! `epdsi` library.
//!
//! This is a **different panel** from the Tri-Color `GDEY0266Z90` this repo's
//! `ssd1680_gdey0266z90_epd` example drives — same nominal size and controller, but a
//! monochrome-only glass, not a config of the color one. Unlike the Tri-Color sibling, this panel
//! is genuinely fast: real Full and Partial (differential) refresh, so the phases below follow
//! `ssd1680_gdem0213b74_epd`/`uc8253_gdey037t03_epd`'s structure exactly — Full, then a
//! partial-window loop that swaps two logos on every pass, then a full-waveform cleanup pass —
//! rather than each panel improvising its own shape.
//!
//! `Ssd168xRefreshMode::FastFull` is also available on this controller (see
//! `ssd1680_gdey0266z90_epd` for that mode's temperature-override mechanics), but is deliberately
//! **not** exercised here: an earlier revision of this example added a fourth "FastFull timing
//! comparison" phase between the partial loop and the cleanup pass, and it introduced a real bug
//! — the cleanup pass re-sent `bw_buf`'s first `BAND_BYTES` bytes assuming they still held the
//! partial loop's last *band-relative* content, but the FastFull phase in between had just
//! overwritten the whole buffer as *frame-relative* content, so the cleanup pass painted a
//! `BAND_Y`-pixel-shifted duplicate of the header over the bottom of the panel. Matching the other
//! two panels' 3-phase shape removes the bug at its root, not just its symptom.
//!
//! Demonstrates:
//! 1. **Phase 1**: Full monochrome refresh — header, side-by-side Ferris/Rust logos, footer
//!    labels, and a status line. Seeds the secondary RAM (`0x26`) with the same image so Phase
//!    2's differential update has a correct base to diff against.
//! 2. **Phase 2**: Fast *differential* partial-window refresh loop over the content band (logos
//!    through the bottom status line), swapping the Ferris and Rust logos on every pass and
//!    advancing a progress bar — the same "logo swap" idiom `ssd1680_gdem0213b74_epd` uses (there,
//!    the two logos are stacked and swap top/bottom because its 122px panel is too narrow for
//!    them side by side; here, at 152px, they swap left/right instead).
//! 3. **Phase 3**: Full-waveform cleanup pass over the whole content band (logos, footer and
//!    status line), restoring the ink density the shortened Phase 2 differential waveform leaves
//!    behind — the same idiom `ssd1680_gdem0213b74_epd`/`uc8253_gdey037t03_epd` use.
//!
//! ## Note on refresh speed
//!
//! `GxEPD2_266_GDEY0266T90`'s reference driver quotes `full_refresh_time = 1700` ms and
//! `partial_refresh_time = 500` ms — an order of magnitude faster than the Tri-Color
//! `GDEY0266Z90` (~20 s), because there is no red pigment waveform to drive. Measured on hardware,
//! `Full` and `Partial` both took ~4.1-4.2 s here — nowhere near the reference figures, and
//! `Partial` was no faster than `Full` at all. That gap is worth investigating on its own before
//! trusting this panel's `Partial` mode as a real speed-up; this example logs its own measured
//! timings so the discrepancy is visible rather than assumed away.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Good Display GDEY0266T90 / Waveshare 2.66" e-Paper (SKU 18401, FPC-7510 REV.C)
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
//! cargo run --example ssd1680_gdey0266t90_epd
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

/// Row stride in bytes. 152 px is byte-aligned, so this is exactly 19 with no padding.
const STRIDE: usize = GDEY0266T90::WIDTH.div_ceil(8) as usize;

/// Full frame buffer size: 19 x 296 = 5,624 bytes.
const FRAME_BYTES: usize = STRIDE * GDEY0266T90::HEIGHT as usize;

/// Top Y coordinate of the content band repainted in Phases 2 and 3 — everything from just below
/// the title/subtitle separator down to the bottom of the panel, so the logo swap, footer labels
/// and status line are all inside the partial-refresh window. Only the border, title and subtitle
/// above it are painted once in Phase 1 and never touched again.
const BAND_Y: u32 = 52;

/// Height of the content band in pixels (y = 52..295).
const BAND_H: u32 = GDEY0266T90::HEIGHT - BAND_Y;

/// Content band buffer size: 19 x 244 = 4,636 bytes.
const BAND_BYTES: usize = STRIDE * BAND_H as usize;

/// X coordinate of the left logo slot.
const LOGO_X_LEFT: i32 = 10;

/// X coordinate of the right logo slot.
const LOGO_X_RIGHT: i32 = 78;

/// Ferris's own Y offset (64x42 — shorter than Rust, so it sits a little lower to bottom-align).
const FERRIS_Y: i32 = 92;

/// Rust's own Y offset (64x64).
const RUST_Y: i32 = 82;

/// All-white fill for the content band's secondary RAM, used to blank the "previous image" buffer
/// during the Phase 3 cleanup pass. Lives in flash rather than on the stack.
static WHITE_BAND: [u8; BAND_BYTES] = [0xFFu8; BAND_BYTES];

/// Refreshes the panel, reporting how long it took and returning the elapsed milliseconds.
fn timed_refresh<BUS, C, P>(
    epd: &mut EpdDriver<BUS, C, P>,
    timer: &mut hal::Timer<hal::timer::CopyableTimer0>,
    label: &str,
) -> u64
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
{
    let start = timer.get_counter().ticks();
    epd.refresh(timer).unwrap();
    let elapsed_ms = (timer.get_counter().ticks() - start) / 1000;
    defmt::info!("{}: refresh took {} ms", label, elapsed_ms);
    elapsed_ms
}

/// Draws Ferris and Rust side by side (152 px fits both 64 px-wide logos, unlike the 122 px
/// `GDEM0213B74` where they have to be stacked). `swapped` exchanges which logo occupies the
/// left slot: Phase 2 flips it on every partial update, the same idiom `ssd1680_gdem0213b74_epd`
/// uses for its stacked top/bottom swap.
fn draw_logos(
    display: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
    swapped: bool,
) {
    let (ferris_x, rust_x) = if swapped {
        (LOGO_X_RIGHT, LOGO_X_LEFT)
    } else {
        (LOGO_X_LEFT, LOGO_X_RIGHT)
    };

    let ferris_pos = Point::new(ferris_x, FERRIS_Y);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }

    let rust_pos = Point::new(rust_x, RUST_Y);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, BinaryColor::On)
                .draw(display)
                .unwrap();
        }
    }
}

/// Draws the footer labels, mode line and the separator above them — identical every time it is
/// called, so Phase 2 can redraw it unchanged inside the content band alongside the swapped logos.
fn draw_footer(display: &mut PageBuffer, mode_label: &str) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::new("RP2350 Pico 2", Point::new(8, 170), small_text_style)
        .draw(display)
        .unwrap();
    Text::new("epdsi SSD1680", Point::new(8, 184), small_text_style)
        .draw(display)
        .unwrap();
    Text::new(mode_label, Point::new(8, 198), small_text_style)
        .draw(display)
        .unwrap();

    // Separator above the status line that Phase 2's counter/bar sits below.
    Line::new(Point::new(8, 210), Point::new(143, 210))
        .into_styled(stroke)
        .draw(display)
        .unwrap();
}

/// Draws the Phase 1 static content: border, title, subtitle, logos (never swapped here — only
/// Phase 2 swaps them) and footer.
fn draw_static_content(
    display: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
    mode_label: &str,
) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Outer border, so a shifted or wrapped raster is obvious.
    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEY0266T90::WIDTH, GDEY0266T90::HEIGHT),
    )
    .into_styled(stroke)
    .draw(display)
    .unwrap();

    Text::new("GDEY0266T90", Point::new(8, 22), text_style)
        .draw(display)
        .unwrap();

    Text::new("2.66\" Mono", Point::new(8, 40), small_text_style)
        .draw(display)
        .unwrap();

    Line::new(Point::new(8, 48), Point::new(143, 48))
        .into_styled(stroke)
        .draw(display)
        .unwrap();

    draw_logos(display, ferris_bmp, rust_bmp, false);
    draw_footer(display, mode_label);
}

/// Writes the full frame to Black/White RAM, then seeds the secondary RAM with the same image so
/// it is a correct differential base for the Phase 2 partial updates that follow.
fn write_full_frame<BUS, C, P>(epd: &mut EpdDriver<BUS, C, P>, data: &[u8])
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
{
    epd.set_window(0, 0, GDEY0266T90::WIDTH - 1, GDEY0266T90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, data).unwrap();

    epd.set_window(0, 0, GDEY0266T90::WIDTH - 1, GDEY0266T90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, data).unwrap();
}

/// Draws the status line: label, counter and progress bar. Fixed at an absolute Y position —
/// deliberately independent of [`BAND_Y`], which is just the partial-refresh window's top edge,
/// not where content starts. This sits well inside that window, below the logos and footer.
fn draw_band(band: &mut PageBuffer, count: u32, label: &str) {
    const STATUS_Y: i32 = 220;

    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::new(label, Point::new(8, STATUS_Y + 14), small_text_style)
        .draw(band)
        .unwrap();

    let mut count_buf = [0u8; 32];
    let count_str = format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
    Text::new(count_str, Point::new(8, STATUS_Y + 28), small_text_style)
        .draw(band)
        .unwrap();

    Rectangle::new(Point::new(8, STATUS_Y + 38), Size::new(136, 16))
        .into_styled(stroke)
        .draw(band)
        .unwrap();

    // Capped at 132: the outline above is 136px wide starting at x=8, the fill starts 2px in at
    // x=10, so 132 lands the fill's right edge 2px inside the outline's, symmetric with the left
    // inset. `count * 33` alone overshoots that at count=5 (165px) — past the outline *and* past
    // the panel's own 152px width — which is exactly the overflowing bar seen on real hardware.
    Rectangle::new(
        Point::new(10, STATUS_Y + 40),
        Size::new((count * 33).min(132), 12),
    )
    .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
    .draw(band)
    .unwrap();
}

/// Redraws the outer border's left, right and bottom edges for this band's row range.
///
/// Phase 1 draws the full-panel border once, but the content band's `clear_byte` + full redraw
/// on every Phase 2 pass (and the Phase 3 cleanup resend) wipes out whatever of that border falls
/// within the band — everything except the sliver above [`BAND_Y`], which is never touched. Without
/// this, the border only ever appears around the title and looks disconnected from the rest of the
/// content below it.
fn draw_band_border(band: &mut PageBuffer) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let bottom = GDEY0266T90::HEIGHT as i32 - 1;
    let right = GDEY0266T90::WIDTH as i32 - 1;

    Line::new(Point::new(0, BAND_Y as i32), Point::new(0, bottom))
        .into_styled(stroke)
        .draw(band)
        .unwrap();
    Line::new(Point::new(right, BAND_Y as i32), Point::new(right, bottom))
        .into_styled(stroke)
        .draw(band)
        .unwrap();
    Line::new(Point::new(0, bottom), Point::new(right, bottom))
        .into_styled(stroke)
        .draw(band)
        .unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEY0266T90 2.66\" Monochrome EPD example (epdsi SSD1680)");
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

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1680 controller. No variant selection
    // needed: this panel shares the default SSD1680 register profile with GDEM0213B74/GDEY0266Z90.
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1680Controller::new(GDEY0266T90::WIDTH, GDEY0266T90::HEIGHT)
        .with_refresh_mode(Ssd168xRefreshMode::Full);
    let mut epd = EpdBuilder::<_, GDEY0266T90>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1680 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Both RAM banks start white. On this monochrome panel the secondary RAM (0x26) is the
    // "previous image" buffer used by differential updates, not a color plane.
    epd.clear_frame(ColorChannel::BlackWhite, 0xFF).unwrap();
    epd.clear_frame(ColorChannel::RedYellow, 0xFF).unwrap();

    let mut bw_buf = [0xFFu8; FRAME_BYTES];

    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    defmt::info!("--- Phase 1: Full Monochrome Refresh ---");

    {
        let mut display = PageBuffer::new(&mut bw_buf, GDEY0266T90::WIDTH, GDEY0266T90::HEIGHT, 0);
        draw_static_content(&mut display, &ferris_bmp, &rust_bmp, "mode: Full");

        defmt::info!("Sending frame ({} bytes)...", FRAME_BYTES);
        write_full_frame(&mut epd, display.as_slice());

        timed_refresh(&mut epd, &mut timer, "Phase 1 (Full)");
    };

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Fast Partial Window Refresh (logo swap) ---");

    // Select the SSD1680 built-in fast LUT (0x22 = 0xFC). Unlike the Tri-Color GDEY0266Z90, this
    // is a genuine differential update on this monochrome panel and should complete in well under
    // a second.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Partial);

    for count in 1..=5u32 {
        // Flip the logo order on every pass — same idiom `ssd1680_gdem0213b74_epd` and
        // `uc8253_gdey037t03_epd` use, just left/right instead of top/bottom.
        let swapped = count % 2 == 1;

        let mut band = PageBuffer::new(
            &mut bw_buf[..BAND_BYTES],
            GDEY0266T90::WIDTH,
            BAND_H,
            BAND_Y,
        );
        band.clear_byte(0xFF);
        draw_band_border(&mut band);
        draw_logos(&mut band, &ferris_bmp, &rust_bmp, swapped);
        draw_footer(&mut band, "mode: Full");
        draw_band(&mut band, count, "Fast partial");

        // Restrict controller RAM to the band, write the new image to Black/White RAM.
        epd.set_window(0, BAND_Y, GDEY0266T90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, band.as_slice())
            .unwrap();

        let ms = timed_refresh(&mut epd, &mut timer, "Phase 2 (Partial)");
        defmt::info!(
            "Update #{}: {} ms (logos {})",
            count,
            ms,
            if swapped { "swapped" } else { "normal" }
        );

        // Copy the band we just displayed into the "previous image" RAM so the next iteration
        // diffs against what is actually on the panel.
        epd.set_window(0, BAND_Y, GDEY0266T90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::RedYellow, band.as_slice())
            .unwrap();

        timer.delay_ms(500);
    }

    defmt::info!("--- Phase 3: Full-Waveform Cleanup Pass ---");

    // Differential updates drive the pixels with a shorter waveform than the OTP full-refresh
    // LUT, so ink density can drift after several Phase 2 passes. Re-running the final band
    // content through the full waveform restores even density. Blanking the secondary RAM
    // first stops it being read as a stale differential base afterwards.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Full);

    epd.set_window(0, BAND_Y, GDEY0266T90::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &WHITE_BAND)
        .unwrap();

    // `bw_buf` still holds the last band drawn in Phase 2 — nothing has touched it since, so
    // re-sending it unchanged is safe. (This is exactly the assumption that broke when an earlier
    // revision inserted a FastFull full-frame phase here: see the module doc.)
    epd.set_window(0, BAND_Y, GDEY0266T90::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &bw_buf[..BAND_BYTES])
        .unwrap();

    timed_refresh(&mut epd, &mut timer, "Phase 3 (cleanup)");

    // Restore the full-frame RAM window and the default waveform for any subsequent updates.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Full);
    epd.set_window(0, 0, GDEY0266T90::WIDTH - 1, GDEY0266T90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();

    // Deep sleep. init() must be called again before any further frame.
    epd.sleep(&mut timer).unwrap();
    defmt::info!("Display complete, controller asleep.");

    // Blink status LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
