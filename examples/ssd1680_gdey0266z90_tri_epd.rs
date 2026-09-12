//! # GDEY0266Z90 Tri-Color `PageBufferPair` Draw Target Example (`epdsi`)
//!
//! Full-parity companion to `ssd1680_gdey0266z90_epd` — same four phases, same content, same
//! timing measurements, same hardware — but drawn entirely through `PageBufferPair`/`TriColor`
//! instead of two separate `PageBuffer`s and panel-specific `BinaryColor::On`/`Off` polarity
//! choices. Exists to prove the new draw-target API can do everything the manual approach does,
//! not just a minimal smoke test.
//!
//! 1. **Phase 1**: Full tri-color refresh ([`Ssd168xRefreshMode::Full`]) — header, Black and
//!    Accent swatches, Ferris logo (Accent), Rust logo (Black), and text labels.
//! 2. **Phase 2**: Partial *window* refresh loop on the full waveform, repainting only the
//!    bottom status band with a Black counter and an Accent progress bar.
//! 3. **Phase 3**: [`Ssd168xRefreshMode::FastFull`] full-screen refresh, timed against Phase 1.
//! 4. **Phase 4**: [`Ssd168xRefreshMode::BaseMap`] and [`Ssd168xRefreshMode::Partial`], shown at
//!    their real (non-differential, on this panel) cost.
//!
//! See `ssd1680_gdey0266z90_epd` for the full narrative on refresh-mode behavior, ink polarity,
//! ink physics, duty cycle, and glass provenance — none of that changed, so it isn't repeated
//! here. What differs is only the drawing code:
//!
//! - One `page: &mut PageBufferPair` replaces the two `bw`/`red` `PageBuffer` parameters
//!   `draw_static_content`/`draw_band`/`draw_band_bar` used to take.
//! - `TriColor::{Black, Accent}` replaces every panel-polarity-aware `BinaryColor::On`/`Off`
//!   choice — no more picking `Off` "because the Red plane is inverted."
//! - `PageBufferPair::clear()` replaces the pair of `clear_byte(0xFF)` / `clear_byte(0x00)`
//!   calls before each windowed redraw — one call, no raw byte to keep in sync with polarity.
//! - The `NO_RED_BAND` static the original example wrote for the BaseMap phase (a workaround for
//!   needing to hand-compute an all-`0x00` array of the right size) is gone: `page.accent()`'s
//!   own background bytes, derived from `PlanePolarity`, already are that array.
//!
//! ## Hardware
//!
//! Same board, panel and wiring as `ssd1680_gdey0266z90_epd` — see that example for the pin
//! table and glass provenance notes.
//!
//! ## Run
//!
//! Power-cycle the board first, then run once and let it finish — this takes roughly two
//! minutes, the same as `ssd1680_gdey0266z90_epd`.
//!
//! ```bash
//! cargo run --example ssd1680_gdey0266z90_tri_epd
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
const STRIDE: usize = GDEY0266Z90::WIDTH.div_ceil(8) as usize;

/// Full frame buffer size per plane: 19 x 296 = 5,624 bytes.
const FRAME_BYTES: usize = STRIDE * GDEY0266Z90::HEIGHT as usize;

/// Top Y coordinate of the status band repainted in Phases 2 and 4. Everything above it is
/// painted in Phase 1 and never touched again, so the Accent logo stays put.
const BAND_Y: u32 = 220;

/// Height of the status band in pixels (y = 220..295).
const BAND_H: u32 = 76;

/// Status band buffer size: 19 x 76 = 1,444 bytes.
const BAND_BYTES: usize = STRIDE * BAND_H as usize;

/// The polarity this panel needs — Black/White plane normal, accent plane inverted. Passed to
/// every `PageBufferPair::new` call rather than assumed once, since a different panel could
/// need `PlanePolarity::UC8253` instead.
const POLARITY: PlanePolarity = PlanePolarity::SSD168X;

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

/// Draws the Phase 1 / Phase 3 static content: everything above the status band.
///
/// One `page` addresses both RAM planes — no separate `bw`/`red` buffers, and no
/// polarity-aware `BinaryColor` choice: `TriColor::Black`/`TriColor::Accent` say what they mean.
fn draw_static_content(
    page: &mut PageBufferPair,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
    mode_label: &str,
) {
    let stroke = PrimitiveStyle::with_stroke(TriColor::Black, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, TriColor::Black);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, TriColor::Black);
    let accent_small_text_style = MonoTextStyle::new(&FONT_6X10, TriColor::Accent);

    // Outer border (Black), so a shifted or wrapped raster is obvious.
    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT),
    )
    .into_styled(stroke)
    .draw(page)
    .unwrap();

    // Header (Black). 11 chars at 10 px each fits the 152 px width.
    Text::new("GDEY0266Z90", Point::new(8, 22), text_style)
        .draw(page)
        .unwrap();

    // Subtitle: "Tri-Color " in Black, "BWR" in Accent.
    Text::new("Tri-Color ", Point::new(8, 40), small_text_style)
        .draw(page)
        .unwrap();
    Text::new("BWR", Point::new(68, 40), accent_small_text_style)
        .draw(page)
        .unwrap();

    Line::new(Point::new(8, 48), Point::new(143, 48))
        .into_styled(stroke)
        .draw(page)
        .unwrap();

    // Colour swatches: Black left, Accent right, inside a shared outline.
    Rectangle::new(Point::new(8, 56), Size::new(136, 18))
        .into_styled(stroke)
        .draw(page)
        .unwrap();
    Rectangle::new(Point::new(10, 58), Size::new(64, 14))
        .into_styled(PrimitiveStyle::with_fill(TriColor::Black))
        .draw(page)
        .unwrap();
    Rectangle::new(Point::new(78, 58), Size::new(64, 14))
        .into_styled(PrimitiveStyle::with_fill(TriColor::Accent))
        .draw(page)
        .unwrap();

    // Ferris (64x42) in Accent and Rust (64x64) in Black, side by side — both drawn straight
    // onto the same `page`, unlike the original example's separate `red`/`bw` targets.
    let ferris_pos = Point::new(10, 92);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, TriColor::Accent)
                .draw(page)
                .unwrap();
        }
    }

    let rust_pos = Point::new(78, 82);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, TriColor::Black)
                .draw(page)
                .unwrap();
        }
    }

    // Labels (Black).
    Text::new("RP2350 Pico 2", Point::new(8, 170), small_text_style)
        .draw(page)
        .unwrap();
    Text::new("epdsi PageBufferPair", Point::new(8, 184), small_text_style)
        .draw(page)
        .unwrap();
    Text::new(mode_label, Point::new(8, 198), small_text_style)
        .draw(page)
        .unwrap();

    // Separator above the status band that Phases 2 and 4 repaint.
    Line::new(Point::new(8, 210), Point::new(143, 210))
        .into_styled(stroke)
        .draw(page)
        .unwrap();
}

/// Writes both colour planes for the full frame, resetting the RAM window and cursor first.
fn write_full_frame<BUS, C, P>(epd: &mut EpdDriver<BUS, C, P>, page: &PageBufferPair)
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
{
    epd.set_window(0, 0, GDEY0266Z90::WIDTH - 1, GDEY0266Z90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, page.bw().as_slice())
        .unwrap();

    epd.set_window(0, 0, GDEY0266Z90::WIDTH - 1, GDEY0266Z90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, page.accent().as_slice())
        .unwrap();
}

/// Draws the status band's Black content: label, counter and progress bar outline.
fn draw_band(page: &mut PageBufferPair, count: u32, label: &str) {
    let stroke = PrimitiveStyle::with_stroke(TriColor::Black, 1);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, TriColor::Black);

    Text::new(label, Point::new(8, BAND_Y as i32 + 14), small_text_style)
        .draw(page)
        .unwrap();

    let mut count_buf = [0u8; 32];
    let count_str = format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
    Text::new(
        count_str,
        Point::new(8, BAND_Y as i32 + 28),
        small_text_style,
    )
    .draw(page)
    .unwrap();

    // Progress bar outline (Black); the fill is a separate call, always Accent.
    Rectangle::new(Point::new(8, BAND_Y as i32 + 38), Size::new(136, 16))
        .into_styled(stroke)
        .draw(page)
        .unwrap();
}

/// Draws the progress bar fill for `count`, always in Accent — the original example's
/// `draw_band_bar` took a `BinaryColor` parameter, but every call site passed the same value
/// (`BinaryColor::Off`, targeting the Red plane), so there was nothing for that parameter to
/// vary; `TriColor::Accent` is simply hardcoded here instead.
fn draw_band_bar(page: &mut PageBufferPair, count: u32) {
    Rectangle::new(
        Point::new(10, BAND_Y as i32 + 40),
        Size::new(count * 33, 12),
    )
    .into_styled(PrimitiveStyle::with_fill(TriColor::Accent))
    .draw(page)
    .unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!(
        "Starting GDEY0266Z90 2.66\" Tri-Color EPD example (epdsi SSD1680, PageBufferPair)"
    );
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

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1680 controller.
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1680Controller::new(GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT)
        .with_refresh_mode(Ssd168xRefreshMode::Full);

    // Build EPD Driver using epdsi with GDEY0266Z90 panel specification (152x296)
    let mut epd = EpdBuilder::<_, GDEY0266Z90>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1680 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Frame buffers: 5,624 bytes each, pre-filled to each plane's own background byte under
    // POLARITY — no `clear_frame` call needed, since Phase 1's draw covers the whole panel.
    let mut bw_buf = [POLARITY.bw_background_byte(); FRAME_BYTES];
    let mut red_buf = [POLARITY.accent_background_byte(); FRAME_BYTES];

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    defmt::info!("--- Phase 1: Full Tri-Color Refresh ---");

    // Scoped so the full-frame borrows end before Phase 2 re-borrows them as band buffers.
    let full_ms = {
        let mut page = PageBufferPair::new(
            &mut bw_buf,
            &mut red_buf,
            GDEY0266Z90::WIDTH,
            GDEY0266Z90::HEIGHT,
            0,
            POLARITY,
        );

        draw_static_content(&mut page, &ferris_bmp, &rust_bmp, "mode: Full");

        defmt::info!("Sending both planes ({} bytes each)...", FRAME_BYTES);
        write_full_frame(&mut epd, &page);

        timed_refresh(&mut epd, &mut timer, "Phase 1 (Full)")
    };

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Windowed Refresh on the Full Waveform ---");

    // Colour panels have no differential waveform, so a region update is not a speed-up — it is
    // a full refresh over a smaller area. Both planes must be written for the window.
    for count in 1..=2u32 {
        {
            let mut band = PageBufferPair::new(
                &mut bw_buf[..BAND_BYTES],
                &mut red_buf[..BAND_BYTES],
                GDEY0266Z90::WIDTH,
                BAND_H,
                BAND_Y,
                POLARITY,
            );
            band.clear();

            draw_band(&mut band, count, "Full window");
            draw_band_bar(&mut band, count);

            epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
                .unwrap();
            epd.set_cursor(0, BAND_Y).unwrap();
            epd.write_frame(ColorChannel::BlackWhite, band.bw().as_slice())
                .unwrap();
            epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
                .unwrap();
            epd.set_cursor(0, BAND_Y).unwrap();
            epd.write_frame(ColorChannel::RedYellow, band.accent().as_slice())
                .unwrap();
        }

        defmt::info!("Refreshing band y={}..{}...", BAND_Y, BAND_Y + BAND_H - 1);
        timed_refresh(&mut epd, &mut timer, "Phase 2 (windowed Full)");
        timer.delay_ms(1000);
    }

    defmt::info!("--- Phase 3: FastFull Full-Screen Refresh ---");

    // Re-render the Phase 1 content, this time on the temperature-override waveform, and compare.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::FastFull);

    let fast_ms = {
        let mut page = PageBufferPair::new(
            &mut bw_buf,
            &mut red_buf,
            GDEY0266Z90::WIDTH,
            GDEY0266Z90::HEIGHT,
            0,
            POLARITY,
        );
        page.clear();

        draw_static_content(&mut page, &ferris_bmp, &rust_bmp, "mode: FastFull");

        write_full_frame(&mut epd, &page);

        timed_refresh(&mut epd, &mut timer, "Phase 3 (FastFull)")
    };

    defmt::info!(
        "Full {} ms vs FastFull {} ms. Reference for this glass is 20048 vs 16180 (~19% faster). \
         Good Display quote ~20000 vs ~19000 on their own glass, so expect the saving to vary \
         with the OTP waveform rather than assuming either figure.",
        full_ms,
        fast_ms
    );

    timer.delay_ms(2000);

    defmt::info!("--- Phase 4: BaseMap and Partial (both full-waveform on this panel) ---");

    // Both planes are written for every pass here, exactly as in Phases 1-3. There is no
    // previous-frame seeding: on a Tri-Color panel 0x26 is *always* the colour plane. Measured
    // on hardware, both modes below take ~19.9 s, the same as Full.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::BaseMap);

    {
        let mut band = PageBufferPair::new(
            &mut bw_buf[..BAND_BYTES],
            &mut red_buf[..BAND_BYTES],
            GDEY0266Z90::WIDTH,
            BAND_H,
            BAND_Y,
            POLARITY,
        );
        band.clear();

        draw_band(&mut band, 0, "BaseMap");
        // count == 0 draws a zero-width rectangle — a no-op, kept only for structural parity
        // with Phase 2/the Partial loop below.
        draw_band_bar(&mut band, 0);

        epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, band.bw().as_slice())
            .unwrap();
        epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        // No separate "empty red band" constant needed: `band.accent()` is already all
        // background bytes, since nothing drew Accent content into it above.
        epd.write_frame(ColorChannel::RedYellow, band.accent().as_slice())
            .unwrap();
    }

    timed_refresh(&mut epd, &mut timer, "Phase 4 (BaseMap)");

    timer.delay_ms(1000);

    // Partial selects the controller's built-in fast LUT (0x22 = 0xFC). That LUT exists only for
    // monochrome panels, so here it is neither fast nor differential — measured at 19.9 s, the
    // same as Full. It is included to show what the mode actually does on colour glass, and it
    // still needs both planes written or the Accent content in the window is dropped.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Partial);

    for count in 1..=2u32 {
        {
            let mut band = PageBufferPair::new(
                &mut bw_buf[..BAND_BYTES],
                &mut red_buf[..BAND_BYTES],
                GDEY0266Z90::WIDTH,
                BAND_H,
                BAND_Y,
                POLARITY,
            );
            band.clear();

            draw_band(&mut band, count, "Partial mode");
            draw_band_bar(&mut band, count);

            epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
                .unwrap();
            epd.set_cursor(0, BAND_Y).unwrap();
            epd.write_frame(ColorChannel::BlackWhite, band.bw().as_slice())
                .unwrap();
            epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
                .unwrap();
            epd.set_cursor(0, BAND_Y).unwrap();
            epd.write_frame(ColorChannel::RedYellow, band.accent().as_slice())
                .unwrap();
        }

        defmt::info!("Partial-mode update #{}...", count);
        timed_refresh(&mut epd, &mut timer, "Phase 4 (Partial)");

        timer.delay_ms(1000);
    }

    // Restore the full-frame RAM window and the default waveform for any subsequent updates.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Full);
    epd.set_window(0, 0, GDEY0266Z90::WIDTH - 1, GDEY0266Z90::HEIGHT - 1)
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
