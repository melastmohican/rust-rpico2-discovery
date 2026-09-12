//! # SE0352N14TNGA0 Tri-Color `PageBufferPair` Draw Target Example (`epdsi`)
//!
//! Full-parity companion to `uc8253_se0352n14_epd` — same two phases, same content, same
//! hardware — but drawn entirely through `PageBufferPair`/`TriColor` instead of two separate
//! `PageBuffer`s and the panel-specific `INK = BinaryColor::Off` alias.
//!
//! 1. **Phase 1**: A white | black | red panel test that makes miswiring self-evident.
//! 2. **Phase 2**: Full Tri-Color refresh showing a header, the Rust logo in black, the Ferris
//!    logo in red, text labels and an accent bar.
//!
//! See `uc8253_se0352n14_epd` for the full narrative on this panel's quirks (swapped RAM plane
//! routing vs. `GDEY037T03`, inverted ink polarity on *both* planes, no partial/fast waveform,
//! the charge-pump `POWER_ON` requirement, active-low BUSY, and native orientation). None of
//! that changed — this file only differs in the drawing code:
//!
//! - `PlanePolarity::UC8253` replaces the `INK`/`WHITE_BYTE` constants: this is the one panel
//!   `epdsi` ships where *both* planes are inverted (a set bit is ink), unlike SSD168x where
//!   only the accent plane is. Getting the wrong constant here is exactly the mistake
//!   `PlanePolarity` exists to catch — see its docs.
//! - One shared `PageBufferPair` (built once per phase via `frame()`) replaces the two
//!   independently-rotated `PageBuffer`s the original built per phase. The original's own doc
//!   comment warned that going through anything other than one shared constructor risks
//!   `DisplayRotation::Rotate180` landing on three planes but not a fourth; going from four
//!   `frame()` call sites (two phases x two planes) to two (one per phase, both planes together)
//!   removes that risk structurally rather than by discipline.
//! - `blit()` takes a `TriColor` destination color as well as the `BinaryColor` source-pixel
//!   test, since which RAM plane a logo lands on is now a draw-time choice rather than which
//!   `PageBuffer` it was handed.
//!
//! ## Hardware
//!
//! Same board, panel and wiring as `uc8253_se0352n14_epd` — see that example for the pin table
//! and glass provenance notes.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example uc8253_se0352n14_tri_epd
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

/// This panel's polarity: *both* planes are inverted (a set bit is ink), unlike SSD168x where
/// only the accent plane is. See the module doc.
const POLARITY: PlanePolarity = PlanePolarity::UC8253;

/// Row stride in bytes: 240 / 8 = 30. This panel is already byte-aligned.
const STRIDE: usize = SE0352N14TNGA0::WIDTH.div_ceil(8) as usize;

/// Frame buffer size for one plane: 30 x 360 = 10,800 bytes. Two planes are allocated, so this
/// example holds 21,600 bytes of frame buffer.
const FRAME_BYTES: usize = STRIDE * SE0352N14TNGA0::HEIGHT as usize;

/// X coordinate of the left logo slot.
const LOGO_LEFT_X: i32 = 30;

/// X coordinate of the right logo slot.
const LOGO_RIGHT_X: i32 = 146;

/// Builds a full-frame drawing surface addressing both RAM planes, rotated to this repo's
/// convention.
///
/// This panel's native `(0,0)` is top-left with the FPC ribbon at the **top**, but every other
/// EPD example here renders with the ribbon at the **bottom**, so the surface is rotated 180°.
/// Going through one constructor for both planes together — rather than one per plane, as the
/// manual `PageBuffer` approach needs — makes "rotation applied to one plane but not the other"
/// structurally impossible instead of a discipline to maintain.
fn frame<'a>(bw_buf: &'a mut [u8], accent_buf: &'a mut [u8]) -> PageBufferPair<'a> {
    let mut page = PageBufferPair::new(
        bw_buf,
        accent_buf,
        SE0352N14TNGA0::WIDTH,
        SE0352N14TNGA0::HEIGHT,
        0,
        POLARITY,
    );
    page.set_rotation(DisplayRotation::Rotate180);
    page
}

/// Draws a BMP into `page` as `color`, treating source pixels equal to `source_ink` as ink.
///
/// The two bundled logos disagree on polarity: `ferrisbw.bmp` marks its subject with
/// `BinaryColor::Off` and `rustbw.bmp` with `BinaryColor::On` — that is a fact about the BMP
/// files, unrelated to which RAM plane `color` routes to.
fn blit(
    page: &mut PageBufferPair,
    bmp: &Bmp<BinaryColor>,
    origin: Point,
    source_ink: BinaryColor,
    color: TriColor,
) {
    for Pixel(point, pixel_color) in bmp.pixels() {
        if pixel_color == source_ink {
            Pixel(point + origin, color).draw(page).unwrap();
        }
    }
}

/// Draws the panel-test Black content: title, separator, the pattern frame, the black band and
/// the band labels.
fn draw_test_black_content(page: &mut PageBufferPair) {
    let text_style = MonoTextStyle::new(&FONT_10X20, TriColor::Black);
    let stroke = PrimitiveStyle::with_stroke(TriColor::Black, 1);

    Text::new("PLANE TEST", Point::new(10, 24), text_style)
        .draw(page)
        .unwrap();

    Line::new(Point::new(10, 34), Point::new(229, 34))
        .into_styled(stroke)
        .draw(page)
        .unwrap();

    // Black frame around the whole pattern. Without it the white band has no visible edge at all
    // and the pattern reads as two bands on a blank panel rather than three. Outer edges land on
    // x = 11/228 and y = 50/289, leaving a 216 x 238 interior.
    Rectangle::new(Point::new(11, 50), Size::new(218, 240))
        .into_styled(stroke)
        .draw(page)
        .unwrap();

    // Middle third solid black, inset by one pixel so it abuts the frame rather than overlapping
    // it. The left third is left bare, so it shows the panel's white.
    Rectangle::new(Point::new(84, 51), Size::new(72, 238))
        .into_styled(PrimitiveStyle::with_fill(TriColor::Black))
        .draw(page)
        .unwrap();

    // One label centred under each band (interior thirds are 72 px, so centres are x = 48, 120,
    // 192; FONT_10X20 glyphs are 10 px wide). The frame stops at y = 289, so these sit on white
    // and stay legible whichever plane misbehaves.
    for (label, centre_x) in [("W", 48), ("B", 120), ("R", 192)] {
        Text::new(label, Point::new(centre_x - 5, 320), text_style)
            .draw(page)
            .unwrap();
    }
}

/// Draws the panel-test Accent content: the right-hand red band.
///
/// Must not touch the frame drawn in [`draw_test_black_content`] — the two thirds are disjoint
/// (x = 84..156 Black, x = 156..228 Accent), so drawing order doesn't matter here, but drawing
/// Accent over a Black pixel would clear that pixel's Black-plane bit back to background (see
/// `PageBufferPair::set_pixel`'s docs on why redrawing a pixel clears the other plane).
fn draw_test_accent_content(page: &mut PageBufferPair) {
    Rectangle::new(Point::new(156, 51), Size::new(72, 238))
        .into_styled(PrimitiveStyle::with_fill(TriColor::Accent))
        .draw(page)
        .unwrap();
}

/// Draws the Phase 2 content's Black half: border, header, separators, labels and the Rust logo.
fn draw_black_content(page: &mut PageBufferPair, rust_bmp: &Bmp<BinaryColor>) {
    let stroke = PrimitiveStyle::with_stroke(TriColor::Black, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, TriColor::Black);

    Rectangle::new(
        Point::new(0, 0),
        Size::new(SE0352N14TNGA0::WIDTH, SE0352N14TNGA0::HEIGHT),
    )
    .into_styled(stroke)
    .draw(page)
    .unwrap();

    Text::new("SE0352N14", Point::new(10, 24), text_style)
        .draw(page)
        .unwrap();

    Text::new("3.52\" BWR", Point::new(10, 48), text_style)
        .draw(page)
        .unwrap();

    Line::new(Point::new(10, 58), Point::new(229, 58))
        .into_styled(stroke)
        .draw(page)
        .unwrap();

    blit(
        page,
        rust_bmp,
        Point::new(LOGO_RIGHT_X, 90),
        BinaryColor::On,
        TriColor::Black,
    );

    Text::new("RP2350 Pico 2", Point::new(10, 200), text_style)
        .draw(page)
        .unwrap();

    Text::new("epdsi PageBufferPair", Point::new(10, 225), text_style)
        .draw(page)
        .unwrap();

    Line::new(Point::new(10, 240), Point::new(229, 240))
        .into_styled(stroke)
        .draw(page)
        .unwrap();

    Text::new("Waveshare (B)", Point::new(10, 285), text_style)
        .draw(page)
        .unwrap();
}

/// Draws the Phase 2 content's Accent half: the Ferris logo and an accent bar.
fn draw_accent_content(page: &mut PageBufferPair, ferris_bmp: &Bmp<BinaryColor>) {
    blit(
        page,
        ferris_bmp,
        Point::new(LOGO_LEFT_X, 112),
        BinaryColor::Off,
        TriColor::Accent,
    );

    Rectangle::new(Point::new(10, 300), Size::new(220, 30))
        .into_styled(PrimitiveStyle::with_fill(TriColor::Accent))
        .draw(page)
        .unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!(
        "Starting SE0352N14TNGA0 3.52\" Tri-Color EPD example (epdsi UC8253, PageBufferPair)"
    );
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

    // The variant is not optional. The default Gdey037t03 profile's init, plane order and CDI
    // value are all wrong for this panel, and it renders inverted or blank rather than erroring.
    let controller = Uc8253Controller::new(SE0352N14TNGA0::WIDTH, SE0352N14TNGA0::HEIGHT)
        .with_variant(Uc8253Variant::Se0352n14);
    let mut epd = EpdBuilder::<_, SE0352N14TNGA0>::new(controller).build(epd_bus);

    defmt::info!("Initializing UC8253 epdsi EPD driver (Se0352n14 profile)...");
    epd.init(&mut timer).unwrap();

    // Both planes start white — PlanePolarity::UC8253's background byte for both, 0x00, not the
    // monochrome UC8253 panel's 0xFF.
    epd.clear_frame(ColorChannel::BlackWhite, POLARITY.bw_background_byte())
        .unwrap();
    epd.clear_frame(ColorChannel::RedYellow, POLARITY.accent_background_byte())
        .unwrap();

    let mut bw_buf = [POLARITY.bw_background_byte(); FRAME_BYTES];
    let mut red_buf = [POLARITY.accent_background_byte(); FRAME_BYTES];

    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    defmt::info!("--- Phase 1: Panel Test ---");

    // The buffers start white, so nothing to clear yet.
    {
        let mut page = frame(&mut bw_buf, &mut red_buf);
        draw_test_black_content(&mut page);
        draw_test_accent_content(&mut page);
    }

    // No set_window: a full-frame write must not be wrapped in a partial-window session, and
    // this panel has no partial mode anyway. Leaving the window unset keeps the SPI stream
    // identical to Waveshare's reference driver.
    defmt::info!("Sending diagnostic pattern (white | black | red)...");
    epd.write_frame(ColorChannel::BlackWhite, &bw_buf).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &red_buf).unwrap();

    defmt::info!("Refreshing display hardware (full waveform, expect ~16-20 s)...");
    refresh_timed(&mut epd, &mut timer);

    defmt::info!("Bands left to right should read white | black | red.");
    defmt::info!("  black/red swapped -> plane routing crossed (0x10/0x13)");
    defmt::info!("  white/black swapped -> CDI/DDX polarity or buffer base wrong");
    defmt::info!("  cleared panel looking grey -> normal white point, not a fault");

    timer.delay_ms(3000);

    defmt::info!("--- Phase 2: Full Tri-Color Refresh ---");

    // Drawn second so the panel is left showing the picture rather than the test pattern,
    // matching the other EPD examples in this repo.
    {
        let mut page = frame(&mut bw_buf, &mut red_buf);
        page.clear();
        draw_black_content(&mut page, &rust_bmp);
        draw_accent_content(&mut page, &ferris_bmp);
    }

    defmt::info!("Sending both planes (10,800 bytes each)...");
    epd.write_frame(ColorChannel::BlackWhite, &bw_buf).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &red_buf).unwrap();

    defmt::info!("Refreshing display hardware (full waveform, expect ~16-20 s)...");
    refresh_timed(&mut epd, &mut timer);

    // After sleep the controller is in deep sleep: init() must be called again before drawing.
    epd.sleep(&mut timer).unwrap();
    defmt::info!("Display complete, panel asleep!");

    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}

/// Refreshes the panel and reports how long it took.
///
/// A full refresh on this panel is 16-20 s. Anything under a second means BUSY was never
/// observed asserted — most likely it had not gone low yet when the poll started, so the
/// refresh is still running and a following `sleep` would land mid-update.
fn refresh_timed<BUS, C, P, D>(epd: &mut EpdDriver<BUS, C, P>, timer: &mut D)
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
    D: DelayNs + FnCounter,
{
    let start = timer.now_us();
    epd.refresh(timer).unwrap();
    let elapsed_ms = (timer.now_us().saturating_sub(start)) / 1_000;

    defmt::info!("Refresh returned after {} ms", elapsed_ms);
    if elapsed_ms < 1_000 {
        defmt::warn!(
            "Refresh took {} ms, expected ~16-20 s. BUSY likely had not asserted when polling \
             started; a guard delay between DISPLAY_REFRESH and the busy poll is the known fix.",
            elapsed_ms
        );
    }
}

/// Minimal microsecond clock, so `refresh_timed` can stay generic over the delay provider.
trait FnCounter {
    fn now_us(&self) -> u64;
}

impl FnCounter for hal::Timer<hal::timer::CopyableTimer0> {
    fn now_us(&self) -> u64 {
        self.get_counter().ticks()
    }
}
