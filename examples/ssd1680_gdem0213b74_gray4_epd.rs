//! # GDEM0213B74 4-Level Grayscale (Gray4) E-Paper Example (`epdsi`)
//!
//! Companion to `ssd1680_gdem0213b74_epd` (plain 1-bit monochrome). Same board, panel and
//! wiring, but drives the panel's **4-level grayscale** mode instead: White/Light/Dark/Black
//! instead of just White/Black.
//!
//! Two static screens:
//!
//! 1. **Screen 1**: title/subtitle banner over a 4-band swatch (Black/Dark/Light/White), each
//!    band carrying a single-letter label (`B`/`D`/`L`/`W`) sized to fit this panel's narrower
//!    122px width without overflowing into its neighbor.
//! 2. **Screen 2**: three concentric rounded rectangles (Light/Dark/White) with a centered
//!    "4-Lvl Gray" label in the innermost White ring.
//!
//! Like `ssd1680_gdey0266t90_gray4_epd`, there is no partial/differential refresh phase here:
//! Gray4 mode has exactly one trigger ([`Ssd168xRefreshMode::Gray4`]).
//!
//! ## Provenance: read before trusting this on hardware
//!
//! `GDEM0213B74::GRAY4` is **not** Adafruit's own product-page default mode for this breakout.
//! It is transcribed verbatim from Adafruit_EPD's `ti_213mfgn_gray4_init_code`/
//! `ti_213mfgn_gray4_lut_code`, and is byte-identical to the already-shipped `GDEY0266T90::GRAY4`
//! bundle (same 233-byte LUT, same VCOM/gate/source/LUT-end bytes): both come from the same
//! Adafruit reference family for the SSD1680 IC. See `epdsi`'s `GDEM0213B74` panel doc for the
//! full caveat, including a per-breakout-revision `_colstart` offset Adafruit's own driver
//! carries that `epdsi` has no equivalent of. This example is the hardware verification this
//! bundle has not had yet: run it, and see what actually lights up.
//!
//! ## Note on the 122 pixel panel width
//!
//! Unlike `GDEY0266T90` (152px, an exact multiple of 8 and of 4), this panel's 122px width
//! divides awkwardly: `WIDTH.div_ceil(8) = 16` bytes/row (the controller's own RAM alignment,
//! unchanged by Gray4), and `WIDTH / 4` for the Screen 1 swatch bands leaves a 2px remainder,
//! handled below by widening the last band rather than leaving a gap. More significantly, the
//! narrower width does not have room for the longer text or 4-ring layout
//! `ssd1680_gdey0266t90_gray4_epd` uses. See that example's own hardware-learned note on
//! `FONT_10X20` overflowing a padded ring on a wider panel. This example uses `FONT_6X10`
//! throughout (same choice `ssd1680_gdem0213b74_epd` already makes for its own footer labels),
//! single-letter swatch labels instead of words, and three rings instead of four, each sized
//! with its text-fit math spelled out in the comments below rather than copied from the wider
//! panel's layout and hoped to still fit.
//!
//! ## Hardware
//!
//! Same board, panel class, adapter and wiring as `ssd1680_gdem0213b74_epd`. See that example
//! for the full pin table. Repeated here for convenience:
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
//! cargo run --example ssd1680_gdem0213b74_gray4_epd
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle, RoundedRectangle};
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

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Row stride in bytes. 122 px rounds up to 16, same alignment the plain-mono example uses.
const STRIDE: usize = GDEM0213B74::WIDTH.div_ceil(8) as usize;

/// Full frame buffer size per plane: 16 x 250 = 4,000 bytes.
const FRAME_BYTES: usize = STRIDE * GDEM0213B74::HEIGHT as usize;

/// Adafruit's SSD1680 Gray4 convention: neither RAM plane inverted, a set bit is the code bit
/// directly. The only convention `epdsi` has evidence for so far.
const POLARITY: Gray4Polarity = Gray4Polarity::ADAFRUIT_SSD1680;

/// Writes both bit-planes for the full frame, resetting the RAM window and cursor first.
fn write_full_frame<BUS, C>(epd: &mut EpdDriver<BUS, C, GDEM0213B74>, page: &GrayBufferPair)
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
{
    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, page.plane_a().as_slice())
        .unwrap();

    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, page.plane_b().as_slice())
        .unwrap();
}

/// Screen 1: title/subtitle banner over a 4-band Black/Dark/Light/White swatch, each band
/// labeled with a single letter. All three text lines and all four band widths are sized to
/// this panel's 122px width explicitly (see the widths/positions computed below), not copied
/// from the wider `GDEY0266T90` layout.
fn draw_banner(page: &mut GrayBufferPair) {
    let dark_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Dark);
    let light_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Light);
    let black_small_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Black);
    let white_small_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::White);

    // "SSD1680 Gray4" is 13 chars at 6px = 78px, centered in the 122px width: x = (122-78)/2 = 22.
    Text::new("SSD1680 Gray4", Point::new(22, 10), black_small_style)
        .draw(page)
        .unwrap();

    // "GDEM0213B74 122x250" is 19 chars at 6px = 114px: x = (122-114)/2 = 4.
    Text::new("GDEM0213B74 122x250", Point::new(4, 24), dark_style)
        .draw(page)
        .unwrap();

    // "epdsi Gray4 demo" is 16 chars at 6px = 96px: x = (122-96)/2 = 13.
    Text::new("epdsi Gray4 demo", Point::new(13, 38), light_style)
        .draw(page)
        .unwrap();

    // Four bands spanning the full 122px width. 122/4 = 30 remainder 2, so the last band is
    // widened to 32px rather than leaving a 2px gap: 30+30+30+32 = 122.
    const BAR_Y: i32 = 55;
    const BAR_H: u32 = 40;
    const BAR_W: u32 = GDEM0213B74::WIDTH / 4;
    const LAST_BAR_W: u32 = GDEM0213B74::WIDTH - 3 * BAR_W;

    let bands = [
        (0u32, BAR_W, Gray4Color::Black, "B", white_small_style),
        (1u32, BAR_W, Gray4Color::Dark, "D", white_small_style),
        (2u32, BAR_W, Gray4Color::Light, "L", black_small_style),
        (3u32, LAST_BAR_W, Gray4Color::White, "W", black_small_style),
    ];
    let mut x = 0i32;
    for (_index, width, fill, label, label_style) in bands {
        Rectangle::new(Point::new(x, BAR_Y), Size::new(width, BAR_H))
            .into_styled(PrimitiveStyle::with_fill(fill))
            .draw(page)
            .unwrap();
        // Single 6px-wide character, centered in the band: x + (width-6)/2.
        Text::new(
            label,
            Point::new(x + (width as i32 - 6) / 2, BAR_Y + BAR_H as i32 / 2 + 3),
            label_style,
        )
        .draw(page)
        .unwrap();
        x += width as i32;
    }
    // Outline around the White band so its edge is visible against the page background.
    Rectangle::new(
        Point::new(3 * BAR_W as i32, BAR_Y),
        Size::new(LAST_BAR_W, BAR_H),
    )
    .into_styled(PrimitiveStyle::with_stroke(Gray4Color::Black, 1))
    .draw(page)
    .unwrap();
}

/// Screen 2: three concentric rounded rectangles (Light/Dark/White, outer to inner) with a
/// centered "4-Lvl Gray" label in the innermost White ring. One ring fewer than
/// `ssd1680_gdey0266t90_gray4_epd`'s four: that version's 76px-wide innermost ring has room for
/// a 12-character label; this panel's narrower width does not reach that at the same padding
/// (see the inner-ring math below), so the ring count and label text are sized down instead of
/// risking the overflow that example's own module doc warns about.
fn draw_geometric(page: &mut GrayBufferPair) {
    let stroke = PrimitiveStyle::with_stroke(Gray4Color::Black, 1);
    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEM0213B74::WIDTH, GDEM0213B74::HEIGHT),
    )
    .into_styled(stroke)
    .draw(page)
    .unwrap();

    // (padding, corner radius, fill) for each nested ring, outer to inner. Innermost pad=22
    // leaves an inner rectangle 122 - 2*22 = 78px wide: enough for the 60px label below plus a
    // margin, which pad=30 or higher (as used on the wider panel) would not be.
    let rings: [(u32, u32, Gray4Color); 3] = [
        (6, 6, Gray4Color::Light),
        (14, 5, Gray4Color::Dark),
        (22, 4, Gray4Color::White),
    ];
    for (pad, radius, fill) in rings {
        let rect = Rectangle::new(
            Point::new(pad as i32, pad as i32),
            Size::new(GDEM0213B74::WIDTH - 2 * pad, GDEM0213B74::HEIGHT - 2 * pad),
        );
        RoundedRectangle::with_equal_corners(rect, Size::new(radius, radius))
            .into_styled(PrimitiveStyle::with_fill(fill))
            .draw(page)
            .unwrap();
    }

    // Innermost ring (pad=22) is 78px wide. "4-Lvl Gray" at FONT_6X10 is 10 chars * 6px = 60px,
    // centered: x = 22 + (78-60)/2 = 31.
    let inner_pad = 22i32;
    let inner_width = GDEM0213B74::WIDTH as i32 - 2 * inner_pad;
    let label_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Black);
    let label = "4-Lvl Gray";
    let label_width = label.len() as i32 * 6;
    Text::new(
        label,
        Point::new(
            inner_pad + (inner_width - label_width) / 2,
            GDEM0213B74::HEIGHT as i32 / 2,
        ),
        label_style,
    )
    .draw(page)
    .unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEM0213B74 4-Level Grayscale (Gray4) EPD example (epdsi SSD1680)");
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

    // `for_panel` picks up `GDEM0213B74`'s dimensions; `.with_gray4` layers on the Adafruit_EPD-
    // sourced register bundle, and `Ssd168xRefreshMode::Gray4` selects its one refresh trigger.
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1680Controller::for_panel::<GDEM0213B74>()
        .with_gray4(GDEM0213B74::GRAY4)
        .with_refresh_mode(Ssd168xRefreshMode::Gray4);
    let mut epd = EpdBuilder::<_, GDEM0213B74>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1680 epdsi EPD driver (Gray4)...");
    epd.init(&mut timer).unwrap();

    let mut plane_a = [0u8; FRAME_BYTES];
    let mut plane_b = [0u8; FRAME_BYTES];

    defmt::info!("--- Screen 1: Banner & 4-Level Swatch ---");
    {
        let mut page = GrayBufferPair::new(
            &mut plane_a,
            &mut plane_b,
            GDEM0213B74::WIDTH,
            GDEM0213B74::HEIGHT,
            0,
            POLARITY,
        );
        page.clear();
        draw_banner(&mut page);
        write_full_frame(&mut epd, &page);
    }
    epd.refresh(&mut timer).unwrap();

    timer.delay_ms(8000);

    defmt::info!("--- Screen 2: Concentric Geometric Grayscale Test Pattern ---");
    {
        let mut page = GrayBufferPair::new(
            &mut plane_a,
            &mut plane_b,
            GDEM0213B74::WIDTH,
            GDEM0213B74::HEIGHT,
            0,
            POLARITY,
        );
        page.clear();
        draw_geometric(&mut page);
        write_full_frame(&mut epd, &page);
    }
    epd.refresh(&mut timer).unwrap();

    // Deep sleep. init() must be called again before any further frame. Panel is left on the
    // geometric test pattern.
    epd.sleep(&mut timer).unwrap();
    defmt::info!("Display complete, controller asleep.");

    // Blink status LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
