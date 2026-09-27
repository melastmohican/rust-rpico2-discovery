//! # Good Display GDEQ0426T82 4.26" 4-Level Grayscale (Gray4) E-Paper Example (`epdsi`)
//!
//! Companion to `ssd1677_gdeq0426t82_epd` (plain 1-bit monochrome) — same board, panel and
//! wiring, but drives the panel's **4-level grayscale** mode instead: White/Light/Dark/Black
//! instead of just White/Black.
//!
//! ## Provenance — read before trusting this on hardware
//!
//! `GDEQ0426T82::GRAY4` is **not** Good Display/Seeed material — Good Display's own spec lists this
//! as a 2-level (monochrome) panel. It is transcribed verbatim from Adafruit_EPD's
//! `ThinkInk_426_Grayscale4_GDEQ` reference driver (`ti_426_gray4_init_code` /
//! `ti_426_gray4_lut_code`), the only Gray4 reference for this controller/panel pairing. **Not yet
//! confirmed on hardware** — this is a from-scratch, from-source port, not a copy of a working
//! `epdsi` example. This example is exactly that verification: run it, and see what actually
//! lights up.
//!
//! Unlike single-pass SSD1680 Gray4 panels (see this repo's other Gray4 examples),
//! `Adafruit_SSD1677::update()`'s grayscale branch is **two-pass**: a full refresh with the OTP LUT
//! (Red/Yellow plane bypassed) sets a known monochrome baseline, then the custom LUT and voltage
//! registers are reloaded, then a second refresh with the real Black/White (LSB) and Red/Yellow
//! (MSB) planes. See `epdsi`'s `Ssd1677RefreshMode::Gray4Preclear`/`Gray4` docs for the full
//! sequence — this example drives it directly with `EpdDriver` primitives (`write_frame`/`refresh`/
//! `reload_gray4_lut`) rather than the paged `render_paged_gray4_preclear` helper, since the whole
//! 800x480 frame fits comfortably in two 48,000-byte buffers without paging.
//!
//! ## Note on orientation
//!
//! Same portrait convention as `ssd1677_gdeq0426t82_epd`: [`DisplayRotation::Rotate270`] turns the
//! panel's native 800x480 landscape RAM into a 480x800 portrait drawing surface with the FPC
//! ribbon at the bottom. Drawing coordinates below are in that 480-wide by 800-tall space.
//!
//! ## Hardware
//!
//! Same board, panel, adapter and wiring as `ssd1677_gdeq0426t82_epd` — see that example for the
//! full pin table. Repeated here for convenience:
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Dalian Good Display GDEQ0426T82 4.26" (800x480), driven in Gray4 mode
//! - **Adapter Board:** [Good Display DESPI-C02](https://www.good-display.com/product/516.html)
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
//! cargo run --example ssd1677_gdeq0426t82_gray4_epd
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_10X20};
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

/// Frame buffer size per bit-plane: 100 bytes per RAM row x 480 rows = 48,000 bytes.
const FRAME_BYTES: usize = GDEQ0426T82::WIDTH.div_ceil(8) as usize * GDEQ0426T82::HEIGHT as usize;

/// Visible width in the rotated portrait frame (the panel's 480 px axis).
const VIEW_W: u32 = GDEQ0426T82::HEIGHT;

/// Visible height in the rotated portrait frame (the panel's 800 px axis).
const VIEW_H: u32 = GDEQ0426T82::WIDTH;

/// This panel's two RAM planes are both inverted — see `Gray4Polarity::ADAFRUIT_SSD1677`'s doc
/// for the derivation from `ThinkInk_426_Grayscale4_GDEQ.h`.
const POLARITY: Gray4Polarity = Gray4Polarity::ADAFRUIT_SSD1677;

/// Screen 1: title/subtitle banner over a 4-band Black/Dark/Light/White swatch.
fn draw_banner(page: &mut GrayBufferPair) {
    let title_style = MonoTextStyle::new(&FONT_10X20, Gray4Color::Black);
    let dark_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Dark);
    let light_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Light);
    let black_small_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Black);
    let white_small_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::White);

    // "SSD1677 Gray4" is 13 chars at 10px = 130px, centered in the 480px width.
    Text::new("SSD1677 Gray4", Point::new(175, 40), title_style)
        .draw(page)
        .unwrap();
    // "GDEQ0426T82 800x480" is 20 chars at 6px = 120px.
    Text::new("GDEQ0426T82 800x480", Point::new(180, 62), dark_style)
        .draw(page)
        .unwrap();
    // "epdsi Gray4 demo" is 16 chars at 6px = 96px.
    Text::new("epdsi Gray4 demo", Point::new(192, 84), light_style)
        .draw(page)
        .unwrap();

    // Four equal bands spanning the full 480px width, one per gray level.
    const BAR_Y: i32 = 140;
    const BAR_H: u32 = 100;
    const BAR_W: u32 = VIEW_W / 4;

    let bands = [
        (0u32, Gray4Color::Black, "Black", white_small_style),
        (1u32, Gray4Color::Dark, "Dark", white_small_style),
        (2u32, Gray4Color::Light, "Light", black_small_style),
        (3u32, Gray4Color::White, "White", black_small_style),
    ];
    for (index, fill, label, label_style) in bands {
        let x = (index * BAR_W) as i32;
        Rectangle::new(Point::new(x, BAR_Y), Size::new(BAR_W, BAR_H))
            .into_styled(PrimitiveStyle::with_fill(fill))
            .draw(page)
            .unwrap();
        Text::new(label, Point::new(x + 8, BAR_Y + 56), label_style)
            .draw(page)
            .unwrap();
    }
    // Outline around the White band so its edge is visible against the page background.
    Rectangle::new(Point::new(3 * BAR_W as i32, BAR_Y), Size::new(BAR_W, BAR_H))
        .into_styled(PrimitiveStyle::with_stroke(Gray4Color::Black, 1))
        .draw(page)
        .unwrap();
}

/// Screen 2: four concentric rounded rectangles alternating gray levels, with a centered label.
fn draw_geometric(page: &mut GrayBufferPair) {
    let stroke = PrimitiveStyle::with_stroke(Gray4Color::Black, 1);
    Rectangle::new(Point::new(0, 0), Size::new(VIEW_W, VIEW_H))
        .into_styled(stroke)
        .draw(page)
        .unwrap();

    // (padding, corner radius, fill) for each nested ring, outer to inner.
    let rings: [(u32, u32, Gray4Color); 4] = [
        (20, 24, Gray4Color::Light),
        (60, 18, Gray4Color::Dark),
        (100, 12, Gray4Color::Black),
        (140, 12, Gray4Color::White),
    ];
    for (pad, radius, fill) in rings {
        let rect = Rectangle::new(
            Point::new(pad as i32, pad as i32),
            Size::new(VIEW_W - 2 * pad, VIEW_H - 2 * pad),
        );
        RoundedRectangle::with_equal_corners(rect, Size::new(radius, radius))
            .into_styled(PrimitiveStyle::with_fill(fill))
            .draw(page)
            .unwrap();
    }

    // Centered "4-Level Gray" label inside the innermost White ring.
    let inner_pad = 140i32;
    let inner_width = VIEW_W as i32 - 2 * inner_pad;
    let label_style = MonoTextStyle::new(&FONT_6X10, Gray4Color::Black);
    let label = "4-Level Gray";
    let label_width = label.len() as i32 * 6;
    Text::new(
        label,
        Point::new(
            inner_pad + (inner_width - label_width) / 2,
            VIEW_H as i32 / 2,
        ),
        label_style,
    )
    .draw(page)
    .unwrap();
}

/// Preclear pass: writes `data` to *both* the Black/White and Red/Yellow channels, so the
/// baseline OTP-LUT refresh (`Ssd1677RefreshMode::Gray4Preclear`) matches the final image's
/// black/white split.
fn write_preclear<BUS, C, P>(epd: &mut EpdDriver<BUS, C, P>, data: &[u8])
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
{
    epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, data).unwrap();

    epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, data).unwrap();
}

/// Final pass: writes the real Black/White (LSB) and Red/Yellow (MSB) planes.
fn write_real_planes<BUS, C, P>(epd: &mut EpdDriver<BUS, C, P>, plane_a: &[u8], plane_b: &[u8])
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
{
    epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, plane_a).unwrap();

    epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, plane_b).unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!(
        "Starting GDEQ0426T82 4.26\" 4-Level Grayscale (Gray4) EPD example (epdsi SSD1677)"
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
    // SSD1677 BUSY is active-HIGH
    let busy = pins.gpio13.into_pull_down_input();

    // Create SPI driver instance (16 MHz clock)
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Instantiate epdsi SPI bus wrapper and Gray4-configured SSD1677 controller. `for_panel`
    // picks up GDEQ0426T82's dimensions; `.with_gray4` layers on the Adafruit_EPD-sourced register
    // bundle. Start on `Gray4Preclear` — the first pass of every screen below.
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1677Controller::for_panel::<GDEQ0426T82>()
        .with_gray4(GDEQ0426T82::GRAY4)
        .with_refresh_mode(Ssd1677RefreshMode::Gray4Preclear);

    // Build EPD Driver using epdsi with GDEQ0426T82 panel specification (800x480)
    let mut epd = EpdBuilder::<_, GDEQ0426T82>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1677 epdsi EPD driver (Gray4)...");
    epd.init(&mut timer).unwrap();

    // Two 48,000-byte bit-plane buffers, held as plain stack arrays — the same choice
    // `ssd1677_gdeq0426t82_epd` makes for its one buffer, and the RP2350's 512 KB of SRAM
    // absorbs both comfortably.
    let mut plane_a_buf = [0u8; FRAME_BYTES];
    let mut plane_b_buf = [0u8; FRAME_BYTES];

    defmt::info!("--- Screen 1: Banner & 4-Level Swatch ---");
    {
        let mut page = GrayBufferPair::new(
            &mut plane_a_buf,
            &mut plane_b_buf,
            GDEQ0426T82::WIDTH,
            GDEQ0426T82::HEIGHT,
            0,
            POLARITY,
        );
        page.set_rotation(DisplayRotation::Rotate270);
        page.clear();
        draw_banner(&mut page);

        defmt::info!("Preclear pass (mono baseline, expect several seconds)...");
        write_preclear(&mut epd, page.plane_a().as_slice());
        epd.refresh(&mut timer).unwrap();

        // The preclear refresh's OTP LUT load overwrote the custom LUT/voltage registers
        // uploaded during init — reload them before the real Gray4 refresh.
        {
            let (bus, controller) = epd.split_mut();
            controller.reload_gray4_lut(bus).unwrap();
        }
        epd.controller_mut()
            .set_refresh_mode(Ssd1677RefreshMode::Gray4);

        defmt::info!("Gray4 pass (real image, expect several seconds)...");
        write_real_planes(
            &mut epd,
            page.plane_a().as_slice(),
            page.plane_b().as_slice(),
        );
        epd.refresh(&mut timer).unwrap();
    }

    timer.delay_ms(8000);

    defmt::info!("--- Screen 2: Concentric Geometric Grayscale Test Pattern ---");
    epd.controller_mut()
        .set_refresh_mode(Ssd1677RefreshMode::Gray4Preclear);
    {
        let mut page = GrayBufferPair::new(
            &mut plane_a_buf,
            &mut plane_b_buf,
            GDEQ0426T82::WIDTH,
            GDEQ0426T82::HEIGHT,
            0,
            POLARITY,
        );
        page.set_rotation(DisplayRotation::Rotate270);
        page.clear();
        draw_geometric(&mut page);

        defmt::info!("Preclear pass (mono baseline, expect several seconds)...");
        write_preclear(&mut epd, page.plane_a().as_slice());
        epd.refresh(&mut timer).unwrap();

        {
            let (bus, controller) = epd.split_mut();
            controller.reload_gray4_lut(bus).unwrap();
        }
        epd.controller_mut()
            .set_refresh_mode(Ssd1677RefreshMode::Gray4);

        defmt::info!("Gray4 pass (real image, expect several seconds)...");
        write_real_planes(
            &mut epd,
            page.plane_a().as_slice(),
            page.plane_b().as_slice(),
        );
        epd.refresh(&mut timer).unwrap();
    }

    defmt::info!("Display complete!");

    // Blink status LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
