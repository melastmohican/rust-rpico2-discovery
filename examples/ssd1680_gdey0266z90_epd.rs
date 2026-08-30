//! # GDEY0266Z90 2.66" Tri-Color E-Paper Example (`epdsi`)
//!
//! Example for the Raspberry Pi Pico 2 microcontroller board driving the **GDEY0266Z90**
//! 2.66" Tri-Color (Black/White/Red, 152x296) E-Paper Display using the `Ssd1680Controller`
//! from the `epdsi` library.
//!
//! Demonstrates every refresh mode the SSD1680 exposes for this panel:
//!
//! 1. **Phase 1**: Full tri-color refresh ([`Ssd168xRefreshMode::Full`]) — header, Black and Red
//!    swatches, Ferris logo (Red), Rust logo (Black), and text labels.
//! 2. **Phase 2**: Partial *window* refresh loop on the full waveform, repainting only the bottom
//!    status band with a Black counter and a Red progress bar, leaving the logos untouched.
//! 3. **Phase 3**: [`Ssd168xRefreshMode::FastFull`] full-screen refresh, timed against Phase 1 to
//!    show what the temperature-override waveform actually buys on a colour panel.
//! 4. **Phase 4**: [`Ssd168xRefreshMode::BaseMap`] and [`Ssd168xRefreshMode::Partial`], the two
//!    modes ported from Good Display's reference driver, shown at their real cost.
//!
//! ## Note on refresh speed
//!
//! Tri-color (BWR) panels have **no fast/differential waveform**: the red pigment needs the long
//! OTP waveform, so every full update takes ~18-20 s — slower than any panel in this repo's
//! reference timings table. A run that looks hung almost certainly is not; the whole example takes
//! roughly two minutes.
//!
//! Measured on this hardware (RP2350 + DESPI-C02, DKE glass), and logged by the example itself:
//!
//! | Mode | Measured | Notes |
//! |---|---:|---|
//! | `Full` | 20.0 s | Phase 1, whole screen |
//! | `Full`, windowed | 20.0 s | Phase 2 — a narrower window costs exactly the same |
//! | `FastFull` | **16.2 s** | Phase 3, ~19 % faster |
//! | `BaseMap` | 19.9 s | Phase 4 |
//! | `Partial` | 19.9 s | Phase 4 — not differential, not faster |
//!
//! Phase 3 is the one worth reading. [`Ssd168xRefreshMode::FastFull`] drives
//! `UPDATE_DISPLAY_CTRL2 = 0xC7` after overriding the temperature register to 90 °C
//! (`0x22 0xB1` → `0x1A 0x5A 0x00` → `0x22 0x91`), reloading the OTP LUT at a temperature where
//! the waveform is shorter. Good Display quote ~19 s against ~20 s on their own glass, i.e. barely
//! a saving; on the DKE glass here it is 16.2 s against 20.0 s, a real one. That difference is the
//! OTP waveform, which varies by glass supplier even at identical controller and resolution — so
//! measure it on the panel in front of you rather than trusting either number, which is why this
//! phase logs it.
//!
//! ## Note on duty cycle
//!
//! Waveshare recommend **at least 180 s between refreshes** on this panel, and at least one update
//! every 24 h to avoid burn-in. This example deliberately ignores the first of those: it runs
//! seven refreshes a couple of seconds apart, because demonstrating the modes back to back is the
//! whole point. That is fine as a one-off bring-up run — **do not loop it, and do not model
//! production firmware on its pacing.** Anything long-running should leave the panel in
//! `sleep()` between updates and refresh no faster than the vendor interval.
//!
//! ## Note on Phase 4: there is no differential update on this panel
//!
//! [`Ssd168xRefreshMode::Partial`] selects the controller's built-in fast LUT, which exists only
//! for monochrome panels. On colour glass it is neither fast nor differential: measured here at
//! 19.9 s, indistinguishable from `Full`. Phase 4 shows the two modes doing what they really do,
//! rather than pretending to a fast path this panel does not have.
//!
//! **Do not port the monochrome differential idiom to this panel.** On the mono `GDEM0213B74`,
//! `0x26` is a previous-frame buffer sharing the Black/White polarity, so
//! `ssd1680_gdem0213b74_epd` seeds it with the displayed image and diffs against it. Here `0x26`
//! is *always* the Red plane. Writing a mostly-`0xFF` Black/White image into it sets nearly every
//! bit, and a set bit is red — the band renders solid red with the text knocked out of it. An
//! earlier revision of this example did exactly that; the give-away in the log was a "differential"
//! update taking 19.9 s, which is a full colour waveform, not a diff.
//!
//! Every pass in Phase 4 therefore writes **both** planes, exactly like Phases 1-3. Region updates
//! do still work — only the window is redrawn, and the Ferris logo above the band survives
//! untouched because e-paper is bistable — they just cost a full refresh.
//!
//! ## Note on ink polarity
//!
//! The two RAM planes disagree, which is the thing most likely to be wrong when porting this to
//! another panel. `0xFF` is white in the Black/White plane (`0x24`), but the Red plane (`0x26`) is
//! **inverted**: `0x00` is no red and a *set* bit is red. So `red_buf` starts at `0x00` and red
//! content is drawn as [`BinaryColor::Off`], which is what sets a bit in [`PageBuffer`] — the same
//! idiom as the `ssd1681_gdem0154z90_epd` example. Failure signatures, if you are bringing up a
//! fresh panel:
//!
//! | What you see | What it means |
//! |---|---|
//! | Panel comes up all red on the first clear | Red plane fill inverted — `0x26` wants `0x00`, not `0xFF` |
//! | Panel comes up all black on the first clear | Black/White plane fill inverted — `0x24` wants `0xFF` |
//! | Red artwork appears as its photographic negative | Red drawn as `On` instead of `Off` |
//! | Black and red content swapped | The two `ColorChannel`s are routed to the wrong commands |
//! | Correct image, upside down | Orientation only — apply `DisplayRotation::Rotate180` |
//! | Correct geometry, weak red or heavy ghosting | OTP waveform, not a register fault. See the note on glass provenance below |
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Good Display GDEY0266Z90 / Waveshare 2.66inch e-Paper Module (B), 152x296 BWR
//!   ([product page](https://www.good-display.com/product/430.html))
//! - **Adapter Board:** [Good Display DESPI-C02](https://www.good-display.com/product/516.html),
//!   matching the other `epdsi` examples in this repo. Any 24-pin FPC breakout exposing the same
//!   signals works; only the pin mapping below matters.
//!
//! The glass this was brought up on is stamped `DEPG0266RWS800F34HP` with the ribbon marked
//! `FPC-7510 Rev. C`, which makes it **DKE** glass rather than Good Display's — normal for this
//! module, which Waveshare source from more than one supplier. `S800` in that part number is the
//! SSD1680 itself; the same 2.66" glass also ships with a JD79651B (`F51B`) or UC8251d (`U25D`),
//! and neither works with this driver. Check the label before assuming a fault.
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
//! Power-cycle the board first, then run once and let it finish — an interrupted run can leave
//! the controller latched busy, and the *next* run then looks broken for reasons that are not in
//! the code.
//!
//! ```bash
//! cargo run --example ssd1680_gdey0266z90_epd
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
/// painted in Phase 1 and never touched again, so the red logo stays put.
const BAND_Y: u32 = 220;

/// Height of the status band in pixels (y = 220..295).
const BAND_H: u32 = 76;

/// Status band buffer size: 19 x 76 = 1,444 bytes.
const BAND_BYTES: usize = STRIDE * BAND_H as usize;

/// Empty fill for the Red plane over the status band, clearing the red the Phase 2 progress bar
/// left there so the base-map pass lands on white. Lives in flash rather than on the stack.
///
/// Note the value: this is `0x00`, **not** the `0xFF` that means white in the Black/White plane.
/// The Red plane is inverted — a set bit is red — so `0xFF` here would paint the whole band solid
/// red. The monochrome `ssd1680_gdem0213b74_epd` example uses `0xFF` for its equivalent buffer,
/// because on that panel `0x26` is a previous-frame buffer sharing the Black/White polarity rather
/// than a colour plane. Do not copy that constant across.
static NO_RED_BAND: [u8; BAND_BYTES] = [0x00u8; BAND_BYTES];

/// Refreshes the panel, reporting how long it took and returning the elapsed milliseconds.
///
/// A full refresh on this panel is ~18-20 s; anything close to zero means the refresh was not
/// actually driven, which is a failure that otherwise looks like success in the log.
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
/// `bw` receives Black content in the ordinary `BinaryColor::On` convention; `red` receives Red
/// content as `BinaryColor::Off`, which sets a bit in the `0x00`-based Red plane.
fn draw_static_content(
    bw: &mut PageBuffer,
    red: &mut PageBuffer,
    ferris_bmp: &Bmp<BinaryColor>,
    rust_bmp: &Bmp<BinaryColor>,
    mode_label: &str,
) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Outer border (Black), so a shifted or wrapped raster is obvious.
    Rectangle::new(
        Point::new(0, 0),
        Size::new(GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT),
    )
    .into_styled(stroke)
    .draw(bw)
    .unwrap();

    // Header (Black). 11 chars at 10 px each fits the 152 px width.
    Text::new("GDEY0266Z90", Point::new(8, 22), text_style)
        .draw(bw)
        .unwrap();

    // Subtitle: "Tri-Color " in Black, "BWR" in Red.
    Text::new("Tri-Color ", Point::new(8, 40), small_text_style)
        .draw(bw)
        .unwrap();
    Text::new(
        "BWR",
        Point::new(68, 40),
        MonoTextStyle::new(&FONT_6X10, BinaryColor::Off),
    )
    .draw(red)
    .unwrap();

    Line::new(Point::new(8, 48), Point::new(143, 48))
        .into_styled(stroke)
        .draw(bw)
        .unwrap();

    // Colour swatches: Black left, Red right, inside a shared outline.
    Rectangle::new(Point::new(8, 56), Size::new(136, 18))
        .into_styled(stroke)
        .draw(bw)
        .unwrap();
    Rectangle::new(Point::new(10, 58), Size::new(64, 14))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(bw)
        .unwrap();
    Rectangle::new(Point::new(78, 58), Size::new(64, 14))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(red)
        .unwrap();

    // Ferris (64x42) in Red and Rust (64x64) in Black, side by side — 128 px of artwork fits the
    // 152 px width, unlike the 122 px monochrome panel where they have to be stacked.
    let ferris_pos = Point::new(10, 92);
    for pixel in ferris_bmp.pixels() {
        if pixel.1 == BinaryColor::Off {
            Pixel(pixel.0 + ferris_pos, BinaryColor::Off)
                .draw(red)
                .unwrap();
        }
    }

    let rust_pos = Point::new(78, 82);
    for pixel in rust_bmp.pixels() {
        if pixel.1 == BinaryColor::On {
            Pixel(pixel.0 + rust_pos, BinaryColor::On).draw(bw).unwrap();
        }
    }

    // Labels (Black).
    Text::new("RP2350 Pico 2", Point::new(8, 170), small_text_style)
        .draw(bw)
        .unwrap();
    Text::new("epdsi SSD1680", Point::new(8, 184), small_text_style)
        .draw(bw)
        .unwrap();
    Text::new(mode_label, Point::new(8, 198), small_text_style)
        .draw(bw)
        .unwrap();

    // Separator above the status band that Phases 2 and 4 repaint.
    Line::new(Point::new(8, 210), Point::new(143, 210))
        .into_styled(stroke)
        .draw(bw)
        .unwrap();
}

/// Writes both colour planes for the full frame, resetting the RAM window and cursor first.
///
/// Each RAM write restarts from the window origin, so the window and cursor have to be re-armed
/// before every plane rather than once per frame.
fn write_full_frame<BUS, C, P>(epd: &mut EpdDriver<BUS, C, P>, bw: &[u8], red: &[u8])
where
    C: EpdController<BUS>,
    C::Error: core::fmt::Debug,
    P: EpdPanel,
{
    epd.set_window(0, 0, GDEY0266Z90::WIDTH - 1, GDEY0266Z90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, bw).unwrap();

    epd.set_window(0, 0, GDEY0266Z90::WIDTH - 1, GDEY0266Z90::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, red).unwrap();
}

/// Draws the Black/White half of the status band: label, counter and progress bar outline.
///
/// The bar *fill* is left to the caller, because which plane it belongs in differs by phase.
fn draw_band(band: &mut PageBuffer, count: u32, label: &str) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let small_text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::new(label, Point::new(8, BAND_Y as i32 + 14), small_text_style)
        .draw(band)
        .unwrap();

    let mut count_buf = [0u8; 32];
    let count_str = format_no_std::show(&mut count_buf, format_args!("Update #{}", count)).unwrap();
    Text::new(
        count_str,
        Point::new(8, BAND_Y as i32 + 28),
        small_text_style,
    )
    .draw(band)
    .unwrap();

    // Progress bar outline always lands on the Black/White plane.
    Rectangle::new(Point::new(8, BAND_Y as i32 + 38), Size::new(136, 16))
        .into_styled(stroke)
        .draw(band)
        .unwrap();
}

/// Draws the progress bar fill for `count` into `plane`.
///
/// `color` carries the plane's convention: `BinaryColor::Off` sets a bit, which is red in the
/// `0x00`-based Red plane; `BinaryColor::On` clears one, which is black in the `0xFF`-based
/// Black/White plane. Passing the wrong one for the plane yields an invisible bar, or a bar-shaped
/// hole in a solid field.
fn draw_band_bar(plane: &mut PageBuffer, count: u32, color: BinaryColor) {
    Rectangle::new(
        Point::new(10, BAND_Y as i32 + 40),
        Size::new(count * 33, 12),
    )
    .into_styled(PrimitiveStyle::with_fill(color))
    .draw(plane)
    .unwrap();
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting GDEY0266Z90 2.66\" Tri-Color EPD example (epdsi SSD1680)");
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

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1680 controller. This panel needs no
    // variant selection: it shares the default SSD1680 register profile with the GDEM0213B74.
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1680Controller::new(GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT)
        .with_refresh_mode(Ssd168xRefreshMode::Full);

    // Build EPD Driver using epdsi with GDEY0266Z90 panel specification (152x296)
    let mut epd = EpdBuilder::<_, GDEY0266Z90>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1680 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // The asymmetric pair: 0xFF is white in the Black/White plane, but the Red plane is inverted,
    // so 0x00 is *no* red.
    epd.clear_frame(ColorChannel::BlackWhite, 0xFF).unwrap();
    epd.clear_frame(ColorChannel::RedYellow, 0x00).unwrap();

    // Frame buffers: 5,624 bytes each. Note the different clear values, per the polarity note.
    let mut bw_buf = [0xFFu8; FRAME_BYTES];
    let mut red_buf = [0x00u8; FRAME_BYTES];

    // Load BMP images
    let ferris_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("ferrisbw.bmp")).unwrap();
    let rust_bmp: Bmp<BinaryColor> = Bmp::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    defmt::info!("--- Phase 1: Full Tri-Color Refresh ---");

    // Scoped so the full-frame borrows end before Phase 2 re-borrows them as band buffers.
    let full_ms = {
        let mut bw = PageBuffer::new(&mut bw_buf, GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT, 0);
        let mut red = PageBuffer::new(&mut red_buf, GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT, 0);

        draw_static_content(&mut bw, &mut red, &ferris_bmp, &rust_bmp, "mode: Full");

        defmt::info!("Sending both planes ({} bytes each)...", FRAME_BYTES);
        write_full_frame(&mut epd, bw.as_slice(), red.as_slice());

        timed_refresh(&mut epd, &mut timer, "Phase 1 (Full)")
    };

    timer.delay_ms(2000);

    defmt::info!("--- Phase 2: Windowed Refresh on the Full Waveform ---");

    // Colour panels have no differential waveform, so a region update is not a speed-up — it is a
    // full refresh over a smaller area. Both planes must be written for the window.
    for count in 1..=2u32 {
        {
            let mut band = PageBuffer::new(
                &mut bw_buf[..BAND_BYTES],
                GDEY0266Z90::WIDTH,
                BAND_H,
                BAND_Y,
            );
            band.clear_byte(0xFF);
            let mut band_red = PageBuffer::new(
                &mut red_buf[..BAND_BYTES],
                GDEY0266Z90::WIDTH,
                BAND_H,
                BAND_Y,
            );
            band_red.clear_byte(0x00);

            draw_band(&mut band, count, "Full window");
            draw_band_bar(&mut band_red, count, BinaryColor::Off);
        }

        epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, &bw_buf[..BAND_BYTES])
            .unwrap();
        epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::RedYellow, &red_buf[..BAND_BYTES])
            .unwrap();

        defmt::info!("Refreshing band y={}..{}...", BAND_Y, BAND_Y + BAND_H - 1);
        timed_refresh(&mut epd, &mut timer, "Phase 2 (windowed Full)");
        timer.delay_ms(1000);
    }

    defmt::info!("--- Phase 3: FastFull Full-Screen Refresh ---");

    // Re-render the Phase 1 content, this time on the temperature-override waveform, and compare.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::FastFull);

    let fast_ms = {
        let mut bw = PageBuffer::new(&mut bw_buf, GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT, 0);
        bw.clear_byte(0xFF);
        let mut red = PageBuffer::new(&mut red_buf, GDEY0266Z90::WIDTH, GDEY0266Z90::HEIGHT, 0);
        red.clear_byte(0x00);

        draw_static_content(&mut bw, &mut red, &ferris_bmp, &rust_bmp, "mode: FastFull");

        write_full_frame(&mut epd, bw.as_slice(), red.as_slice());

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
    // previous-frame seeding: on a Tri-Color panel 0x26 is *always* the Red plane, so writing a
    // Black/White image into it — the idiom the monochrome ssd1680_gdem0213b74_epd example uses —
    // paints the band solid red instead of priming a differential buffer. Measured on hardware,
    // both modes below take ~19.9 s, the same as Full.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::BaseMap);

    {
        let mut band = PageBuffer::new(
            &mut bw_buf[..BAND_BYTES],
            GDEY0266Z90::WIDTH,
            BAND_H,
            BAND_Y,
        );
        band.clear_byte(0xFF);
        let mut band_red = PageBuffer::new(
            &mut red_buf[..BAND_BYTES],
            GDEY0266Z90::WIDTH,
            BAND_H,
            BAND_Y,
        );
        band_red.clear_byte(0x00);

        draw_band(&mut band, 0, "BaseMap");
        draw_band_bar(&mut band_red, 0, BinaryColor::Off);
    }

    epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &bw_buf[..BAND_BYTES])
        .unwrap();
    epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &NO_RED_BAND)
        .unwrap();

    timed_refresh(&mut epd, &mut timer, "Phase 4 (BaseMap)");

    timer.delay_ms(1000);

    // Partial selects the controller's built-in fast LUT (0x22 = 0xFC). That LUT exists only for
    // monochrome panels, so here it is neither fast nor differential — measured at 19.9 s, the
    // same as Full. It is included to show what the mode actually does on colour glass, and it
    // still needs both planes written or the red content in the window is dropped.
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Partial);

    for count in 1..=2u32 {
        {
            let mut band = PageBuffer::new(
                &mut bw_buf[..BAND_BYTES],
                GDEY0266Z90::WIDTH,
                BAND_H,
                BAND_Y,
            );
            band.clear_byte(0xFF);
            let mut band_red = PageBuffer::new(
                &mut red_buf[..BAND_BYTES],
                GDEY0266Z90::WIDTH,
                BAND_H,
                BAND_Y,
            );
            band_red.clear_byte(0x00);

            draw_band(&mut band, count, "Partial mode");
            draw_band_bar(&mut band_red, count, BinaryColor::Off);
        }

        epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::BlackWhite, &bw_buf[..BAND_BYTES])
            .unwrap();
        epd.set_window(0, BAND_Y, GDEY0266Z90::WIDTH - 1, BAND_Y + BAND_H - 1)
            .unwrap();
        epd.set_cursor(0, BAND_Y).unwrap();
        epd.write_frame(ColorChannel::RedYellow, &red_buf[..BAND_BYTES])
            .unwrap();

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
