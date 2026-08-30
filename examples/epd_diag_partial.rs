//! # SSD1680 partial-refresh bisect (RP2350)
//!
//! Port of `epd_diag_partial` from
//! [`xiao-esp32c3-blinky`](https://github.com/melastmohican/xiao-esp32c3-blinky), so the same
//! three measurements can be taken on a second host and compared. Everything from `STRIDE` down
//! to the end of `stripes` is unchanged; only `main` differs.
//!
//! This exists because the C3 produced numbers that disagree with that repo's own reference
//! timings in *both* directions — A about 2x slow, B and C several times too fast — on a setup
//! independently known to have marginal USB power delivery. Running the identical sequence on the
//! RP2350 separates "the C3 setup degrades the waveform" from "these are simply what `epdsi`
//! costs, and the reference timings need revisiting".
//!
//! - **A**: full frame, `Full` mode — baseline.
//! - **B**: full frame, `Partial` mode — is the fast LUT the problem?
//! - **C**: band y=50..249, `Partial` mode — is the *windowed* write the problem?
//!
//! Each test draws a distinct pattern, so the panel says as much as the log:
//! A = horizontal stripes, B = inverted stripes, C = stripes only below y=50.
//! **Watch the panel.** A stage that completes quickly without visibly changing the image has not
//! driven the ink, and that is a different fault from one that stalls.
//!
//! Reference figures measured on a XIAO ESP32-C3: A ~3891 ms, B and C ~1017 ms.
//!
//! ## Note on the SPI clock
//!
//! Deliberately 4 MHz, not the 16 MHz the other RP2350 examples use, to match the C3 diagnostic
//! exactly. Data transfer is never the bottleneck here — 4,000 bytes at 4 MHz is about 8 ms
//! against a multi-second refresh — but a controlled comparison should change one variable, and
//! this one is free to hold constant.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Good Display GDEM0213B74 2.13" Monochrome, 122x250
//! - **Adapter Board:** Good Display DESPI-C02, wired as in the other `epdsi` examples here
//!
//! ## Run
//!
//! ```bash
//! cargo run --example epd_diag_partial
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use epdsi::prelude::*;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSpi, Pin};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

const STRIDE: usize = GDEM0213B74::WIDTH.div_ceil(8) as usize;
const FRAME_BYTES: usize = STRIDE * GDEM0213B74::HEIGHT as usize;
const BAND_Y: u32 = 50;
const BAND_H: u32 = 200;
const BAND_BYTES: usize = STRIDE * BAND_H as usize;

/// Horizontal stripes, `phase` selecting which bands are black.
fn stripes(buf: &mut [u8], rows: usize, phase: usize) {
    buf.fill(0xFF);
    for row in 0..rows {
        if (row / 20) % 2 == phase {
            for b in 0..STRIDE {
                buf[row * STRIDE + b] = 0x00;
            }
        }
    }
}

#[hal::entry]
fn main() -> ! {
    defmt::info!("=== SSD1680 partial-refresh bisect (RP2350) ===");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

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

    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let rst = pins.gpio11.into_push_pull_output();
    // SSD1680 BUSY is active-HIGH
    let busy = pins.gpio13.into_pull_down_input();

    // 4 MHz to match the C3 diagnostic exactly — see the module docs.
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        4_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1680Controller::new(GDEM0213B74::WIDTH, GDEM0213B74::HEIGHT);
    let mut epd = EpdBuilder::<_, GDEM0213B74>::new(controller).build(epd_bus);

    defmt::info!("init...");
    epd.init(&mut timer).unwrap();

    let mut buf = [0xFFu8; FRAME_BYTES];

    // --- A: full frame, Full mode. Baseline. ---
    defmt::info!("--- A: full frame, Full mode --- (panel: horizontal stripes)");
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Full);
    stripes(&mut buf[..], GDEM0213B74::HEIGHT as usize, 0);

    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &buf[..]).unwrap();
    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &buf[..]).unwrap();

    let t = timer.get_counter().ticks();
    epd.refresh(&mut timer).unwrap();
    defmt::info!(
        "  A: {} ms  (C3 measured 7450; that repo's reference is ~3900)",
        (timer.get_counter().ticks() - t) / 1000
    );
    timer.delay_ms(3000);

    // --- B: full frame, Partial mode. ---
    defmt::info!("--- B: full frame, Partial mode --- (panel: stripes invert)");
    epd.controller_mut()
        .set_refresh_mode(Ssd168xRefreshMode::Partial);
    stripes(&mut buf[..], GDEM0213B74::HEIGHT as usize, 1);

    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &buf[..]).unwrap();

    let t = timer.get_counter().ticks();
    epd.refresh(&mut timer).unwrap();
    defmt::info!(
        "  B: {} ms  (C3 measured 98; that repo's reference is ~1017)",
        (timer.get_counter().ticks() - t) / 1000
    );
    timer.delay_ms(3000);

    // Keep the previous-image RAM in step so C diffs against what is on the panel.
    epd.set_window(0, 0, GDEM0213B74::WIDTH - 1, GDEM0213B74::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::RedYellow, &buf[..]).unwrap();

    // --- C: banded, Partial mode. ---
    defmt::info!("--- C: band y=50..249, Partial mode --- (panel: lower part inverts)");
    stripes(&mut buf[..BAND_BYTES], BAND_H as usize, 0);

    epd.set_window(0, BAND_Y, GDEM0213B74::WIDTH - 1, BAND_Y + BAND_H - 1)
        .unwrap();
    epd.set_cursor(0, BAND_Y).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &buf[..BAND_BYTES])
        .unwrap();

    let t = timer.get_counter().ticks();
    epd.refresh(&mut timer).unwrap();
    defmt::info!(
        "  C: {} ms  (C3 measured 333; that repo's reference is ~1017)",
        (timer.get_counter().ticks() - t) / 1000
    );

    defmt::info!("=== done ===");
    loop {
        timer.delay_ms(1000);
    }
}
