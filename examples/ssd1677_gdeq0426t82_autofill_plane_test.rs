//! # SSD1677 `0x46` / `0x47` plane-mapping test on GDEQ0426T82 (`epdsi`)
//!
//! Bench test for open lead F2: which RAM plane do the SSD1677's two auto-fill commands write?
//!
//! The datasheet (Rev 1.0, Nov 2018) contradicts itself. Its command table (p.34 and p.35) says
//! `0x46` is "Auto Write RED RAM" and `0x47` is "Auto Write B/W RAM", which is what
//! `Ssd1677Controller` uses. Its initialization procedure (p.41) says `0x46` fills RAM `0x24`
//! (B/W) and `0x47` fills RAM `0x26` (RED). M5GFX and TernOS follow p.41.
//!
//! ## How the test isolates one plane
//!
//! `Ssd1677RefreshMode::Full` sends `0x21 = [0x40, 0x00]`, which bypasses the RED RAM as zero, so a
//! full refresh shows the **B/W RAM (`0x24`) only**. Each screen below seeds both RAM planes
//! white with ordinary streamed writes, fires one auto-fill command with the "all black" pattern
//! (`0x77`: first step 0, maximum step height and width), then does a full refresh.
//!
//! ## Reading the result
//!
//! | Screen | Command sent | Panel after refresh | Meaning |
//! |---|---|---|---|
//! | 1 | none, B/W plane streamed black | black | sanity: the refresh path shows RAM `0x24` |
//! | 2 | `0x46 = 0x77` | black | `0x46` writes the B/W RAM (p.41, M5GFX, TernOS are right) |
//! | 2 | `0x46 = 0x77` | white | `0x46` writes the RED RAM (command table p.34 is right) |
//! | 3 | `0x47 = 0x77` | black | `0x47` writes the B/W RAM (command table p.35, epdsi) |
//! | 3 | `0x47 = 0x77` | white | `0x47` writes the RED RAM (p.41 is right, epdsi has the names swapped) |
//!
//! Screens 2 and 3 should come out opposite. If both are black or both are white, the
//! auto-fill is not doing what this test assumes: treat that as a different finding and stop.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Display:** Good Display GDEQ0426T82 4.26" (800x480) on a DESPI-C02
//! - Wiring is the same as `ssd1677_gdeq0426t82_epd`.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ssd1677_gdeq0426t82_autofill_plane_test
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

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

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// One full 1-bpp plane: 100 bytes per RAM row x 480 rows = 48,000 bytes.
const FRAME_BYTES: usize = GDEQ0426T82::WIDTH.div_ceil(8) as usize * GDEQ0426T82::HEIGHT as usize;

/// Auto-fill parameter for an all-black plane: `A[7]` = 0, step height `111`, step width `111`.
const PATTERN_BLACK: u8 = 0x77;

/// Seconds the result stays on screen for a photo before the next screen.
const HOLD_MS: u32 = 8000;

#[hal::entry]
fn main() -> ! {
    defmt::info!("Starting SSD1677 0x46/0x47 plane-mapping test (GDEQ0426T82)");
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

    // Instantiate epdsi SPI bus wrapper and dedicated SSD1677 controller
    let epd_bus = SpiBusWrapper::new(spi_device, dc, rst, busy);
    let controller = Ssd1677Controller::new(GDEQ0426T82::WIDTH, GDEQ0426T82::HEIGHT);

    // Build EPD Driver using epdsi with GDEQ0426T82 panel specification (800x480)
    let mut epd = EpdBuilder::<_, GDEQ0426T82>::new(controller).build(epd_bus);

    defmt::info!("Initializing SSD1677 epdsi EPD driver...");
    epd.init(&mut timer).unwrap();

    // Plain streamed buffers: this test must not depend on epdsi's own auto-fill mapping.
    let white = [0xFFu8; FRAME_BYTES];
    let black = [0x00u8; FRAME_BYTES];

    // Seed both planes white by streaming, then leave the window and cursor at the origin.
    let seed_white = |epd: &mut _| {
        for channel in [ColorChannel::BlackWhite, ColorChannel::RedYellow] {
            let epd: &mut EpdDriver<_, _, GDEQ0426T82> = epd;
            epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
                .unwrap();
            epd.set_cursor(0, 0).unwrap();
            epd.write_frame(channel, &white).unwrap();
        }
    };

    // --- Screen 1: sanity, streamed black into the B/W plane only -----------------------
    defmt::info!("--- Screen 1: streamed BLACK into 0x24 only. Expect BLACK. ---");
    seed_white(&mut epd);
    epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
        .unwrap();
    epd.set_cursor(0, 0).unwrap();
    epd.write_frame(ColorChannel::BlackWhite, &black).unwrap();
    epd.refresh(&mut timer).unwrap();
    timer.delay_ms(HOLD_MS);

    // --- Screens 2 and 3: one auto-fill command each, everything else white -----------
    for (screen, command) in [(2u8, 0x46u8), (3u8, 0x47u8)] {
        defmt::info!(
            "--- Screen {}: auto-fill 0x{:02X} = 0x77 on a white-seeded panel ---",
            screen,
            command
        );
        seed_white(&mut epd);
        // Re-assert the full window: the pattern generator paints the whole RAM area.
        epd.set_window(0, 0, GDEQ0426T82::WIDTH - 1, GDEQ0426T82::HEIGHT - 1)
            .unwrap();
        epd.set_cursor(0, 0).unwrap();
        {
            let (bus, _controller) = epd.split_mut();
            bus.send_command_with_data(command, &[PATTERN_BLACK])
                .unwrap();
            // BUSY is high while the generator sweeps.
            bus.wait_busy_with_delay(&mut timer, true).unwrap();
        }
        epd.refresh(&mut timer).unwrap();
        defmt::info!(
            "Screen {}: BLACK means 0x{:02X} wrote the B/W RAM, WHITE means it wrote the RED RAM.",
            screen,
            command
        );
        timer.delay_ms(HOLD_MS);
    }

    defmt::info!("Done. Screen 2 and screen 3 should be opposite colors.");

    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
