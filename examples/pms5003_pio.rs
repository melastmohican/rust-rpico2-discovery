//! # PMS5003 PM2.5 Air Quality Sensor (Non-blocking UART) Example
//!
//! This example demonstrates a "from scratch" low-level implementation of the
//! PMS5003 UART protocol. It uses a **split hardware UART** (UART1) to perform
//! non-blocking RX on GPIO9.
//!
//! ## Non-blocking Strategy
//!
//! - **UART Split**: The UART peripheral is split into separate RX and TX handles.
//! - **nb::Read**: Uses the `embedded_hal_nb` trait to pull single bytes from the
//!   hardware FIFO only when they are available (`WouldBlock` is handled).
//! - **Manual Parser**: Implements a 32-byte state machine to hunt for the preamble
//!   (`0x42 0x4D`) and verify the checksum manually.
//!
//! This version is ideal for integration into async executors or control loops
//! where blocking a core for 32ms (the time to receive one frame at 9600 baud)
//! is unacceptable.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Sensor:** Adafruit PMS5003 with breadboard adapter
//!
//! ## Wiring
//!
//! | PMS5003 Pin | Pico 2 Pin     | GPIO  | Function         |
//! |-------------|----------------|-------|------------------|
//! | VCC         | VBUS (Pin 40)  | -     | 5V Power         |
//! | GND         | GND  (Pin 38)  | -     | Ground           |
//! | TXD         | GP9  (Pin 12)  | GPIO9 | MCU RX (UART1 RX)|
//! | RXD         | GP8  (Pin 11)  | GPIO8 | MCU TX (UART1 TX)|
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pms5003_pio
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal_nb::serial::Read;
use hal::block::ImageDef;
use hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_hal::clocks::ClockSource;
use rp235x_hal::fugit::RateExtU32;

/// Tell the Boot ROM about our application.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// ---------------------------------------------------------------------------
// PMS5003 frame parser
// ---------------------------------------------------------------------------

const PMS_FRAME_LEN: usize = 32;
const PMS_START1: u8 = 0x42;
const PMS_START2: u8 = 0x4D;

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct PmsFrame {
    pub pm1_0: u16,
    pub pm2_5: u16,
    pub pm10: u16,
}

fn parse_frame(buf: &[u8; PMS_FRAME_LEN]) -> Option<PmsFrame> {
    if buf[0] != PMS_START1 || buf[1] != PMS_START2 {
        return None;
    }
    let checksum: u16 = buf[..30].iter().map(|&b| b as u16).sum();
    let expected = ((buf[30] as u16) << 8) | buf[31] as u16;
    if checksum != expected {
        return None;
    }
    Some(PmsFrame {
        pm1_0: ((buf[10] as u16) << 8) | buf[11] as u16,
        pm2_5: ((buf[12] as u16) << 8) | buf[13] as u16,
        pm10: ((buf[14] as u16) << 8) | buf[15] as u16,
    })
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[hal::entry]
fn main() -> ! {
    defmt::info!("PMS5003 non-blocking UART RX example");

    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let uart_pins = (
        pins.gpio8.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio9.into_function::<hal::gpio::FunctionUart>(),
    );

    let uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.get_freq(),
        )
        .unwrap();

    let (mut uart_rx, _uart_tx) = uart.split();

    defmt::info!("UART1 RX non-blocking on GPIO9 at 9600 baud — entering frame loop");

    let mut buf = [0u8; PMS_FRAME_LEN];
    let mut pos: usize = 0;
    let mut synced = false;

    loop {
        match uart_rx.read() {
            Ok(byte) => {
                if !synced {
                    match pos {
                        0 if byte == PMS_START1 => {
                            buf[0] = byte;
                            pos = 1;
                        }
                        1 if byte == PMS_START2 => {
                            buf[1] = byte;
                            pos = 2;
                            synced = true;
                        }
                        _ => {
                            pos = 0;
                        }
                    }
                } else {
                    buf[pos] = byte;
                    pos += 1;
                    if pos == PMS_FRAME_LEN {
                        match parse_frame(&buf) {
                            Some(frame) => {
                                defmt::info!("--- PMS5003 Report ---");
                                defmt::info!("PM1.0: {} ug/m3", frame.pm1_0);
                                defmt::info!("PM2.5: {} ug/m3", frame.pm2_5);
                                defmt::info!("PM10:  {} ug/m3", frame.pm10);
                            }
                            None => defmt::warn!("checksum fail — re-syncing"),
                        }
                        pos = 0;
                        synced = false;
                    }
                }
            }
            Err(nb::Error::WouldBlock) => {
                // FIFO empty — yield briefly rather than spinning at 150 MHz
                timer.delay_us(50);
            }
            Err(nb::Error::Other(_)) => {
                // Framing or overrun error — drop sync and rescan
                pos = 0;
                synced = false;
            }
        }
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"PMS5003 manual non-blocking UART RX"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
