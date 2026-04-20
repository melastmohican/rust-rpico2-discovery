//! # HLK-LD2410C Human Presence Radar Example
//!
//! This example demonstrates how to interface with the HLK-LD2410C 24GHz mmWave radar sensor
//! using the `ld2410` crate.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** HLK-LD2410C (24GHz Human Presence Sensing Module)
//!
//! ## Wiring
//!
//! | HLK-LD2410C Pin | RPi Pico 2     | Notes                      |
//! |-----------------|----------------|----------------------------|
//! | VCC (5V)        | VBUS (Pin 40)  | Sensor requires 5V supply  |
//! | GND             | GND (Pin 38)   |                            |
//! | TX              | GPIO9 (Pin 12) | MCU RX connects to Sensor TX |
//! | RX              | GPIO8 (Pin 11) | MCU TX connects to Sensor RX |
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ld2410
//! ```
//!
//! ## Expected Output
//!
//! When properly connected, you should see continuous reports like:
//!
//! ```text
//! [INFO ] HLK-LD2410C Human Presence Radar Sensor
//! [INFO ] Initializing LD2410 UART at 256000 baud
//! [INFO ] Connect Sensor TX -> GPIO9 and Sensor RX -> GPIO8
//! --- HLK-LD2410C Report ---
//! Target State: Both
//! Moving Target: 30 cm (Energy: 100)
//! Static Target: 30 cm (Energy: 0)
//! Detection Distance: 100 cm
//! --------------------------
//! ```

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;
use hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use ld2410::LD2410;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_hal::clocks::ClockSource;
use rp235x_hal::fugit::RateExtU32;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {
    info!("HLK-LD2410C Human Presence Radar Sensor");

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

    // Configure UART1 for LD2410 communication
    // LD2410 default baud rate is 256000
    let uart_pins = (
        pins.gpio8.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio9.into_function::<hal::gpio::FunctionUart>(),
    );

    let uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(256_000u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.get_freq(),
        )
        .unwrap();

    info!("Initializing LD2410 UART at 256000 baud");
    info!("Connect Sensor TX -> GPIO9 and Sensor RX -> GPIO8");

    let mut radar = LD2410::new(uart);
    let mut last_dist: Option<u16> = None;

    loop {
        match radar.read_presence() {
            Ok(data) => {
                println!("--- HLK-LD2410C Report ---");

                let state_str = match data.target_state {
                    ld2410::TargetState::NoTarget => "NoTarget",
                    ld2410::TargetState::Moving => "Moving",
                    ld2410::TargetState::Stationary => "Stationary",
                    ld2410::TargetState::Both => "Both",
                };
                println!("Target State: {}", state_str);

                println!(
                    "Moving Target: {} cm (Energy: {})",
                    data.moving_distance_cm, data.moving_energy
                );

                // Calculate approximate speed based on distance change
                if let Some(prev) = last_dist {
                    let diff = data.moving_distance_cm.abs_diff(prev);

                    if diff > 0 {
                        let direction = if data.moving_distance_cm < prev {
                            "Approaching"
                        } else {
                            "Receding"
                        };
                        println!("Motion: {} (delta: {} cm)", direction, diff);
                    }
                }
                last_dist = Some(data.moving_distance_cm);
                println!(
                    "Static Target: {} cm (Energy: {})",
                    data.still_distance_cm, data.still_energy
                );
                println!("Detection Distance: {} cm", data.detection_distance_cm);
                println!("--------------------------");
            }
            Err(_) => {
                // Ignore errors (timeouts, partial frames)
            }
        }

        // Small delay to avoid tight loop
        timer.delay_ms(10);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"HLK-LD2410C radar sensor example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
