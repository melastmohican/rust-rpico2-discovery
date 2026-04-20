//! # HC-SR501 PIR Motion Sensor Example
//!
//! This example demonstrates how to interface with an HC-SR501 PIR motion sensor.
//! It tracks the motion state and logs "Motion detected!" and "Motion ended!" via RTT.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** HC-SR501 PIR Motion Sensor
//!
//! ## Wiring
//!
//! ```
//! HC-SR501 Pin -> RPi Pico 2
//! ----------    --------------
//! VCC          -> VBUS (Pin 40) - 5V required
//! GND          -> GND (Pin 38)
//! OUT          -> GPIO16 (Pin 21)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example hc_sr501
//! ```

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use hal::block::ImageDef;
use panic_probe as _;
use rp235x_hal as hal;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {
    info!("HC-SR501 test");

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

    // Configure the LED pin
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Configure the HC-SR501 OUT pin
    let mut pir_input = pins.gpio16.into_pull_down_input();

    let mut pir_state = false;

    loop {
        // Read the sensor state
        let val = pir_input.is_high().unwrap();

        if val {
            // Motion detected
            let _ = led_pin.set_high();

            if !pir_state {
                info!("Motion detected!");
                pir_state = true;
            }
        } else {
            // No motion
            let _ = led_pin.set_low();

            if pir_state {
                info!("Motion ended!");
                pir_state = false;
            }
        }

        // Small delay to avoid tight loop
        timer.delay_ms(100);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"HC-SR501 PIR sensor example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
