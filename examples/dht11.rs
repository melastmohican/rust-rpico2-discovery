//! # DHT11 Temperature & Humidity Sensor Example
//!
//! Reads temperature and humidity from a DHT11 sensor using a single GPIO pin.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** DHT11 Temperature & Humidity Sensor
//!
//! ## Wiring Schematic
//!
//! ```text
//!                      Raspberry Pi Pico 2             DHT11 Module
//!                    +---------------------+      +---------------------+
//!                    |                     |      |                     |
//!                    | GND (Pin 38) -------+----->| GND                 |
//!                    | 3V3 (Pin 36) -------+----->| VCC                 |
//!                    | GPIO16 (Pin 21) ----+----->| DAT (Data)          |
//!                    |                     |      |                     |
//!                    +---------------------+      +---------------------+
//! ```
//!
//! > [!IMPORTANT]
//! > **Pull-up Resistor:**
//! > - **If using a DHT11 module board:** It likely already has a built-in pull-up resistor. No extra component is needed.
//! > - **If using a bare 4-pin DHT11 sensor:** You must add an external 4.7kΩ to 10kΩ pull-up resistor between the DAT (Data) and VCC lines.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example dht11 --release
//! ```
//! Note: Due to timing sensitivity of the DHT11 protocol, you must run this example in **release** mode.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use dht_sensor::*;
use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;
use hal::gpio::Pins;
use hal::pac;
use rp235x_hal as hal;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    info!("DHT11 Temperature & Humidity Sensor Example");

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
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

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Initializing DHT11 sensor on GPIO16 (Pin 21)...");

    // Configure GPIO16 (Pin 21) as pull-up input initially
    let wire_pin = pins.gpio16.into_pull_up_input();
    // Use the built-in `InOutPin` wrapper to emulate high-speed open-drain behavior
    let mut dht_pin = hal::gpio::InOutPin::new(wire_pin);

    info!("DHT11 initialized. Starting reading loop (every 2 seconds)...");

    loop {
        // Perform a blocking read of the DHT11 sensor
        match dht11::blocking::read(&mut timer, &mut dht_pin) {
            Ok(reading) => {
                info!(
                    "Temperature: {}°C | Humidity: {}%",
                    reading.temperature,
                    reading.relative_humidity
                );
            }
            Err(e) => {
                // We print errors as info/warn since transient errors are common with DHT11 sensors
                match e {
                    DhtError::Timeout => {
                        warn!(
                            "Error: Reading timed out. Verify wiring and ensure running in --release mode."
                        );
                    }
                    DhtError::ChecksumMismatch => {
                        warn!(
                            "Error: Checksum mismatch. The data might have been corrupted."
                        );
                    }
                    _ => {
                        warn!("Error reading sensor: {:?}", defmt::Debug2Format(&e));
                    }
                }
            }
        }

        // The DHT11 is slow and should not be polled more than once every 2 seconds
        timer.delay_ms(2000);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"DHT11 Temperature & Humidity Sensor example for RPi Pico 2"
    ),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
