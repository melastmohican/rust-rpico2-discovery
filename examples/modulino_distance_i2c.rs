//! # Arduino Modulino Distance Example for Raspberry Pi Pico 2
//!
//! This example uses the **modulino** library: https://github.com/melastmohican/modulino-rs
//!
//! Reads distance from the Arduino Modulino Distance module (VL53L4CD) over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Distance (VL53L4CD)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x29 (default)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Distance.
//! The cable provides:
//! ```
//!      Modulino Distance -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! Run with `cargo run --example modulino_distance_i2c`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;

// Import from modulino library
use modulino::Distance;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
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

    let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    // Use TIMER1 for application delays so we don't block the sensor timer logic if it was shared (though init takes &mut)
    let mut delay_timer = Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Distance...");

    // Create Modulino Distance driver
    let mut distance = Distance::new(i2c);

    // Initialize the sensor (loads firmware, sets tuning)
    info!("Waiting for sensor boot and loading firmware...");
    // We pass `timer` for the initialization delays
    if let Err(e) = distance.init(&mut timer) {
        error!(
            "Failed to initialize Modulino Distance: {:?}",
            Debug2Format(&e)
        );
        loop {
            delay_timer.delay_ms(1000);
        }
    }

    info!(
        "Modulino Distance initialized at address 0x{:02X}!",
        distance.address()
    );

    // Start continuous ranging
    if let Err(e) = distance.start_ranging() {
        error!("Failed to start ranging: {:?}", Debug2Format(&e));
    }
    info!("Ranging started...");

    loop {
        // Check if data is ready
        match distance.data_ready() {
            Ok(ready) => {
                if ready {
                    // Read distance
                    match distance.read_distance() {
                        Ok(Some(mm)) => {
                            info!("Distance: {} mm", mm);
                        }
                        Ok(None) => {
                            // If None is returned, check raw status if possible
                            if let Ok(status) = distance.read_range_status() {
                                info!("Invalid measurement (Status: {})", status);
                            } else {
                                info!("Invalid measurement");
                            }
                        }
                        Err(e) => {
                            error!("Failed to read distance: {:?}", Debug2Format(&e));
                        }
                    }
                }
            }
            Err(e) => {
                error!("Failed to check data ready: {:?}", Debug2Format(&e));
            }
        }

        // Poll interval - increase to 500ms to reduce output frequency
        delay_timer.delay_ms(500);
    }
}
