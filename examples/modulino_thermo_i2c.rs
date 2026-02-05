//! # Arduino Modulino Thermo Example for Raspberry Pi Pico 2
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Reads temperature and humidity from the Arduino Modulino Thermo module (HS3003) over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Thermo (HS3003)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x44 (fixed)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Thermo.
//! The cable provides:
//! ```
//!      Modulino Thermo -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! Run with `cargo run --example modulino_thermo_i2c`.

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
use modulino::Thermo;

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

    info!("Initializing Arduino Modulino Thermo...");

    // Create Modulino Thermo driver
    // The HS3003 has a fixed address of 0x44
    let mut thermo = Thermo::new(i2c);

    info!(
        "Modulino Thermo initialized at address 0x{:02X}!",
        thermo.address()
    );
    info!("Starting measurements...");

    loop {
        // Read temperature and humidity
        // The read method requires a delay provider to wait for the measurement to complete
        match thermo.read(&mut timer) {
            Ok(measurement) => {
                info!(
                    "Temperature: {} °C, Humidity: {} %",
                    measurement.temperature, measurement.humidity
                );
            }
            Err(e) => {
                error!("Failed to read sensor: {:?}", Debug2Format(&e));
            }
        }

        // Wait 1 second before next measurement
        timer.delay_ms(1000);
    }
}
