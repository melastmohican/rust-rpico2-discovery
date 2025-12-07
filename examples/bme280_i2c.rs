//! # BME280 Temperature/Humidity/Pressure Sensor Example for Raspberry Pi Pico 2
//!
//! Reads temperature, humidity, and atmospheric pressure from a BME280 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit BME280 Temperature Humidity Pressure Sensor (or compatible)
//! - **Connection:** I2C
//! - **I2C Address:** 0x77
//!
//! ## Wiring
//!
//! ```
//!      BME280 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## I2C Address
//!
//! The BME280 can have two I2C addresses:
//! - 0x76 (SDO pin to GND) - use `BME280::new_primary()`
//! - 0x77 (SDO pin to VCC) - use `BME280::new_secondary()`
//!
//! The Adafruit BME280 uses address 0x77 by default, which matches the device detected in the i2c_scan.
//!
//! Run with `cargo run --example bme280_i2c`.

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

// The BME280 driver
use bme280::i2c::BME280;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
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

    let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure I2C0 pins
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    // Create I2C0 peripheral
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Create a BME280 driver instance
    let mut bme280 = BME280::new_secondary(i2c);

    // Initialize the sensor
    if let Err(e) = bme280.init(&mut timer) {
        error!("Failed to initialize BME280: {:?}", defmt::Debug2Format(&e));
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("BME280 initialized successfully!");
    info!("Starting measurements...");

    loop {
        // Take a measurement
        match bme280.measure(&mut timer) {
            Ok(measurements) => {
                info!(
                    "Temperature: {} C, Pressure: {} hPa, Humidity: {} %",
                    measurements.temperature,
                    measurements.pressure / 100.0,
                    measurements.humidity
                );
            }
            Err(e) => {
                error!("Error reading BME280 sensor: {:?}", defmt::Debug2Format(&e));
            }
        }

        // Wait 1 second between measurements
        timer.delay_ms(1000);
    }
}
