//! # HS3003 Temperature/Humidity Sensor Example for Raspberry Pi Pico 2
//!
//! Reads temperature and humidity from an HS3003 sensor over I2C0.
//!
//! This example is configured for the **Arduino Modulino Thermo** connected via **Qwiic/STEMMA QT** cable.
//!
//! ## Hardware
//!
//! - **Sensor:** Arduino Modulino Thermo (Renesas HS3003)
//! - **Connection:** I2C
//! - **I2C Address:** 0x44 (fixed for HS3003)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Thermo.
//! The cable provides:
//! ```
//!      Modulino -> RPi Pico 2
//! (black)  GND  -> GND
//! (red)    VCC  -> 3.3V
//! (yellow) SCL  -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA  -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## About HS3003
//!
//! The Renesas HS3003 is a high-performance temperature and humidity sensor:
//! - Temperature range: -40°C to +125°C (±0.2°C accuracy)
//! - Humidity range: 0% to 100% RH (±1.5% accuracy)
//! - 14-bit resolution for both measurements
//! - Ultra-low power consumption
//!
//! Run with `cargo run --example hs3003_i2c`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use embedded_hal::delay::DelayNs;
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::block::ImageDef;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use hs3003::Hs3003;
use rp235x_hal as hal;
use rp235x_hal::entry;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    info!("HS3003 Sensor Example for RP2350");

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

    let mut delay = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

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
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Create sensor instance
    let mut sensor = Hs3003::new(i2c);
    info!("Starting measurements...");

    loop {
        match sensor.read(&mut delay) {
            Ok(measurement) => {
                info!(
                    "Temperature: {}°C, Humidity: {}%",
                    measurement.temperature, measurement.humidity
                );
            }
            Err(_) => {
                error!("Failed to read sensor");
            }
        }
        delay.delay_ms(2000u32);
    }
}
