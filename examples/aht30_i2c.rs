//! # AHT30 Temperature/Humidity Sensor Example
//!
//! Reads temperature and humidity from an AHT30 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** AHT30 Temperature & Humidity Sensor (or compatible)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the sensor.
//! The cable provides:
//! ```
//!        Sensor -> RPi Pico 2
//! (black)  GND  -> GND
//! (red)    VCC  -> 3.3V
//! (yellow) SCL  -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA  -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example aht30_i2c
//! ```
//!
//! ## About AHT30
//!
//! The AHT30 is a high-precision, fully calibrated temperature and humidity sensor:
//! - Temperature range: -40°C to +85°C (±0.3°C accuracy)
//! - Humidity range: 0% to 100% RH (±2% accuracy)
//! - Communicates via I2C at default address 0x38.

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
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use aht20_driver::{AHT20, SENSOR_ADDRESS};
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    info!("AHT30 Sensor Example for RP2350");

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

    // Create I2C0 peripheral (100kHz is a standard safe speed for I2C)
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Instantiate AHT20/30 driver
    let mut aht20_uninit = AHT20::new(i2c, SENSOR_ADDRESS);

    // Calibrate sensor (this consumes the uninit struct and returns an initialized one)
    let mut aht = match aht20_uninit.init(&mut timer) {
        Ok(sensor) => {
            info!("AHT30 initialized and calibrated successfully!");
            sensor
        }
        Err(e) => {
            error!("Failed to initialize AHT30: {:?}", defmt::Debug2Format(&e));
            loop {
                cortex_m::asm::wfi();
            }
        }
    };

    info!("Starting measurements...");

    loop {
        // Read sensor values
        match aht.measure(&mut timer) {
            Ok(measurement) => {
                let temp = measurement.temperature;
                let temp_scaled = (temp * 100.0) as i32;
                let temp_whole = temp_scaled / 100;
                let temp_frac = (temp_scaled % 100).abs();

                let hum = measurement.humidity;
                let hum_scaled = (hum * 100.0) as i32;
                let hum_whole = hum_scaled / 100;
                let hum_frac = (hum_scaled % 100).abs();

                info!(
                    "Temperature: {}.{=i32:02} °C, Humidity: {}.{=i32:02} %",
                    temp_whole,
                    temp_frac,
                    hum_whole,
                    hum_frac
                );
            }
            Err(e) => {
                error!("Error reading AHT30 sensor: {:?}", defmt::Debug2Format(&e));
            }
        }

        // Wait 2 seconds between measurements
        timer.delay_ms(2000);
    }
}
