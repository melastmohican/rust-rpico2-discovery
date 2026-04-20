//! # BME680/BME688 Environment Sensor Example
//!
//! Reads temperature, humidity, atmospheric pressure, and gas resistance (VOCs)
//! from a BME680 or BME688 sensor over I2C.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** Adafruit BME688 Temperature Humidity Pressure Gas Sensor
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the sensor.
//! ```text
//!      BME688 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example bme680_i2c
//! ```
//!
//! ## Notes

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use bosch_bme680::{Bme680, Configuration, DeviceAddress};
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
use rp235x_hal as hal;
use rp235x_hal::entry;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
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

    let mut sensor_delay = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut loop_delay = Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Initializing BME680/BME688 sensor...");

    // Configure I2C0 pins
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    // Create I2C0 peripheral with default configuration (100kHz)
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Initialize configuration (default values for oversampling and filter)
    let config = Configuration::default();

    // Initialize the sensor
    // Address: 0x77 (Secondary) for Adafruit BME688
    // ambient_temp: 25°C (used for initial calibration)
    let mut bme680 = match Bme680::new(
        i2c,
        DeviceAddress::Secondary,
        &mut sensor_delay,
        &config,
        25,
    ) {
        Ok(s) => {
            info!("BME680/BME688 initialized successfully!");
            s
        }
        Err(_e) => {
            info!("Failed to initialize BME680/BME688!");
            info!("Check wiring and I2C address (default 0x77)");
            loop {
                loop_delay.delay_ms(1000u32);
            }
        }
    };

    info!("Starting measurements...");

    loop {
        // Take a measurement
        match bme680.measure() {
            Ok(values) => {
                info!("--- Measurements ---");
                info!(
                    "Temperature: {}.{:02} °C",
                    values.temperature as i32,
                    ((values.temperature % 1.0) * 100.0) as u32
                );
                info!(
                    "Pressure:    {}.{:02} hPa",
                    values.pressure as i32,
                    ((values.pressure % 1.0) * 100.0) as u32
                );
                info!(
                    "Humidity:    {}.{:02} %",
                    values.humidity as i32,
                    ((values.humidity % 1.0) * 100.0) as u32
                );

                if let Some(gas) = values.gas_resistance {
                    info!("Gas Res.:    {} Ohms", gas as u32);
                } else {
                    info!("Gas Res.:    (Not ready or heater disabled)");
                }
            }
            Err(_e) => {
                info!("Error reading sensor data!");
            }
        }

        // Wait 2 seconds between measurements (gas sensing takes time)
        loop_delay.delay_ms(2000u32);
    }
}
