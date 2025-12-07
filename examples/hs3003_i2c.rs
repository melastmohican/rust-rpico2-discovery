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
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use hal::timer::CopyableTimer0;
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// HS3003 I2C address (fixed)
const HS3003_ADDR: u8 = 0x44;

/// HS3003 measurement data structure
struct Hs3003Measurement {
    humidity: f32,    // Relative humidity in %
    temperature: f32, // Temperature in °C
}

/// Read temperature and humidity from HS3003 sensor
fn read_hs3003<I2cDev: embedded_hal::i2c::I2c>(
    i2c: &mut I2cDev,
    delay: &mut hal::Timer<CopyableTimer0>,
) -> Result<Hs3003Measurement, I2cDev::Error> {
    // Send measurement trigger (write with no data)
    i2c.write(HS3003_ADDR, &[0x00])?;

    // Wait for measurement to complete (minimum 33ms per datasheet)
    delay.delay_ms(100);

    // Read 4 bytes of data
    let mut data = [0u8; 4];
    i2c.read(HS3003_ADDR, &mut data)?;

    // Note: Status bits in this sensor appear unreliable, so we parse data directly
    // Debug testing showed valid temperature/humidity readings regardless of status bits
    Ok(parse_hs3003_data(&data))
}

/// Parse raw 4-byte data from HS3003 into temperature and humidity
fn parse_hs3003_data(data: &[u8; 4]) -> Hs3003Measurement {
    // Parse humidity (first 2 bytes, 14-bit value)
    // The top 2 bits of the first byte are status flags and must be masked out.
    let humidity_raw = (((data[0] as u16) & 0x3F) << 8) | (data[1] as u16);
    let humidity = (humidity_raw as f32 / 16383.0) * 100.0;

    // Parse temperature (last 2 bytes, 14-bit value)
    let temp_raw = ((data[2] as u16) << 8) | (data[3] as u16);
    let temp_value = (temp_raw >> 2) & 0x3FFF; // Extract 14-bit value
    let temperature = ((temp_value as f32 / 16383.0) * 165.0) - 40.0;

    Hs3003Measurement {
        humidity,
        temperature,
    }
}

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
    let mut i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("HS3003 sensor ready! I2C address: 0x{:02X}", HS3003_ADDR);
    info!("Starting measurements...");

    // Allow sensor to stabilize
    timer.delay_ms(100);

    loop {
        // Read sensor
        match read_hs3003(&mut i2c, &mut timer) {
            Ok(measurement) => {
                info!(
                    "Temperature: {}.{:02} °C | Humidity: {}.{:02} %",
                    measurement.temperature as i32,
                    ((measurement.temperature.abs() % 1.0) * 100.0) as u32,
                    measurement.humidity as u32,
                    ((measurement.humidity % 1.0) * 100.0) as u32,
                );
            }
            Err(e) => {
                error!("Error reading HS3003 sensor: {:?}", defmt::Debug2Format(&e));
                // Wait longer after error to allow sensor to recover
                timer.delay_ms(500);
            }
        }

        // Wait 2 seconds between measurements to allow sensor to be ready
        // HS3003 needs adequate time between measurement cycles
        timer.delay_ms(2000);
    }
}