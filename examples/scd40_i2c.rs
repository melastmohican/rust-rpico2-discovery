//! # SCD40 CO2/Temperature/Humidity Sensor Example for Raspberry Pi Pico 2
//!
//! Reads CO2 concentration, temperature, and humidity from an SCD40 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Sensor:** Apollo Automation SCD40 Breakout (or compatible Sensirion SCD4x)
//! - **Connection:** I2C
//! - **I2C Address:** 0x62 (Fixed for SCD4x)
//!
//! ## Wiring
//!
//! ```
//!      SCD40 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! Run with `cargo run --example scd40_i2c`.

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

// The SCD4x driver
use scd4x::Scd4x;

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

    let timer0 = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut timer1 = Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

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

    // Create a SCD4x driver instance
    // The driver uses embedded-hal 1.0 traits
    let mut scd40 = Scd4x::new(i2c, timer0);

    info!("SCD40: Stopping periodic measurement (in case it was running)...");
    // Stop periodic measurement to ensure we can send commands
    if let Err(e) = scd40.stop_periodic_measurement() {
        warn!(
            "Failed to stop periodic measurement (might already be stopped): {:?}",
            defmt::Debug2Format(&e)
        );
    }

    // Wait a bit after stopping
    timer1.delay_ms(500);

    info!("SCD40: Initializing...");

    // Start periodic measurement
    if let Err(e) = scd40.start_periodic_measurement() {
        error!(
            "Failed to start periodic measurement: {:?}",
            defmt::Debug2Format(&e)
        );
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("SCD40 initialized successfully!");
    info!("Waiting for first measurement (approx 5 seconds)...");

    loop {
        // Check if data is ready
        match scd40.data_ready_status() {
            Ok(true) => match scd40.measurement() {
                Ok(m) => {
                    info!(
                        "CO2: {} ppm, Temperature: {} C, Humidity: {} %",
                        m.co2, m.temperature, m.humidity
                    );
                }
                Err(e) => {
                    error!(
                        "Error reading SCD40 measurement: {:?}",
                        defmt::Debug2Format(&e)
                    );
                }
            },
            Ok(false) => {
                // Data not ready yet
            }
            Err(e) => {
                error!(
                    "Error checking SCD40 data ready status: {:?}",
                    defmt::Debug2Format(&e)
                );
            }
        }

        // The sensor updates every 5 seconds in normal mode
        timer1.delay_ms(1000);
    }
}
