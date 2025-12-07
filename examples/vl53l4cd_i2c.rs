//! # VL53L4CD Distance Sensor Example for Raspberry Pi Pico 2
//!
//! Reads distance from a VL53L4CD sensor over I2C0 using a blocking driver.
//!
//! ## Hardware
//!
//! - **Sensor:** VL53L4CD Time-of-Flight Distance Sensor (e.g., Arduino Modulino Distance)
//! - **Connection:** I2C
//! - **I2C Address:** 0x29 (default)
//!
//! ## Wiring
//!
//! ```
//!      VL53L4CD Sensor -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## Resources
//!
//! - [Arduino Modulino Distance documentation](https://docs.arduino.cc/hardware/modulino-distance/)
//!
//! Run with `cargo run --example vl53l4cd_i2c`.

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
use rp235x_hal::block::ImageDef;
use vl53l4cd_ulp::VL53L4cd;

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

    let mut sensor_timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let mut delay_timer = Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

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

    // Create a VL53L4CD driver instance

    let mut sensor = VL53L4cd::new(i2c, &mut sensor_timer);

    // Initialize the sensor

    if let Err(e) = sensor.sensor_init() {
        error!(
            "Failed to initialize VL53L4CD: {:?}",
            defmt::Debug2Format(&e)
        );

        loop {
            cortex_m::asm::wfi();
        }
    }

    sensor.start_ranging().unwrap();

    info!("VL53L4CD initialized successfully!");

    info!("Starting measurements...");

    loop {
        // Check if data is ready

        if let Ok(true) = sensor.check_for_data_ready() {
            match sensor.get_estimated_measurement() {
                Ok(measurement) => {
                    info!("Distance: {} mm", measurement.estimated_distance_mm);
                }

                Err(e) => {
                    error!(
                        "Error reading VL53L4CD sensor: {:?}",
                        defmt::Debug2Format(&e)
                    );
                }
            }

            sensor.clear_interrupt().unwrap();
        }

        // Wait a bit before checking again

        delay_timer.delay_ms(50);
    }
}
