//! # ADXL345 3-Axis Accelerometer Example
//!
//! Reads accelerometer data from an ADXL345 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** ADXL345 3-Axis Digital Accelerometer
//!
//! ## Wiring
//!
//! ```
//!      ADXL345 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example adxl345_i2c
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use adxl345_eh_driver as adxl345;
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

    info!("Initializing ADXL345...");

    // Create a new ADXL345 driver instance
    // For I2C, use the secondary address 0x53 (common for breakouts)
    let mut accel = adxl345::Driver::new(i2c, Some(adxl345::address::SECONDARY)).unwrap();

    info!("ADXL345 initialized successfully!");
    info!("Starting measurements...");

    loop {
        // Read accelerometer data in Gs
        match accel.get_accel() {
            Ok((x, y, z)) => {
                info!("x: {}G, y: {}G, z: {}G", x, y, z);
            }
            Err(e) => {
                error!("Error reading accelerometer data: {:?}", Debug2Format(&e));
            }
        }

        // Wait 1 second between measurements
        timer.delay_ms(1000);
    }
}
