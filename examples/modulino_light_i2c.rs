#![no_std]
#![no_main]

//! # Arduino Modulino Light Example for Raspberry Pi Pico 2
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! It reads RGB, IR, and Lux values from the Modulino Light sensor (LTR-381RGB)
//! over I2C and prints them using `defmt`.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Light
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x53 (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
//!
//! ```
//! Modulino Light -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! Run with `cargo run --example modulino_light_i2c`.

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;
use hal::fugit::RateExtU32;
use hal::{
    I2C, Sio, Timer, Watchdog,
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, Pin},
    pac,
};
use rp235x_hal as hal;

use modulino::Light;
use rust_rpico2_discovery::Fmt;

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
        100_u32.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Light...");

    let mut light = Light::new(i2c);

    // init() sets 18x Gain, 16-bit Resolution, and 25ms Rate (Arduino defaults)
    if let Err(e) = light.init() {
        error!("Failed to init Light sensor: {:?}", Debug2Format(&e));
        loop {
            timer.delay_ms(1000);
        }
    }

    info!("Light sensor initialized!");

    loop {
        // Read all channels and calculate Lux/Color
        match light.read() {
            Ok(meas) => {
                let color = meas.color_name();
                info!(
                    "R: {}, G: {}, B: {}, IR: {}, Raw Lux: {}, Lux: {}, Color: {}",
                    meas.red,
                    meas.green,
                    meas.blue,
                    meas.ir,
                    meas.raw_lux,
                    Fmt(meas.lux),
                    color
                );
            }
            Err(e) => {
                error!("Light read error: {:?}", Debug2Format(&e));
            }
        }

        timer.delay_ms(500);
    }
}
