//! # Arduino Modulino Joystick Example
//!
//! Reads joystick position and button state over I2C and prints values via RTT.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** Arduino Modulino Joystick
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Joystick.
//! The cable provides:
//! ```
//!      Modulino Joystick -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_joystick_i2c
//! ```
 
#![no_std]
#![no_main]

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

use modulino::Joystick;
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

    info!("Initializing Arduino Modulino Joystick...");

    let mut joystick = match Joystick::new(i2c) {
        Ok(j) => j,
        Err(e) => {
            error!("Failed to init Joystick: {:?}", Debug2Format(&e));
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Joystick initialized at address 0x{:02X}",
        joystick.address()
    );

    loop {
        if let Err(e) = joystick.update() {
            error!("Joystick update error: {:?}", Debug2Format(&e));
        } else {
            let (x, y) = joystick.position();
            let btn = joystick.button_pressed();
            let angle = joystick.angle();
            let mag = joystick.magnitude();
            info!(
                "Pos: ({}, {}), Button: {}, Angle: {}, Mag: {}",
                x,
                y,
                btn,
                Fmt(angle),
                Fmt(mag)
            );
        }
        timer.delay_ms(200);
    }
}
