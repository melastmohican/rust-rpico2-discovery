//! # Arduino Modulino Knob Example for Raspberry Pi Pico 2
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Reads rotary encoder value and button state from the Arduino Modulino Knob module.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Knob (Rotary Encoder)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x3A (default, or 0x3B)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Knob.
//! The cable provides:
//! ```
//!      Modulino Knob -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! Run with `cargo run --example modulino_knob_i2c`.

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

use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;

// Import from modulino library
use modulino::Knob;

/// Tell the Boot ROM about our application
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
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Knob...");

    // Create Modulino Knob driver
    let mut knob = match Knob::new(i2c) {
        Ok(k) => k,
        Err(e) => {
            error!("Failed to initialize Modulino Knob: {:?}", Debug2Format(&e));
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Knob initialized at address 0x{:02X}!",
        knob.address()
    );

    // Set range to 0-100 (e.g. for volume)
    knob.set_range(0, 100);
    // Reset starting value to 50
    if let Err(e) = knob.set_value(50) {
        error!("Failed to set initial value: {:?}", Debug2Format(&e));
    }

    info!("Turn the knob! (Range 0-100)");

    let mut prev_value = knob.value();
    let mut prev_pressed = knob.pressed();

    loop {
        // Poll for updates
        match knob.update() {
            Ok(_changed) => {
                let current_value = knob.value();
                let current_pressed = knob.pressed();

                if current_value != prev_value {
                    info!("Value: {}", current_value);
                    prev_value = current_value;
                }

                if current_pressed != prev_pressed {
                    if current_pressed {
                        info!("Button Pressed!");
                        // Optional: Reset on press
                        // knob.set_value(50).ok();
                    } else {
                        info!("Button Released");
                    }
                    prev_pressed = current_pressed;
                }
            }
            Err(e) => {
                error!("Failed to update knob: {:?}", Debug2Format(&e));
            }
        }

        // Poll interval
        timer.delay_ms(20);
    }
}
