//! # Arduino Modulino Buzzer Example for Raspberry Pi Pico 2
//!
//! This example uses the **modulino** library: https://crates.io/crates/modulino
//!
//! Plays a simple melody on the Arduino Modulino Buzzer module over I2C.
//!
//! ## Hardware
//!
//! - **Module:** Arduino Modulino Buzzer
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x1E (7-bit)
//!
//! ## Wiring with Qwiic/STEMMA QT on Raspberry Pi Pico 2
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Buzzer.
//! The cable provides:
//! ```
//!      Modulino Buzzer -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! Run with `cargo run --example modulino_buzzer_i2c`.

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
use modulino::{Buzzer, Note};

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

    info!("Initializing Arduino Modulino Buzzer...");

    // Create Modulino Buzzer driver
    // Automatically uses default address 0x1E
    let mut buzzer = match Buzzer::new(i2c) {
        Ok(b) => b,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Buzzer: {:?}",
                Debug2Format(&e)
            );
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Buzzer initialized at address 0x{:02X}!",
        buzzer.address()
    );
    info!("Playing melody...");

    // Simple Melody: Super Mario Theme (Intro)
    let melody = [
        (Note::E5, 100),
        (Note::E5, 100),
        (Note::Rest, 100),
        (Note::E5, 100),
        (Note::Rest, 100),
        (Note::C5, 100),
        (Note::E5, 100),
        (Note::Rest, 100),
        (Note::G5, 100),
        (Note::Rest, 300),
        (Note::G4, 100),
        (Note::Rest, 300),
    ];

    loop {
        for (note, duration) in melody.iter() {
            if *note == Note::Rest {
                // For rest, ensure no tone is playing and wait
                buzzer.no_tone().ok();
            } else {
                // Play the note
                // Note: The buzzer module handles the duration internally for the tone generation,
                // but we also need to wait here so we don't immediately send the next command.
                // We add a small gap between notes for articulation.
                if let Err(e) = buzzer.play_note(*note, *duration) {
                    error!("Failed to play note: {:?}", Debug2Format(&e));
                }
            }

            // Wait for the note duration plus a little gap
            timer.delay_ms(*duration as u32);

            // Small gap between notes (articulation)
            timer.delay_ms(50);
        }

        // Wait before repeating
        timer.delay_ms(2000);
    }
}
