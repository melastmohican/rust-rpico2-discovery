//! # Arduino Modulino Buttons Example
//!
//! Reads button states from the Arduino Modulino Buttons module and controls the LEDs.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** Arduino Modulino Buttons
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Buttons.
//! The cable provides:
//! ```
//!      Modulino Buttons -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_buttons_i2c
//! ```

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

// Import the Modulino library
use modulino::Buttons;

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

    info!("Initializing Arduino Modulino Buttons...");

    // Create Modulino Buttons driver
    let mut buttons = match Buttons::new(i2c) {
        Ok(b) => b,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Buttons: {:?}",
                Debug2Format(&e)
            );
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Buttons initialized at address 0x{:02X}",
        buttons.address()
    );

    // Startup Test: Blink all LEDs to confirm connection
    info!("Testing LEDs...");
    if let Err(e) = buttons.all_leds_on() {
        error!("Failed to turn on LEDs: {:?}", Debug2Format(&e));
    }
    timer.delay_ms(500);
    if let Err(e) = buttons.all_leds_off() {
        error!("Failed to turn off LEDs: {:?}", Debug2Format(&e));
    }
    info!("LED Test Complete.");

    info!("Press buttons to toggle LEDs!");

    let mut prev_state = modulino::ButtonState::default();

    loop {
        // Read button states
        match buttons.read() {
            Ok(state) => {
                let mut need_update = false;

                // Button A: Rising Edge Detection
                if state.a && !prev_state.a {
                    info!("Button A pressed - Toggling LED A");
                    buttons.led_a.toggle();
                    need_update = true;
                }

                // Button B: Rising Edge Detection
                if state.b && !prev_state.b {
                    info!("Button B pressed - Toggling LED B");
                    buttons.led_b.toggle();
                    need_update = true;
                }

                // Button C: Rising Edge Detection
                if state.c && !prev_state.c {
                    info!("Button C pressed - Toggling LED C");
                    buttons.led_c.toggle();
                    need_update = true;
                }

                // Update LEDs only if state changed
                if need_update {
                    let update_res = buttons.update_leds();
                    if let Err(e) = update_res {
                        error!("Failed to update LEDs: {:?}", Debug2Format(&e));
                    }
                }

                // Save state for next iteration
                prev_state = state;
            }
            Err(e) => {
                error!("Failed to read buttons: {:?}", Debug2Format(&e));
            }
        }

        // Poll every 20ms for better responsiveness
        timer.delay_ms(20);
    }
}
