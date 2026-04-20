//! # Arduino Modulino Latch Relay Example
//!
//! Demonstrates how to control the Arduino Modulino Latch Relay module over I2C.
//! A latching relay maintains its state even when power is removed.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** Arduino Modulino Latch Relay
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Latch Relay.
//! The cable provides:
//! ```
//!      Modulino Latch Relay -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_latch_relay_i2c
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

// Import from modulino library
use modulino::LatchRelay;

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
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Latch Relay...");

    // Create Modulino Latch Relay driver
    // Automatically uses default address 0x02
    let mut relay = match LatchRelay::new(i2c) {
        Ok(r) => r,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Latch Relay: {:?}",
                Debug2Format(&e)
            );
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Latch Relay initialized at address 0x{:02X}!",
        relay.address()
    );

    // Wait a bit for the device to be ready
    timer.delay_ms(100);

    loop {
        // 1. Check current state
        match relay.is_on() {
            Ok(Some(true)) => info!("Relay is currently ON"),
            Ok(Some(false)) => info!("Relay is currently OFF"),
            Ok(None) => info!("Relay state is unknown (first power-up)"),
            Err(e) => error!("Failed to read relay state: {:?}", Debug2Format(&e)),
        }

        // 2. Turn ON
        info!("Turning relay ON...");
        if let Err(e) = relay.on() {
            error!("Failed to turn relay ON: {:?}", Debug2Format(&e));
        }
        timer.delay_ms(2000);

        // 3. Turn OFF
        info!("Turning relay OFF...");
        if let Err(e) = relay.off() {
            error!("Failed to turn relay OFF: {:?}", Debug2Format(&e));
        }
        timer.delay_ms(2000);

        // 4. Toggle
        info!("Toggling relay...");
        if let Err(e) = relay.toggle() {
            error!("Failed to toggle relay: {:?}", Debug2Format(&e));
        }
        timer.delay_ms(2000);

        // 5. Toggle again
        info!("Toggling relay back...");
        if let Err(e) = relay.toggle() {
            error!("Failed to toggle relay back: {:?}", Debug2Format(&e));
        }

        info!("Cycle complete. Waiting 5 seconds...");
        timer.delay_ms(5000);
    }
}
