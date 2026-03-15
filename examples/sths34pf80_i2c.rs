//! # STHS34PF80 IR Presence / Motion Sensor Example for Raspberry Pi Pico 2
//!
//! Reads presence, motion, and temperature data from an STHS34PF80 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 (RP2350)
//! - **Sensor:** Adafruit STHS34PF80 IR Presence / Motion Sensor
//! - **Connection:** I2C
//! - **I2C Address:** 0x5A (default for STHS34PF80)
//!
//! ## Wiring
//!
//! ``` text
//!      STHS34PF80 -> RPi Pico 2
//! (black)  GND    -> GND
//! (red)    VCC    -> 3.3V
//! (yellow) SCL    -> GPIO5 (Pin 7)
//! (blue)   SDA    -> GPIO4 (Pin 6)
//! ```
//!
//! Run with `cargo run --example sths34pf80_i2c`.
//!
//! ## Understanding the Data
//!
//! - **Presence & Motion:** Internal algorithm outputs. Large values (e.g., > 1000) indicate
//!   significant detection. Negative values indicate a decrease in the detected signal
//!   (e.g., something moving away).
//! - **Raw Obj IR:** The raw infrared radiant power intensity. This is the value that
//!   the internal algorithms use to determine presence and motion.
//!
//! ## Expected Output
//!
//! ```text
//! [INFO ] Presence: 152 | Motion: -24 | Raw Obj IR: 2989 (Intensity)
//! [INFO ] Presence: 562 | Motion: 392 | Raw Obj IR: 2568 (Intensity)
//! [INFO ] Presence: 572 | Motion: 390 | Raw Obj IR: 2459 (Intensity)
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

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use hal::block::ImageDef;
use sths34pf80::Sths34pf80;

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
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing STHS34PF80 sensor...");

    // Initialize STHS34PF80 sensor
    // We pass &mut timer so we don't lose ownership of the timer peripheral.
    let mut sensor = Sths34pf80::new(i2c, &mut timer);

    // Initialize the sensor with default configuration
    if let Err(e) = sensor.initialize() {
        error!(
            "Failed to initialize STHS34PF80: {:?}",
            defmt::Debug2Format(&e)
        );
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("STHS34PF80 initialized successfully!");
    info!("Starting measurements...");

    let mut i2c = sensor.release();
    loop {
        let presence;
        let motion;
        let obj_raw;

        {
            // We create a temporary sensor instance to avoid holding a long-term borrow on timer.
            // Sths34pf80::new is just a struct constructor and doesn't perform I2C traffic.
            let mut sensor = Sths34pf80::new(i2c, &mut timer);
            presence = sensor.get_presence().unwrap_or(0);
            motion = sensor.get_tmotion().unwrap_or(0);
            obj_raw = sensor.get_temperature().unwrap_or(0);
            i2c = sensor.release();
        }

        info!(
            "Presence: {} | Motion: {} | Raw Obj IR: {} (Intensity)",
            presence, motion, obj_raw,
        );

        // Wait 500ms between measurements
        timer.delay_ms(500);
    }
}
