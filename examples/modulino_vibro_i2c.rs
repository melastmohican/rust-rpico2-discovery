//! # Arduino Modulino Vibro Example
//!
//! Demonstrates various vibration patterns on the Arduino Modulino Vibro module over I2C.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** Arduino Modulino Vibro
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the Modulino Vibro.
//! The cable provides:
//! ```
//!      Modulino Vibro -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_vibro_i2c
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
use modulino::{PowerLevel, Vibro};

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

    info!("Initializing Arduino Modulino Vibro...");

    // Create Modulino Vibro driver
    // Automatically uses default address 0x24
    let mut vibro = match Vibro::new(i2c) {
        Ok(v) => v,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Vibro: {:?}",
                Debug2Format(&e)
            );
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Vibro initialized at address 0x{:02X}!",
        vibro.address()
    );

    loop {
        // 1. Gentle Pulses
        info!("Pattern 1: Gentle Pulses");
        for _ in 0..3 {
            vibro.pulse(100, PowerLevel::Gentle).ok();
            timer.delay_ms(500);
        }
        timer.delay_ms(1000);

        // 2. Medium Vibration
        info!("Pattern 2: Medium Vibration");
        vibro.on(1000, PowerLevel::Medium).ok();
        timer.delay_ms(2000);

        // 3. Intense Double Pulse
        info!("Pattern 3: Intense Double Pulse");
        vibro.pulse(200, PowerLevel::Intense).ok();
        timer.delay_ms(300);
        vibro.pulse(200, PowerLevel::Intense).ok();
        timer.delay_ms(2000);

        // 4. Power Sweep
        info!("Pattern 4: Power Sweep");
        let power_levels = [
            PowerLevel::Gentle,
            PowerLevel::Moderate,
            PowerLevel::Medium,
            PowerLevel::Intense,
            PowerLevel::Powerful,
            PowerLevel::Maximum,
        ];

        for &level in power_levels.iter() {
            info!("Power Level: {:?}", Debug2Format(&level));
            vibro.on(500, level).ok();
            timer.delay_ms(1000);
        }

        info!("Pattern complete. Waiting 3 seconds...");
        timer.delay_ms(3000);
    }
}
