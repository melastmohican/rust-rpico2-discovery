//! # BH1750 Light Sensor Example for Raspberry Pi Pico 2
//!
//! Reads ambient light levels in lux from a BH1750 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Sensor:** BH1750 Light Sensor Breakout
//! - **Connection:** I2C
//! - **I2C Address:** 0x23 (default) or 0x5C
//!
//! ## Wiring
//!
//! ```
//!      BH1750 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## I2C Address
//!
//! The BH1750 can have two I2C addresses, selected by the `ADDR` pin:
//! - `0x23` (ADDR pin to GND) - `address_pin_high: false`
//! - `0x5C` (ADDR pin to VCC) - `address_pin_high: true`
//!
//! This example uses `0x23` by default.
//!
//! Run with `cargo run --example bh1750_i2c`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

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

// The BH1750 driver
use bh1750::{BH1750, Resolution};

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

    // Create two timers. One for the sensor, one for the delay loop.
    let sensor_timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let mut delay = Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

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

    // Create a BH1750 driver instance
    // false = address 0x23 (ADDR pin to GND)
    // true = address 0x5C (ADDR pin to VCC)
    let mut bh1750 = BH1750::new(i2c, sensor_timer, false);

    info!("BH1750 initialized successfully!");
    info!("Starting light measurements...");

    loop {
        // Take a measurement in high resolution mode 2
        match bh1750.get_one_time_measurement(Resolution::High2) {
            Ok(lux) => {
                info!("Light level: {} lx", lux);
            }
            Err(e) => {
                error!("Error reading BH1750 sensor: {:?}", defmt::Debug2Format(&e));
            }
        }

        delay.delay_ms(1000);
    }
}
