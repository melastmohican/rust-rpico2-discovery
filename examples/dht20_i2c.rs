//! # DHT20 Temperature/Humidity Sensor Example for Raspberry Pi Pico 2
//!
//! Reads temperature, and humidity from a DHT20 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Sensor:** Grove - Temperature & Humidity Sensor (DHT20)
//! - **Connection:** I2C
//! - **I2C Address:** 0x38
//!
//! ## Wiring
//!
//! ```
//!      DHT20 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! Run with `cargo run --example dht20_i2c`.

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

use dht20::{self, Dht20, Reading};

const DHT20_ADDR: u8 = 0x38;

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

    let mut dht20 = Dht20::new(i2c, DHT20_ADDR, timer);

    info!("DHT20 initialized successfully!");
    info!("Starting measurements...");

    loop {
        match dht20.read() {
            Ok(Reading { temp, hum }) => {
                info!("Temperature: {} C, Humidity: {} %", temp, hum);
            }
            Err(e) => {
                error!("Error reading DHT20 sensor: {:?}", defmt::Debug2Format(&e));
            }
        }

        // Wait 1 second between measurements
        timer.delay_ms(1000);
    }
}
