//! # BMP280 Temperature/Pressure Sensor Example for Raspberry Pi Pico 2
//!
//! Reads temperature and atmospheric pressure from a BMP280 sensor over I2C0.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit BMP280 Temperature Pressure Sensor (or compatible)
//! - **Connection:** I2C
//! - **I2C Address:** 0x77
//!
//! ## Wiring
//!
//! ```
//!      BMP280 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## I2C Address
//!
//! The BMP280 can have two I2C addresses, typically 0x76 or 0x77.
//! With the `bmp280-ehal` driver, you specify the address directly during instantiation, e.g.,
//! `BMP280::new_with_address(i2c, 0x77)`.
//!
//! - **0x76:** (SDO pin to GND) - Can be used by setting `BMP280::new_with_address(i2c, 0x76)`.
//! - **0x77:** (SDO pin to VCC) - Used by default in this example (`BMP280::new_with_address(i2c, 0x77)`).
//!
//! Run with `cargo run --example bmp280_i2c`.

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

use rp235x_hal::block::ImageDef;
use bmp280_ehal::{Config, Oversampling, PowerMode, Filter, Standby, Control, BMP280};
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;

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

    // Create a BMP280 driver instance
    let mut bmp280 = BMP280::new_with_address(i2c, 0x77).unwrap();

    // Configure the sensor
    let config = Config {
        t_sb: Standby::ms125,
        filter: Filter::off,
    };

    let control = Control {
        osrs_t: Oversampling::x16,
        osrs_p: Oversampling::x16,
        mode: PowerMode::Normal,
    };

    bmp280.set_config(config);
    bmp280.set_control(control);

    timer.delay_ms(100);

    info!("BMP280 initialized successfully!");
    info!("Starting measurements...");

    loop {
        let temperature = bmp280.temp();
        let pressure = bmp280.pressure();

        info!(
            "Temperature: {} C, Pressure: {} hPa",
            temperature,
            pressure / 100.0,
        );

        timer.delay_ms(1000);
    }
}
