//! # HS3003 Debug Version - Detailed Logging
//!
//! This is a debug version with verbose logging to diagnose the communication issue,
//! adapted for the Raspberry Pi Pico 2.

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

use cortex_m::prelude::{
    _embedded_hal_blocking_delay_DelayMs, _embedded_hal_blocking_i2c_Read,
    _embedded_hal_blocking_i2c_Write,
};
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

const HS3003_ADDR: u8 = 0x44;

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

    let mut i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    
    info!("HS3003 Debug Mode");
    info!("=================");

    info!(
        "I2C initialized. Testing communication with HS3003 at 0x{:02X}",
        HS3003_ADDR
    );
    timer.delay_ms(100);

    loop {
        info!("");
        info!("--- New Measurement Cycle ---");

        // Test 1: Try writing (wake/trigger)
        info!("Test 1: Sending wake/measurement trigger (empty write)");
        match i2c.write(HS3003_ADDR, &[0x00]) {
            Ok(_) => info!("  Write OK"),
            Err(e) => info!("  Write FAILED: {:?}", defmt::Debug2Format(&e)),
        }

        timer.delay_ms(100);

        // Test 2: Try reading immediately
        info!("Test 2: Reading 4 bytes after 100ms");
        let mut data = [0u8; 4];
        match i2c.read(HS3003_ADDR, &mut data) {
            Ok(_) => {
                info!(
                    "  Read OK: [{:02X} {:02X} {:02X} {:02X}]",
                    data[0], data[1], data[2], data[3]
                );
                let status = data[0] >> 6;
                info!(
                    "  Status bits (top 2 bits of first byte): {:02b}",
                    status
                );

                // Try to parse
                let humidity_raw = (((data[0] as u16) & 0x3F) << 8) | (data[1] as u16);
                let humidity = (humidity_raw as f32 / 16383.0) * 100.0;

                let temp_raw = ((data[2] as u16) << 8) | (data[3] as u16);
                let temp_value = (temp_raw >> 2) & 0x3FFF;
                let temperature = ((temp_value as f32 / 16383.0) * 165.0) - 40.0;

                info!(
                    "  Parsed: Temp={}.{:02}°C, Hum={}.{:02}%",
                    temperature as i32,
                    ((temperature.abs() % 1.0) * 100.0) as u32,
                    humidity as u32,
                    ((humidity % 1.0) * 100.0) as u32
                );
            }
            Err(e) => info!("  Read FAILED: {:?}", defmt::Debug2Format(&e)),
        }

        info!("Waiting 2 seconds before next cycle...");
        timer.delay_ms(2000);
    }
}