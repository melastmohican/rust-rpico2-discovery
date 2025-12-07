//! Scans both I2C buses for connected devices and prints their addresses.
//!
//! This example is for the Raspberry Pi Pico 2 board. It initializes both I2C0 and I2C1
//! peripherals and probes all possible 7-bit addresses on each bus.
//!
//! ## Wiring
//!
//! You can connect I2C devices to either or both of the default I2C ports.
//!
//! ### I2C0
//! - **SDA:** GPIO4 (Pin 6)
//! - **SCL:** GPIO5 (Pin 7)
//!
//! ### I2C1
//! - **SDA:** GPIO2 (Pin 4)
//! - **SCL:** GPIO3 (Pin 5)
//!
//! Run with `cargo run --example i2c_scan`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use embedded_hal::i2c::I2c;
use hal::I2C;
use hal::Sio;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // --- Scan I2C1 ---
    info!("Scanning I2C1 on GPIO2 (SDA) and GPIO3 (SCL)...");
    let sda_pin_1: Pin<_, FunctionI2C, _> = pins.gpio2.reconfigure();
    let scl_pin_1: Pin<_, FunctionI2C, _> = pins.gpio3.reconfigure();

    let mut i2c1 = I2C::i2c1(
        pac.I2C1,
        sda_pin_1,
        scl_pin_1,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    for i in 0..=127u8 {
        let mut readbuf: [u8; 1] = [0; 1];
        match i2c1.read(i, &mut readbuf) {
            Ok(_) => {
                info!("I2C1: Device found at address 0x{=u8:X}", i);
            }
            Err(_) => {
                // Ignore errors
            }
        }
    }
    info!("Finished scanning I2C1.");

    // --- Scan I2C0 ---
    info!("Scanning I2C0 on GPIO4 (SDA) and GPIO5 (SCL)...");
    let sda_pin_0: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin_0: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    let mut i2c0 = I2C::i2c0(
        pac.I2C0,
        sda_pin_0,
        scl_pin_0,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    for i in 0..=127u8 {
        let mut readbuf: [u8; 1] = [0; 1];
        match i2c0.read(i, &mut readbuf) {
            Ok(_) => {
                info!("I2C0: Device found at address 0x{=u8:X}", i);
            }
            Err(_) => {
                // Ignore errors
            }
        }
    }
    info!("Finished scanning I2C0.");

    info!("Finished all scans.");
    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
