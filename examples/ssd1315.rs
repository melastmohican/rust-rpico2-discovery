//! Draw a 1 bit per pixel black and white image. On a 128x64 SSD1306 display over I2C.
//!
//! This example is for the Raspberry Pi Pico board using I2C0.
//!
//! Wiring connections are as follows for display:
//!
//! ```
//!      Raspberry Pi Pico 2           SSD1315 128x64 OLED
//!    +-----------------------+      +---------------------------+
//!    |                       |      |                           |
//!    |  3V3 (Pin 36) --------+------+-> VCC                     |
//!    |  GND (Pin 38) --------+------+-> GND                     |
//!    |  GPIO4 (Pin 6) -------+------+-> SDA                     |
//!    |  GPIO5 (Pin 7) -------+------+-> SCL                     |
//!    |                       |      |                           |
//!    +-----------------------+      +---------------------------+
//! ```
//!
//! Run on a Rpi Pico with `cargo run --example ssd1306`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm::nop;
use embedded_graphics::{image::Image, pixelcolor::BinaryColor, prelude::*};
use hal::I2C;
use hal::Sio;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use hal::block::ImageDef;
use ssd1315::{Ssd1315, config, interface};

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

    let interface = interface::I2cDisplayInterface::new_interface(i2c);
    let config = config::Ssd1315DisplayConfig::preset_config();

    let mut display = Ssd1315::new(interface);
    display.set_custom_config(config);
    display.init_screen();

    let logo = tinybmp::Bmp::<BinaryColor>::from_slice(include_bytes!("rustbw.bmp")).unwrap();

    let im = Image::new(&logo, Point::new(32, 0));
    im.draw(&mut display).unwrap();

    display.flush_screen();

    loop {
        nop()
    }
}
