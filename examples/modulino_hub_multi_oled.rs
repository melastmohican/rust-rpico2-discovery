//! # Modulino Hub Multi-OLED Example
//!
//! Demonstrates how to use multiple SSD1306 OLED displays connected to a Modulino Hub
//! (TCA9548A I2C multiplexer) with the Raspberry Pi Pico 2.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Hub:** Arduino Modulino Hub
//! - **OLEDs:** Two SSD1306 128x32 OLED displays (connected to Port 0 and Port 1 of the Hub)
//!
//! ## Wiring Diagram
//!
//! ```
//!      Raspberry Pi Pico 2              Modulino Hub
//!    +----------------------+      +----------------------+
//!    |                      |      |                      |
//!    |  3V3 (Pin 36) -------+------+-> VCC                |
//!    |  GND (Pin 38) -------+------+-> GND                |
//!    |  GPIO4 (Pin 6) ------+------+-> SDA                |
//!    |  GPIO5 (Pin 7) ------+------+-> SCL                |
//!    |                      |      |                      |
//!    +----------------------+      +----------+-----------+
//!                                             |
//!                                     +-------+-------+
//!                                     |               |
//!                                  Port 0          Port 1
//!                                     |               |
//!                                     v               v
//!                              +------------+  +------------+
//!                              | OLED Disp A|  | OLED Disp B|
//!                              +------------+  +------------+
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_hub_multi_oled
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use embedded_graphics::{
    mono_font::{
        MonoTextStyleBuilder,
        ascii::{FONT_6X10, FONT_9X15},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
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
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

use modulino::Hub;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// A custom shared I2C bus wrapper to implement both embedded-hal 1.0 and 0.2 I2C/Write traits.
struct SharedI2c<'a, T> {
    bus: &'a RefCell<T>,
}

impl<'a, T> SharedI2c<'a, T> {
    fn new(bus: &'a RefCell<T>) -> Self {
        Self { bus }
    }
}

// Implement embedded-hal 1.0 I2c for Hub
impl<'a, T> embedded_hal::i2c::ErrorType for SharedI2c<'a, T>
where
    T: embedded_hal::i2c::ErrorType,
{
    type Error = <T as embedded_hal::i2c::ErrorType>::Error;
}

impl<'a, T> embedded_hal::i2c::I2c for SharedI2c<'a, T>
where
    T: embedded_hal::i2c::I2c,
{
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write(address, write)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.bus.borrow_mut().transaction(address, operations)
    }
}

// Implement embedded-hal 0.2 Write for SSD1306
impl<'a, T> embedded_hal_0_2::blocking::i2c::Write for SharedI2c<'a, T>
where
    T: embedded_hal_0_2::blocking::i2c::Write,
{
    type Error = <T as embedded_hal_0_2::blocking::i2c::Write>::Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.bus.borrow_mut().write(addr, bytes)
    }
}

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

    defmt::info!("Initializing Modulino Hub and OLED displays...");

    // Share the I2C bus using a RefCell
    let i2c_bus = RefCell::new(i2c);

    // Initialize Hub using a SharedI2c instance
    let mut hub = Hub::new(SharedI2c::new(&i2c_bus));

    // Initialize OLED A on Port 0
    defmt::info!("Initializing OLED A (Port 0)...");
    hub.select(0).unwrap();

    let interface_a = I2CDisplayInterface::new(SharedI2c::new(&i2c_bus));
    let mut display_a = Ssd1306::new(interface_a, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display_a.init().unwrap();

    display_a.clear(BinaryColor::Off).unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    Text::with_baseline(
        "Screen A Ready",
        Point::new(0, 0),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display_a)
    .unwrap();
    display_a.flush().unwrap();
    hub.clear().unwrap();

    // Initialize OLED B on Port 1
    defmt::info!("Initializing OLED B (Port 1)...");
    hub.select(1).unwrap();

    let interface_b = I2CDisplayInterface::new(SharedI2c::new(&i2c_bus));
    let mut display_b = Ssd1306::new(interface_b, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display_b.init().unwrap();

    display_b.clear(BinaryColor::Off).unwrap();
    Text::with_baseline(
        "Screen B Ready",
        Point::new(0, 0),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display_b)
    .unwrap();
    display_b.flush().unwrap();
    hub.clear().unwrap();

    timer.delay_ms(1000);

    let mut counter = 0;
    let large_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(BinaryColor::On)
        .build();

    defmt::info!("Starting update loop...");
    loop {
        counter += 1;

        // Update Screen A
        hub.select(0).unwrap();
        display_a.clear(BinaryColor::Off).unwrap();
        Text::with_baseline("SCREEN A", Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display_a)
            .unwrap();

        let mut buf_a = [0u8; 32];
        let val_str_a = format_no_std::show(&mut buf_a, format_args!("Val: {}", counter)).unwrap();
        Text::with_baseline(
            val_str_a,
            Point::new(0, 16),
            large_text_style,
            Baseline::Top,
        )
        .draw(&mut display_a)
        .unwrap();
        display_a.flush().unwrap();
        hub.clear().unwrap();

        // Update Screen B
        hub.select(1).unwrap();
        display_b.clear(BinaryColor::Off).unwrap();
        Text::with_baseline("SCREEN B", Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display_b)
            .unwrap();

        let mut buf_b = [0u8; 32];
        let val_str_b =
            format_no_std::show(&mut buf_b, format_args!("Val: {}", counter * 2)).unwrap();
        Text::with_baseline(
            val_str_b,
            Point::new(0, 16),
            large_text_style,
            Baseline::Top,
        )
        .draw(&mut display_b)
        .unwrap();
        display_b.flush().unwrap();
        hub.clear().unwrap();

        timer.delay_ms(1000);
    }
}
