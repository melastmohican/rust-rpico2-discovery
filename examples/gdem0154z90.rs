//! # GDEM0154Z90 Tri-Color Image Example (rust.bmp)
//!
//! Displays `rust.bmp` on the Dalian Good Display 1.54" Tri-Color E-Ink Display.
//! This example demonstrates loading a BMP image and splitting its pixels into
//! Black/White and Red buffers for the e-ink display.
//!
//! ## Wiring (DESPI-C02 connection)
//!
//! Connection for DESPI-C02 Adapter Board (8-pin header):
//!
//! | DESPI-C02 Pin | Pico Pin       | Function     |
//! |---------------|----------------|--------------|
//! | **BUSY**      | GPIO13 (Pin 17)| Busy Signal  |
//! | **RES**       | GPIO11 (Pin 15)| Reset        |
//! | **D/C**       | GPIO12 (Pin 16)| Data/Command |
//! | **CS**        | GPIO17 (Pin 22)| Chip Select  |
//! | **SCK**       | GPIO18 (Pin 24)| SPI Clock    |
//! | **SDI**       | GPIO19 (Pin 25)| SPI MOSI     |
//! | **GND**       | GND (Pin 38)   | Ground       |
//! | **3.3V**      | 3V3 (Pin 36)   | Power        |
//!
//! Run with `cargo run --example gdem0154z90`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::{pixelcolor::Rgb888, prelude::*};
use tinybmp::Bmp;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use ssd1681::{
    HEIGHT, WIDTH,
    color::{Black, Red, White},
    driver::Ssd1681,
    graphics::{Display, Display1in54, DisplayRotation},
};

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    defmt::info!("Program start");
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

    // Create a delay using the system timer
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED pin for status indication
    let mut led_pin: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio25.into_push_pull_output();

    // Configure SPI pins for e-paper display
    let sck: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let sdi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs_spi = pins.gpio21.into_push_pull_output(); // Dummy CS for ExclusiveDevice

    // Control pins for e-paper
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio12.into_push_pull_output();
    let res = pins.gpio11.into_push_pull_output();
    let busy = pins.gpio13.into_pull_down_input();

    // Create SPI bus with 16 MHz clock speed
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (sdi, miso, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    defmt::info!("SPI configured at 16 MHz");

    let mut spi_device = ExclusiveDevice::new_no_delay(spi, cs_spi).unwrap();

    // Initialize display controller
    let mut ssd1681 = Ssd1681::new(&mut spi_device, cs, busy, dc, res, &mut timer).unwrap();

    defmt::info!("E-Paper display initialized");

    // Clear frames on the display driver
    ssd1681.clear_red_frame(&mut spi_device);
    ssd1681.clear_bw_frame(&mut spi_device);

    // Create buffers
    let mut display_bw = Display1in54::bw();
    let mut display_red = Display1in54::red();

    // Set rotation to match standard (usually 0 or 270 depending on usage, sticking to 0 default)
    display_bw.set_rotation(DisplayRotation::Rotate0);
    display_red.set_rotation(DisplayRotation::Rotate0);

    // Clear local buffers
    display_bw.clear(White).unwrap();
    display_red.clear(White).unwrap(); // Red buffer: White = Transparent/No Red

    // Load BMP
    // Note: rust.bmp must be compatible (e.g. 24-bit RGB)
    let bmp_data = include_bytes!("rust_tricolor.bmp");
    let bmp = Bmp::<Rgb888>::from_slice(bmp_data).unwrap();

    defmt::info!("Drawing image...");

    // Iterate over pixels and map to appropriate buffer
    // The display is likely 200x200 or similar check Ssd1681 dimensions
    // We position the image in the center if possible, or 0,0

    // Simple color mapping threshold
    // Red: High Red component, low Green/Blue
    // Black: Low RGB
    // White: High RGB

    for Pixel(point, color) in bmp.pixels() {
        // Skip if out of bounds
        if point.x < 0 || point.y < 0 || point.x >= WIDTH as i32 || point.y >= HEIGHT as i32 {
            continue;
        }

        if color.r() > 200 && color.g() < 100 && color.b() < 100 {
            // Looks like Red
            Pixel(point, Red).draw(&mut display_red).unwrap();
        } else if color.r() < 100 && color.g() < 100 && color.b() < 100 {
            // Looks like Black
            Pixel(point, Black).draw(&mut display_bw).unwrap();
        }
        // Else strictly White (background), do nothing as buffers are initialized to White
    }

    defmt::info!("Send bw frame to display");
    ssd1681.update_bw_frame(&mut spi_device, display_bw.buffer());

    defmt::info!("Send red frame to display");
    ssd1681.update_red_frame(&mut spi_device, display_red.buffer());

    defmt::info!("Update display");
    ssd1681.display_frame(&mut spi_device);

    defmt::info!("Done!");

    // Blink LED to indicate completion
    loop {
        let _ = led_pin.toggle();
        timer.delay_ms(500);
    }
}
