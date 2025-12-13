//! # GDEM0154Z90 Tri-Color E-Ink Display Example for Raspberry Pi Pico 2 (RP2350)
//!
//! Simple "Hello World" example for the Raspberry Pi Pico 2 microcontroller board
//! with Dalian Good Display Tri-Color e-ink display 1.54 inch e-ink small display screen,
//! [GDEM0154Z90](https://www.good-display.com/product/436.html)
//! using DESPI-C02 adapter https://buyepaper.com/products/development-kit-connection-adapter-board-for-eaper-display-demo-kit
//!
//! This example is for the Raspberry Pi Pico 2 board using SPI0.
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
//! Run with `cargo run --example gdem0154z90_text`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::{
    Drawable,
    geometry::{Point, Size},
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X9},
    prelude::Primitive,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use rp235x_hal as hal;
use ssd1681::{
    WIDTH,
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
    let cs_spi = pins.gpio21.into_push_pull_output();

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

    // Create buffer for black and white
    let mut display_bw = Display1in54::bw();

    draw_rotation_and_rulers(&mut display_bw);

    display_bw.set_rotation(DisplayRotation::Rotate0);
    Rectangle::new(Point::new(60, 60), Size::new(100, 100))
        .into_styled(PrimitiveStyle::with_fill(Black))
        .draw(&mut display_bw)
        .unwrap();

    defmt::info!("Send bw frame to display");
    ssd1681.update_bw_frame(&mut spi_device, display_bw.buffer());

    // Draw red color
    let mut display_red = Display1in54::red();

    Circle::new(Point::new(100, 100), 20)
        .into_styled(PrimitiveStyle::with_fill(Red))
        .draw(&mut display_red)
        .unwrap();

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

fn draw_rotation_and_rulers(display: &mut Display1in54) {
    display.set_rotation(DisplayRotation::Rotate0);
    draw_text(display, "rotation 0", 25, 25);
    draw_ruler(display);

    display.set_rotation(DisplayRotation::Rotate90);
    draw_text(display, "rotation 90", 25, 25);
    draw_ruler(display);

    display.set_rotation(DisplayRotation::Rotate180);
    draw_text(display, "rotation 180", 25, 25);
    draw_ruler(display);

    display.set_rotation(DisplayRotation::Rotate270);
    draw_text(display, "rotation 270", 25, 25);
    draw_ruler(display);
}

fn draw_ruler(display: &mut Display1in54) {
    for col in 1..WIDTH {
        if col % 25 == 0 {
            Line::new(Point::new(col as i32, 0), Point::new(col as i32, 10))
                .into_styled(PrimitiveStyle::with_stroke(Black, 1))
                .draw(display)
                .unwrap();
        }

        if col % 50 == 0 {
            let mut buf = [0u8; 4];
            let label = format_no_std::show(&mut buf, format_args!("{}", col)).unwrap();
            draw_text(display, label, col as i32, 12);
        }
    }
}

fn draw_text(display: &mut Display1in54, text: &str, x: i32, y: i32) {
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_6X9)
        .text_color(Black)
        .background_color(White)
        .build();
    let _ = Text::new(text, Point::new(x, y), style).draw(display);
}
