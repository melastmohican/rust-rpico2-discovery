//! # Pico WS2812 RGB LED Example for RP2350
//!
//! The example assumes you connected the data input to pin 9 of the Raspberry Pi Pico, which is GPIO6 of the rp2350.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp235x_hal::{self as hal, Clock, block::ImageDef, entry, pio::PIOExt};

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{RGB8, SmartLedsWrite, brightness};
use ws2812_pio::Ws2812;

// Using Seeed Grove - RGB LED Stick (15-WS2813 Mini)
// https://www.seeedstudio.com/Grove-RGB-LED-Stick-15-WS2813-Mini-p-4270.html
const STRIP_LEN: usize = 15;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
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

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut frame_delay =
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sin = libm::sinf;

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut ws = Ws2812::new(
        pins.gpio15.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut t = 0.0;

    let strip_brightness = 64u8;
    let animation_speed = 0.1;

    loop {
        for (i, led) in leds.iter_mut().enumerate() {
            let hue_offs = match i % 3 {
                1 => 0.25,
                2 => 0.5,
                _ => 0.0,
            };

            let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
            let sin_01 = (sin_11 + 1.0) * 0.5;

            let hue = 360.0 * sin_01;
            let sat = 1.0;
            let val = 1.0;

            let rgb = hsv2rgb_u8(hue, sat, val);
            *led = rgb.into();
        }

        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        frame_delay.delay_ms(16);

        t += (16.0 / 1000.0) * animation_speed;
        while t > 1.0 {
            t -= 1.0;
        }
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}
