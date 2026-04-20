//! # Chrome Dino Game
//!
//! A port of the Chrome Dino Game for the Raspberry Pi Pico 2.
//! Jump over cacti to survive!
//!
//! Controls:
//! - **Push Button:** GPIO15 to GND (Jump)
//!
//! ![Dino Game Demo](dino_game_demo.gif)
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Display:** Waveshare 0.96" ST7735S LCD (80x160)
//! - **Button:** Adafruit STEMMA Wired Tactile Push-Button
//!
//! ## Wiring for Waveshare 0.96" LCD Module
//!
//! ```
//!      Raspberry Pi Pico 2          Waveshare 0.96" ST7735S LCD
//!    +-----------------------+      +---------------------------+
//!    |                       |      |                           |
//!    |  3V3 (Pin 36) --------+------+-> VCC                     |
//!    |  GND (Pin 38) --------+------+-> GND                     |
//!    |  GPIO17 (Pin 22) -----+------+-> CS                      |
//!    |  GPIO21 (Pin 27) -----+------+-> RST                     |
//!    |  GPIO20 (Pin 26) -----+------+-> DC                      |
//!    |  GPIO19 (Pin 25) -----+------+-> DIN(MOSI)               |
//!    |  GPIO18 (Pin 24) -----+------+-> CLK(SCK)                |
//!    |  GPIO14 (Pin 19) -----+------+-> BL (optional)           |
//!    |                       |      |                           |
//!    +-----------------------+      +---------------------------+
//! ```
//!
//! ## Wiring for External Button (Adafruit #4431)
//!
//! ```
//!      Raspberry Pi Pico 2          Adafruit STEMMA Button
//!    +-----------------------+      +---------------------------+
//!    |                       |      |                           |
//!    |  GPIO15 (Pin 20) -----+------+-> Signal (White)          |
//!    |  3V3 (Pin 36) --------+------+-> VCC (Red)               |
//!    |  GND (Pin 38) --------+------+-> GND (Black)             |
//!    |                       |      |                           |
//!    +-----------------------+      +---------------------------+
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example dino_game
//! ```

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSio, FunctionSpi, Pin, SioInput, SioOutput};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use mipidsi::{
    Builder,
    models::ST7735s,
    options::{ColorInversion, ColorOrder, Orientation, Rotation},
};
use rp235x_hal as hal;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// --- Game Constants (Scaled Logic) ---
const DINO_WIDTH: i32 = 16;
const DINO_HEIGHT: i32 = 16;
const CACTUS_WIDTH: i32 = 8;
const CACTUS_HEIGHT: i32 = 12;

const SCREEN_WIDTH: i32 = 160;
const GROUND_Y: i32 = 65;
const SCALE: i32 = 100;
const GRAVITY: i32 = 6; // 0.06 px/frame^2
const JUMP_FORCE: i32 = -240; // -2.4 px/frame
const INITIAL_SPEED: i32 = 60; // 0.6 px/frame

// --- Sprites (Bitmasks) ---
const DINO_IDLE: [u16; 16] = [
    0b0000011111000000,
    0b0000010111100000,
    0b0000011111110000,
    0b0000011111111000,
    0b0000011111100000,
    0b0000011111111100,
    0b0001111111100000,
    0b0111111111111100,
    0b1111111111111000,
    0b1111111111110000,
    0b1111111111000000,
    0b0111111110000000,
    0b0001111101000000,
    0b0000110110000000,
    0b0000110110000000,
    0b0000110111000000,
];

const CACTUS_SPRITE: [u8; 12] = [
    0b00011000, 0b00011000, 0b01011010, 0b01011010, 0b01111110, 0b01111110, 0b00011000, 0b00011000,
    0b00011000, 0b00011000, 0b00011000, 0b00011000,
];

pub struct GameState {
    dino_y: i32,
    prev_dino_y: i32,
    dino_vel_y: i32,
    is_jumping: bool,
    score: u32,
    cactus_x: i32,
    prev_cactus_x: i32,
    speed: i32,
    game_over: bool,
    frame_count: u32,
}

impl GameState {
    fn new() -> Self {
        let start_y = (GROUND_Y - DINO_HEIGHT) * SCALE;
        Self {
            dino_y: start_y,
            prev_dino_y: start_y,
            dino_vel_y: 0,
            is_jumping: false,
            score: 0,
            cactus_x: SCREEN_WIDTH * SCALE,
            prev_cactus_x: SCREEN_WIDTH * SCALE,
            speed: INITIAL_SPEED,
            game_over: false,
            frame_count: 0,
        }
    }

    fn update(&mut self, jump_pressed: bool) {
        if self.game_over {
            return;
        }

        // Jump processing
        if jump_pressed && !self.is_jumping {
            self.is_jumping = true;
            self.dino_vel_y = JUMP_FORCE;
        }

        self.prev_dino_y = self.dino_y;
        if self.is_jumping {
            self.dino_vel_y += GRAVITY;
            self.dino_y += self.dino_vel_y;

            if self.dino_y >= (GROUND_Y - DINO_HEIGHT) * SCALE {
                self.dino_y = (GROUND_Y - DINO_HEIGHT) * SCALE;
                self.dino_vel_y = 0;
                self.is_jumping = false;
            }
        }

        self.prev_cactus_x = self.cactus_x;
        self.cactus_x -= self.speed;
        if self.cactus_x < (-CACTUS_WIDTH) * SCALE {
            self.cactus_x = (SCREEN_WIDTH + (self.frame_count % 50) as i32) * SCALE;
            self.score += 10;
            if self.score.is_multiple_of(100) && self.speed < 800 {
                self.speed += 20;
            }
        }

        // Collision Check
        let d_x = 20;
        let d_y = self.dino_y / SCALE;
        let c_x = self.cactus_x / SCALE;
        let c_y = GROUND_Y - CACTUS_HEIGHT;

        if d_x < c_x + CACTUS_WIDTH
            && d_x + DINO_WIDTH > c_x
            && d_y < c_y + CACTUS_HEIGHT
            && d_y + DINO_HEIGHT > c_y
        {
            self.game_over = true;
        }

        self.frame_count += 1;
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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Initializing Display Pins
    let sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio20.into_push_pull_output();
    let mut rst = pins.gpio21.into_push_pull_output();
    let mut bl: Pin<_, FunctionSio<SioOutput>, _> = pins.gpio14.into_push_pull_output();

    // Button Pin
    let mut button: Pin<_, FunctionSio<SioInput>, _> = pins.gpio15.into_pull_up_input();

    // SPI Configuration
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        16_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let di = SPIInterface::new(spi_device, dc);

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Reset Display
    let _ = rst.set_low();
    timer.delay_ms(10);
    let _ = rst.set_high();
    timer.delay_ms(120);

    let mut display = Builder::new(ST7735s, di)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .display_size(80, 160)
        .display_offset(26, 1)
        .init(&mut timer)
        .unwrap();

    let _ = bl.set_high();
    display.clear(Rgb565::WHITE).unwrap();

    info!("Dino Game Started!");

    let mut game = GameState::new();
    let mut last_score = 0;
    let mut score_buf = [0u8; 12];

    loop {
        if game.game_over {
            let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);
            Text::new("GAME OVER", Point::new(50, 40), text_style)
                .draw(&mut display)
                .unwrap();

            // Wait for button press to restart
            while button.is_high().unwrap() {}
            timer.delay_ms(200); // debounce
            game = GameState::new();
            display.clear(Rgb565::WHITE).unwrap();
            last_score = 0;
            continue;
        }

        // Update Game
        let jump_pressed = button.is_low().unwrap();
        game.update(jump_pressed);

        // Render Game
        let draw_dino_y = game.dino_y / SCALE;
        let draw_prev_dino_y = game.prev_dino_y / SCALE;
        let draw_cactus_x = game.cactus_x / SCALE;
        let draw_prev_cactus_x = game.prev_cactus_x / SCALE;

        // Erase old positions
        if draw_dino_y != draw_prev_dino_y {
            Rectangle::new(
                Point::new(20, draw_prev_dino_y),
                Size::new(DINO_WIDTH as u32, DINO_HEIGHT as u32),
            )
            .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
            .draw(&mut display)
            .unwrap();
        }
        if draw_cactus_x != draw_prev_cactus_x {
            Rectangle::new(
                Point::new(draw_prev_cactus_x, GROUND_Y - CACTUS_HEIGHT),
                Size::new(CACTUS_WIDTH as u32, CACTUS_HEIGHT as u32),
            )
            .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
            .draw(&mut display)
            .unwrap();
        }

        // Draw Ground
        Line::new(Point::new(0, GROUND_Y), Point::new(160, GROUND_Y))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1))
            .draw(&mut display)
            .unwrap();

        // Draw Dino
        draw_bitmask_16(
            &mut display,
            20,
            draw_dino_y as i16,
            &DINO_IDLE,
            Rgb565::BLACK,
        );

        // Draw Cactus
        draw_bitmask_8(
            &mut display,
            draw_cactus_x as i16,
            (GROUND_Y - CACTUS_HEIGHT) as i16,
            &CACTUS_SPRITE,
            Rgb565::GREEN,
        );

        // Update Score
        if game.score != last_score || game.score == 0 {
            let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::BLACK);
            let score_str = u32_to_str(game.score, &mut score_buf);
            // Clear old score area
            Rectangle::new(Point::new(120, 0), Size::new(40, 12))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                .draw(&mut display)
                .unwrap();
            Text::new(score_str, Point::new(120, 10), text_style)
                .draw(&mut display)
                .unwrap();
            last_score = game.score;
        }

        // Delay for ~60 FPS (16.6 ms)
        timer.delay_ms(16);
    }
}

fn draw_bitmask_16<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    x: i16,
    y: i16,
    mask: &[u16; 16],
    color: Rgb565,
) where
    D::Error: core::fmt::Debug,
{
    let iterator = (0..256).map(|i| {
        let row = i / 16;
        let col = i % 16;
        if (mask[row] >> (15 - col)) & 1 == 1 {
            color
        } else {
            Rgb565::WHITE
        }
    });

    let _ = display.fill_contiguous(
        &Rectangle::new(Point::new(x as i32, y as i32), Size::new(16, 16)),
        iterator,
    );
}

fn draw_bitmask_8<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    x: i16,
    y: i16,
    mask: &[u8; 12],
    color: Rgb565,
) where
    D::Error: core::fmt::Debug,
{
    let iterator = (0..96).map(|i| {
        let row = i / 8;
        let col = i % 8;
        if (mask[row] >> (7 - col)) & 1 == 1 {
            color
        } else {
            Rgb565::WHITE
        }
    });

    let _ = display.fill_contiguous(
        &Rectangle::new(Point::new(x as i32, y as i32), Size::new(8, 12)),
        iterator,
    );
}

fn u32_to_str(mut n: u32, buf: &mut [u8]) -> &str {
    if n == 0 {
        return "0";
    }
    let mut i = buf.len();
    while n > 0 && i > 0 {
        i -= 1;
        buf[i] = (n % 10) as u8 + b'0';
        n /= 10;
    }
    core::str::from_utf8(&buf[i..]).unwrap_or("")
}
