//! MAX7219 8x8 LED Matrix Drawing Animations
//!
//! This example demonstrates drawing primitives and some animations (Bouncing Ball,
//! Pong Game, Heartbeat, Blinking Smile) on a MAX7219 8x8 LED Matrix.
//!
//! Wiring Diagram (Raspberry Pi Pico 2 -> MAX7219 Matrix):
//! ------------------------------------------------
//! 5V (VBUS) or 3V3 -> VCC   (Pin 40 or Pin 36)
//! GND              -> GND   (Pin 38 or any GND)
//! GPIO3 (SPI0 TX)  -> DIN   (Pin 5)
//! GPIO5 (SPI0 CSn) -> CS    (Pin 7)
//! GPIO2 (SPI0 SCK) -> CLK   (Pin 4)
//!
//! Run with `cargo run --example max7219_8x8_matrix --release`.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use hal::{
    Spi,
    block::ImageDef,
    gpio::{FunctionSpi, Pin},
};
use max7219::DecodeMode;
use max7219::MAX7219;
use panic_probe as _;
use rp235x_hal as hal;
use rp235x_hal::Clock;
use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::timer::TimerDevice;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// --- Pseudo-random number generator for Pong Game ---
struct Lcg {
    state: u32,
}
impl Lcg {
    fn new(seed: u32) -> Self {
        Self { state: seed }
    }
    fn next(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
        self.state
    }
    fn random_range(&mut self, min: i32, max: i32) -> i32 {
        let range = max - min + 1;
        let val = (self.next() >> 16) as i32;
        min + val.rem_euclid(range)
    }
}

// --------------------------------------
// Drawing Primitives and Buffer
// --------------------------------------
struct Drawing {
    buffer: [u8; 8],
}

impl Drawing {
    fn new() -> Self {
        Self { buffer: [0; 8] }
    }

    fn clear_screen(&mut self, on: bool) {
        self.buffer = if on { [255; 8] } else { [0; 8] };
    }

    fn set_pixel(&mut self, r: u8, c: u8) {
        if r < 8 && c < 8 {
            self.buffer[r as usize] |= 1 << (7 - c);
        }
    }

    fn unset_pixel(&mut self, r: u8, c: u8) {
        if r < 8 && c < 8 {
            self.buffer[r as usize] &= !(1 << (7 - c));
        }
    }

    fn draw_pixel(&mut self, r: u8, c: u8, on: bool) {
        if on {
            self.set_pixel(r, c);
        } else {
            self.unset_pixel(r, c);
        }
    }

    fn draw_line(&mut self, r1: i8, c1: i8, r2: i8, c2: i8, on: bool) {
        let dr = (r2 - r1).abs() as f32;
        let dc = (c2 - c1).abs() as f32;
        let steps = if dr >= dc { dr } else { dc };

        if steps == 0.0 {
            self.draw_pixel(r1 as u8, c1 as u8, on);
            return;
        }

        let rstep = (r2 - r1) as f32 / steps;
        let cstep = (c2 - c1) as f32 / steps;

        let mut r = r1 as f32;
        let mut c = c1 as f32;

        for _ in 0..=(steps as i32) {
            self.draw_pixel(libm::roundf(r) as u8, libm::roundf(c) as u8, on);
            r += rstep;
            c += cstep;
        }
    }

    fn draw_rectangle(&mut self, r1: u8, c1: u8, r2: u8, c2: u8, on: bool) {
        self.draw_line(r1 as i8, c1 as i8, r1 as i8, c2 as i8, on);
        self.draw_line(r2 as i8, c1 as i8, r2 as i8, c2 as i8, on);
        self.draw_line(r1 as i8, c1 as i8, r2 as i8, c1 as i8, on);
        self.draw_line(r1 as i8, c2 as i8, r2 as i8, c2 as i8, on);
    }

    fn draw_rectangle_filled(&mut self, r1: u8, c1: u8, r2: u8, c2: u8, on: bool) {
        for r in r1..=r2 {
            for c in c1..=c2 {
                self.draw_pixel(r, c, on);
            }
        }
    }

    fn draw_row(&mut self, r: u8, on: bool) {
        if r < 8 {
            self.buffer[r as usize] = if on { 255 } else { 0 };
        }
    }

    fn draw_column(&mut self, c: u8, on: bool) {
        for r in 0..8 {
            self.draw_pixel(r, c, on);
        }
    }

    fn shift_down(&mut self, on: bool) {
        for r in (1..8).rev() {
            self.buffer[r] = self.buffer[r - 1];
        }
        self.draw_row(0, on);
    }

    fn shift_up(&mut self, on: bool) {
        for r in 0..7 {
            self.buffer[r] = self.buffer[r + 1];
        }
        self.draw_row(7, on);
    }

    fn shift_left(&mut self, on: bool) {
        for r in 0..8 {
            self.buffer[r] <<= 1;
        }
        if on {
            self.draw_column(7, true);
        }
    }

    fn shift_right(&mut self, on: bool) {
        for r in 0..8 {
            self.buffer[r] >>= 1;
        }
        if on {
            self.draw_column(0, true);
        }
    }

    // Call this to update the display
    fn update_display<C>(&self, display: &mut MAX7219<C>) -> Result<(), max7219::DataError>
    where
        C: max7219::connectors::Connector,
    {
        display.write_raw(0, &self.buffer)
    }
}

// --------------------------------------
// Animations
// --------------------------------------

/// Blinking Smile: A face that blinks its eyes at regular intervals.
fn run_blinking_smile<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: BlinkingSmile");
    let mut draw = Drawing::new();
    let frames = [
        // Eyes Open
        [
            0b00000000, 0b01100110, 0b01100110, 0b00000000, 0b10000001, 0b01000010, 0b00111100,
            0b00000000,
        ],
        // Eyes Closed (Blink)
        [
            0b00000000, 0b00000000, 0b01100110, 0b00000000, 0b10000001, 0b01000010, 0b00111100,
            0b00000000,
        ],
    ];

    let mut current_frame = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = frames[current_frame];
        draw.update_display(display)?;

        current_frame = (current_frame + 1) % 2;

        let interval = if current_frame == 1 { 150 } else { 1200 };
        timer.delay_ms(interval);
    }
    Ok(())
}

/// Bouncing Ball: A single pixel bouncing off the walls with randomized initial velocity and subtle bounce noise.
fn run_bouncing_ball<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: BouncingBall");
    let mut draw = Drawing::new();
    let mut r: f32 = 3.5;
    let mut c: f32 = 3.5;

    // Random direction and speed
    let mut dr = (rng.random_range(5, 12) as f32) / 10.0;
    if rng.random_range(0, 1) == 0 {
        dr *= -1.0;
    }
    let mut dc = (rng.random_range(5, 12) as f32) / 10.0;
    if rng.random_range(0, 1) == 0 {
        dc *= -1.0;
    }

    let interval = 80;

    let start_time = timer.get_counter();
    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.clear_screen(false);

        let current_r = libm::roundf(r) as u8;
        let current_c = libm::roundf(c) as u8;

        draw.draw_pixel(current_r.clamp(0, 7), current_c.clamp(0, 7), true);

        draw.update_display(display)?;

        r += dr;
        c += dc;

        if r <= 0.0 || r >= 7.0 {
            dr *= -1.0;
            dr += (rng.random_range(-2, 2) as f32) / 10.0;
            if libm::fabsf(dr) < 0.5 {
                dr = if dr > 0.0 { 0.5 } else { -0.5 };
            }
            if libm::fabsf(dr) > 1.2 {
                dr = if dr > 0.0 { 1.2 } else { -1.2 };
            }
        }
        if c <= 0.0 || c >= 7.0 {
            dc *= -1.0;
            dc += (rng.random_range(-2, 2) as f32) / 10.0;
            if libm::fabsf(dc) < 0.5 {
                dc = if dc > 0.0 { 0.5 } else { -0.5 };
            }
            if libm::fabsf(dc) > 1.2 {
                dc = if dc > 0.0 { 1.2 } else { -1.2 };
            }
        }

        timer.delay_ms(interval);
    }
    Ok(())
}

/// Countdown: Counts down from 9 to 0 using custom 8x8 digit patterns.
fn run_countdown<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: Countdown");
    let mut draw = Drawing::new();
    let digits = [
        [
            0b00111100, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110, 0b01100110,
            0b00111100,
        ], // 0
        [
            0b00011000, 0b00111000, 0b01011000, 0b00011000, 0b00011000, 0b00011000, 0b00011000,
            0b00111100,
        ], // 1
        [
            0b00111100, 0b01100110, 0b00000110, 0b00001100, 0b00011000, 0b00110000, 0b01100000,
            0b01111110,
        ], // 2
        [
            0b00111100, 0b01100110, 0b00000110, 0b00011100, 0b00000110, 0b00000110, 0b01100110,
            0b00111100,
        ], // 3
        [
            0b00001100, 0b00011100, 0b00101100, 0b01001100, 0b01111110, 0b00001100, 0b00001100,
            0b00001100,
        ], // 4
        [
            0b01111110, 0b01100000, 0b01111100, 0b00000110, 0b00000110, 0b00000110, 0b01100110,
            0b00111100,
        ], // 5
        [
            0b00111100, 0b01100110, 0b01100000, 0b01111100, 0b01100110, 0b01100110, 0b01100110,
            0b00111100,
        ], // 6
        [
            0b01111110, 0b01100110, 0b00000110, 0b00001100, 0b00011000, 0b00110000, 0b01100000,
            0b01100000,
        ], // 7
        [
            0b00111100, 0b01100110, 0b01100110, 0b00111100, 0b01100110, 0b01100110, 0b01100110,
            0b00111100,
        ], // 8
        [
            0b00111100, 0b01100110, 0b01100110, 0b01100110, 0b00111110, 0b00000110, 0b01100110,
            0b00111100,
        ], // 9
    ];

    let mut current = 9;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = digits[current as usize];
        draw.update_display(display)?;

        if current > 0 {
            current -= 1;
        } else {
            current = 9;
        }
        timer.delay_ms(500);
    }
    Ok(())
}

/// Filled Rectangles: Draws randomly positioned and sized filled rectangles.
fn run_filled_rectangles<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: FilledRectangles");
    let mut draw = Drawing::new();
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        let r1 = rng.random_range(0, 7) as u8;
        let c1 = rng.random_range(0, 7) as u8;
        let r2 = rng.random_range(0, 7) as u8;
        let c2 = rng.random_range(0, 7) as u8;

        draw.clear_screen(false);
        draw.draw_rectangle_filled(r1.min(r2), c1.min(c2), r1.max(r2), c1.max(c2), true);
        draw.update_display(display)?;
        timer.delay_ms(500);
    }
    Ok(())
}

/// Game of Life: Conway's Cellular Automata simulation with wrap-around edges and automatic re-seeding.
fn run_game_of_life<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: GameOfLife");
    let mut draw = Drawing::new();

    // Initial random seed
    for r in &mut draw.buffer {
        *r = rng.next() as u8;
    }

    let start_time = timer.get_counter();
    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.update_display(display)?;
        timer.delay_ms(200);

        let mut next_buffer = [0u8; 8];
        for (r, row_val) in next_buffer.iter_mut().enumerate() {
            for c in 0..8 {
                let mut neighbors = 0;
                for dr in -1..=1 {
                    for dc in -1..=1 {
                        if dr == 0 && dc == 0 {
                            continue;
                        }
                        let nr = ((r as i8 + dr + 8) % 8) as usize;
                        let nc = ((c as i8 + dc + 8) % 8) as usize;
                        if (draw.buffer[nr] & (1 << (7 - nc))) != 0 {
                            neighbors += 1;
                        }
                    }
                }

                let is_alive = (draw.buffer[r] & (1 << (7 - c))) != 0;
                if neighbors == 3 || (is_alive && neighbors == 2) {
                    *row_val |= 1 << (7 - c);
                }
            }
        }

        // If stagnant or empty, re-seed
        if next_buffer == draw.buffer || next_buffer == [0; 8] {
            for r in &mut next_buffer {
                *r = rng.next() as u8;
            }
        }
        draw.buffer = next_buffer;
    }
    Ok(())
}

/// Heartbeat: A pulsing heart shape that transitions between two sizes.
fn run_heartbeat<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: Heartbeat");
    let mut draw = Drawing::new();
    let frames = [
        // Small Heart
        [
            0b00000000, 0b00000000, 0b00100100, 0b01111110, 0b00111100, 0b00011000, 0b00000000,
            0b00000000,
        ],
        // Large Heart
        [
            0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000,
            0b00000000,
        ],
    ];

    let mut current_frame = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = frames[current_frame];
        draw.update_display(display)?;

        current_frame = (current_frame + 1) % 2;

        let interval = if current_frame == 1 { 150 } else { 600 };
        timer.delay_ms(interval);
    }
    Ok(())
}

/// Human Walk: A simple stick figure animation showing a walking cycle.
fn run_human_walk<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: HumanWalk");
    let mut draw = Drawing::new();
    let frames = [
        [
            0b00011000, 0b00011000, 0b00001000, 0b00111100, 0b01001010, 0b00001000, 0b00010100,
            0b00100010,
        ],
        [
            0b00011000, 0b00011000, 0b00001000, 0b00011100, 0b00001000, 0b00001000, 0b00001000,
            0b00010100,
        ],
        [
            0b00011000, 0b00011000, 0b00001000, 0b00111100, 0b00101010, 0b00001000, 0b00010100,
            0b00001010,
        ],
        [
            0b00011000, 0b00011000, 0b00001000, 0b00011100, 0b00001000, 0b00001000, 0b00001000,
            0b00010100,
        ],
    ];

    let mut current = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = frames[current];
        draw.update_display(display)?;
        current = (current + 1) % 4;
        timer.delay_ms(250);
    }
    Ok(())
}

/// Line Path: Draws continuous lines between random points, clearing the screen every few steps.
fn run_line_path<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: LinePath");
    let mut draw = Drawing::new();
    let mut row = 3;
    let mut col = 3;
    let mut count = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        let new_row = rng.random_range(0, 7) as i8;
        let new_col = rng.random_range(0, 7) as i8;

        draw.draw_line(row, col, new_row, new_col, true);
        draw.update_display(display)?;

        row = new_row;
        col = new_col;
        count += 1;

        if count == 4 {
            count = 0;
            timer.delay_ms(500);
            draw.clear_screen(false);
        } else {
            timer.delay_ms(500);
        }
    }
    Ok(())
}

/// Matrix Waterfall: A classic digital rain effect dropping pixels down the screen.
fn run_matrix_waterfall<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: MatrixWaterfall");
    let mut draw = Drawing::new();
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.shift_down(false);
        draw.set_pixel(0, rng.random_range(0, 7) as u8);
        draw.update_display(display)?;
        timer.delay_ms(100);
    }
    Ok(())
}

/// Matrix Waterfall Reversing: Digital rain that slows down and reverses direction based on a sine wave.
fn run_matrix_waterfall_reversing<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: MatrixWaterfallReversing");
    let mut draw = Drawing::new();
    let mut state: f32 = 1.0;
    let mut step: f32 = -1.0 / 64.0;
    let w = core::f32::consts::PI / 2.0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        let r = if state >= 0.0 {
            draw.shift_down(false);
            0
        } else {
            draw.shift_up(false);
            7
        };

        for _ in 0..2 {
            draw.set_pixel(r, rng.random_range(0, 7) as u8);
        }

        draw.update_display(display)?;

        state += step;
        if state >= 1.0 || state <= -1.0 {
            step *= -1.0;
        }

        let s = libm::sinf(w * state);
        let interval = (50.0 + 300.0 * (1.0 - s * s)) as u32;

        timer.delay_ms(interval);
    }
    Ok(())
}

/// Moving Columns: A single vertical column that scans back and forth horizontally.
fn run_moving_columns<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: MovingColumns");
    let mut draw = Drawing::new();
    let mut col: i8 = 0;
    let mut direction: i8 = 1;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.clear_screen(false);
        draw.draw_column(col as u8, true);
        draw.update_display(display)?;

        if col == 0 {
            direction = 1;
        }
        if col == 7 {
            direction = -1;
        }
        col += direction;

        timer.delay_ms(50);
    }
    Ok(())
}

/// PacMan: The classic character opening and closing its mouth.
fn run_pacman<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: PacMan");
    let mut draw = Drawing::new();
    let frames = [
        // Mouth Open
        [
            0b00111100, 0b01111110, 0b11100111, 0b11111000, 0b11111000, 0b11111111, 0b01111110,
            0b00111100,
        ],
        // Mouth Closed
        [
            0b00111100, 0b01111110, 0b11100111, 0b11111111, 0b11111111, 0b11111111, 0b01111110,
            0b00111100,
        ],
    ];

    let mut current = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = frames[current];
        draw.update_display(display)?;
        current = (current + 1) % 2;
        timer.delay_ms(200);
    }
    Ok(())
}

/// Pong Game: An automated game of Pong with a ball and two tracking paddles.
fn run_pong_game<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: PongGame");
    let mut draw = Drawing::new();
    let mut ball_r: f32 = 3.0;
    let mut ball_c: f32 = 3.0;
    let mut dr: f32 = 0.5;
    let mut dc: f32 = 1.0;

    let mut paddle1_r: i8 = 3;
    let mut paddle2_r: i8 = 4;
    let interval = 80;

    let start_time = timer.get_counter();
    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.clear_screen(false);

        // Draw left paddle (column 0)
        draw.draw_pixel(paddle1_r as u8, 0, true);
        draw.draw_pixel((paddle1_r + 1) as u8, 0, true);

        // Draw right paddle (column 7)
        draw.draw_pixel(paddle2_r as u8, 7, true);
        draw.draw_pixel((paddle2_r + 1) as u8, 7, true);

        // Draw ball
        let current_r = libm::roundf(ball_r) as i8;
        let current_c = libm::roundf(ball_c) as i8;
        if current_r >= 0 && current_c >= 0 {
            draw.draw_pixel(current_r as u8, current_c as u8, true);
        }

        // Move ball
        ball_r += dr;
        ball_c += dc;

        // Bounce walls
        if ball_r <= 0.0 || ball_r >= 7.0 {
            dr *= -1.0;
            ball_r = ball_r.clamp(0.0, 7.0);
        }

        let abs_dr = libm::fabsf(dr);

        // Paddle hit logic
        if ball_c <= 1.0 {
            dc *= -1.0;
            ball_c = 1.0;

            if dr > 0.0 && paddle1_r < 6 {
                paddle1_r += 1;
            }
            if dr < 0.0 && paddle1_r > 0 {
                paddle1_r -= 1;
            }

            dr = (dr + (rng.random_range(-10, 10) as f32) / 20.0).clamp(-1.0, 1.0);
            if abs_dr < 0.2 {
                dr = if dr > 0.0 { 0.3 } else { -0.3 };
            }
        } else if ball_c >= 6.0 {
            dc *= -1.0;
            ball_c = 6.0;

            if dr > 0.0 && paddle2_r < 6 {
                paddle2_r += 1;
            }
            if dr < 0.0 && paddle2_r > 0 {
                paddle2_r -= 1;
            }

            dr = (dr + (rng.random_range(-10, 10) as f32) / 20.0).clamp(-1.0, 1.0);
            if abs_dr < 0.2 {
                dr = if dr > 0.0 { 0.3 } else { -0.3 };
            }
        } else {
            // Normal travel predicting paddles slowly
            if dc < 0.0 && rng.random_range(0, 10) > 5 {
                if current_r > paddle1_r + 1 && paddle1_r < 6 {
                    paddle1_r += 1;
                }
                if current_r < paddle1_r && paddle1_r > 0 {
                    paddle1_r -= 1;
                }
            }
            if dc > 0.0 && rng.random_range(0, 10) > 5 {
                if current_r > paddle2_r + 1 && paddle2_r < 6 {
                    paddle2_r += 1;
                }
                if current_r < paddle2_r && paddle2_r > 0 {
                    paddle2_r -= 1;
                }
            }
        }

        draw.update_display(display)?;
        timer.delay_ms(interval);
    }
    Ok(())
}

/// Rectangles: Draws outlines of randomly positioned and sized rectangles.
fn run_rectangles<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: Rectangles");
    let mut draw = Drawing::new();
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        let r1 = rng.random_range(0, 7) as u8;
        let c1 = rng.random_range(0, 7) as u8;
        let r2 = rng.random_range(0, 7) as u8;
        let c2 = rng.random_range(0, 7) as u8;

        draw.clear_screen(false);
        draw.draw_rectangle(r1.min(r2), c1.min(c2), r1.max(r2), c1.max(c2), true);
        draw.update_display(display)?;
        timer.delay_ms(500);
    }
    Ok(())
}

/// Ripple Drop: Simulates a drop hitting water and creating expanding ripple rings.
fn run_ripple_drop<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: RippleDrop");
    let mut draw = Drawing::new();
    let frames = [
        [
            0b00000000, 0b00000000, 0b00010000, 0b00010000, 0b00000000, 0b00000000, 0b00000000,
            0b00010000,
        ],
        [
            0b00000000, 0b00000000, 0b00000000, 0b00010000, 0b00000000, 0b00000000, 0b00000000,
            0b00000000,
        ],
        [
            0b00000000, 0b00000000, 0b00000000, 0b00011000, 0b00011000, 0b00000000, 0b00000000,
            0b00000000,
        ],
        [
            0b00000000, 0b00000000, 0b00111100, 0b00100100, 0b00100100, 0b00111100, 0b00000000,
            0b00000000,
        ],
        [
            0b00000000, 0b01111110, 0b01000010, 0b01000010, 0b01000010, 0b01000010, 0b01111110,
            0b00000000,
        ],
        [
            0b11111111, 0b10000001, 0b10000001, 0b10000001, 0b10000001, 0b10000001, 0b10000001,
            0b11111111,
        ],
    ];

    let mut current = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = frames[current];
        draw.update_display(display)?;
        current = (current + 1) % 6;
        let interval = if current == 0 { 600 } else { 120 };
        timer.delay_ms(interval);
    }
    Ok(())
}

/// Sine Wave: A horizontally moving sine wave pattern.
fn run_sine_wave<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: SineWave");
    let mut draw = Drawing::new();
    let mut offset: f32 = 0.0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.clear_screen(false);
        for c in 0..8 {
            let angle = (c as f32 + offset) * 0.785;
            let r = (3.0 + libm::roundf(libm::sinf(angle) * 3.0)) as i8;
            draw.draw_pixel(r.clamp(0, 7) as u8, c as u8, true);
        }
        draw.update_display(display)?;
        offset += 1.0;
        timer.delay_ms(100);
    }
    Ok(())
}

/// Single Pixel Scanner: A complex single-pixel path that traverses the entire grid.
fn run_single_pixel_scanner<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: SinglePixelScanner");
    let mut draw = Drawing::new();
    let rstep: [[i8; 8]; 8] = [
        [0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 1, 1],
        [-1, 0, 0, 0, 0, 1, 1, 1],
        [-1, -1, 0, 0, 1, 1, 1, 1],
        [-1, -1, -1, -4, 0, 1, 1, 1],
        [-1, -1, -1, 0, 0, 0, 1, 1],
        [-1, -1, 0, 0, 0, 0, 0, 1],
        [-1, 0, 0, 0, 0, 0, 0, 0],
    ];
    let cstep: [[i8; 8]; 8] = [
        [1, 1, 1, 1, 1, 1, 1, 0],
        [1, 1, 1, 1, 1, 1, 0, 0],
        [0, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, -3, -1, 0, 0, 0],
        [0, 0, 0, -1, -1, -1, 0, 0],
        [0, 0, -1, -1, -1, -1, -1, 0],
        [0, -1, -1, -1, -1, -1, -1, -1],
    ];
    let toggle: [[u8; 8]; 8] = [
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
    ];

    let mut row: i8 = 0;
    let mut col: i8 = 0;
    let mut colour = true;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.draw_pixel(row as u8, col as u8, colour);
        draw.update_display(display)?;

        let dr = rstep[row as usize][col as usize];
        let dc = cstep[row as usize][col as usize];
        if toggle[row as usize][col as usize] != 0 {
            colour = !colour;
        }
        row += dr;
        col += dc;
        timer.delay_ms(16);
    }
    Ok(())
}

/// Space Invader: An animated alien sprite from the classic game.
fn run_space_invader<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: SpaceInvader");
    let mut draw = Drawing::new();
    let frames = [
        [
            0b00011000, 0b00111100, 0b01111110, 0b11011011, 0b11111111, 0b00100100, 0b01011010,
            0b10000001,
        ],
        [
            0b00011000, 0b00111100, 0b01111110, 0b11011011, 0b11111111, 0b00100100, 0b01000010,
            0b00100100,
        ],
    ];

    let mut current = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.buffer = frames[current];
        draw.update_display(display)?;
        current = (current + 1) % 2;
        timer.delay_ms(500);
    }
    Ok(())
}

/// Spinning Lines: Rotating lines that swivel around the center of the display.
fn run_spinning_lines<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: SpinningLines");
    let mut draw = Drawing::new();
    let mut j = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        draw.clear_screen(false);
        let i = j % 8;
        if j < 8 {
            draw.draw_line(i as i8, 0, 7 - i as i8, 7, true);
        } else {
            draw.draw_line(0, 7 - i as i8, 7, i as i8, true);
        }
        draw.update_display(display)?;
        j = (j + 1) % 16;
        timer.delay_ms(50);
    }
    Ok(())
}

/// Windy Particles: Particles that blow horizontally back and forth across the screen.
fn run_windy_particles<C, D>(
    timer: &mut hal::Timer<D>,
    display: &mut MAX7219<C>,
    rng: &mut Lcg,
) -> Result<(), max7219::DataError>
where
    C: max7219::connectors::Connector,
    D: TimerDevice,
{
    info!("Running Demo: WindyParticles");
    let mut draw = Drawing::new();
    let mut dir = true;
    let mut period = 0;
    let start_time = timer.get_counter();

    while timer.get_counter().ticks() - start_time.ticks() < 5_000_000 {
        if dir {
            draw.shift_left(false);
            draw.set_pixel(rng.random_range(0, 7) as u8, 7);
            draw.set_pixel(rng.random_range(0, 7) as u8, 7);
        } else {
            draw.shift_right(false);
            draw.set_pixel(rng.random_range(0, 7) as u8, 0);
            draw.set_pixel(rng.random_range(0, 7) as u8, 0);
        }
        draw.update_display(display)?;
        period = (period + 1) % 32;
        if period == 0 {
            dir = !dir;
        }
        timer.delay_ms(100);
    }
    Ok(())
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Initialize SPI0 on GP2 (SCK), GP3 (TX), GP5 (CS, configured manually)
    let spi_sclk: Pin<_, FunctionSpi, _> = pins.gpio2.into_function();
    let spi_tx: Pin<_, FunctionSpi, _> = pins.gpio3.into_function();
    let spi_rx: Pin<_, FunctionSpi, _> = pins.gpio4.into_function(); // RX / MISO

    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, (spi_tx, spi_rx, spi_sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10_000_000u32.Hz(), // 10MHz
        embedded_hal::spi::MODE_0,
    );

    let cs = pins.gpio5.into_push_pull_output();

    info!("Initializing MAX7219...");
    let mut display = MAX7219::from_spi_cs(1, spi, cs).unwrap();

    // We power it on
    display.power_on().unwrap();
    display.set_intensity(0, 0x07).unwrap(); // Mid brightness
    display.set_decode_mode(0, DecodeMode::NoDecode).unwrap();

    // Clear the display
    display.clear_display(0).unwrap();

    let mut rng = Lcg::new(timer.get_counter().ticks() as u32);

    info!("Starting animation sequence...");

    loop {
        let _ = run_blinking_smile(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_bouncing_ball(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_countdown(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_filled_rectangles(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_game_of_life(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_heartbeat(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_human_walk(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_line_path(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_matrix_waterfall(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_matrix_waterfall_reversing(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_moving_columns(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_pacman(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_pong_game(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_rectangles(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_ripple_drop(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_sine_wave(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_single_pixel_scanner(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_space_invader(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_spinning_lines(&mut timer, &mut display);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);

        let _ = run_windy_particles(&mut timer, &mut display, &mut rng);
        display.clear_display(0).unwrap();
        timer.delay_ms(2000);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"MAX7219 8x8 Dot Matrix Animations Demo"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
