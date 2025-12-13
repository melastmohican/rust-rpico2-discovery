//! # Zermatt Image Display with Falling Snow Effect
//!
//! Display a 320x240 image of Zermatt on the ILI9341 2.8" TFT LCD display with animated falling snow.
//!
//! This example demonstrates displaying a full-screen image in landscape mode with a physics-based
//! snow effect inspired by https://github.com/raphaelchampeimont/arduino_TFT_display_snow/
//!
//! ## Hardware: 2.8" TFT SPI 240x320 V1.2 Display Module
//!
//! ## Wiring for 2.8" TFT SPI 240x320 V1.2
//!
//! ```
//!      LCD Pin     ->  Raspberry Pi Pico 2
//! -----------------------------------------------
//!        VCC       ->  3.3V
//!        GND       ->  GND
//!        CS        ->  GPIO17  (Chip Select)
//!        RESET     ->  GPIO21  (Reset)
//!        DC        ->  GPIO20  (Data/Command)
//!        SDI(MOSI) ->  GPIO19  (SPI MOSI/Data)
//!        SCK       ->  GPIO18  (SPI Clock)
//!        LED       ->  3.3V (Backlight)
//!        SDO(MISO) ->  GPIO16  (SPI MISO/Data)
//! ```
//!
//! Run with `cargo run --example zermatt_snow`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use defmt::info;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    Drawable,
    draw_target::DrawTarget,
    geometry::Point,
    image::{GetPixel, Image},
    pixelcolor::{Rgb565, RgbColor},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::ClockSource;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSpi, Pin};
use hal::{Sio, Watchdog, clocks::init_clocks_and_plls, pac};
use mipidsi::{
    Builder,
    models::ILI9341Rgb565,
    options::{ColorOrder, Orientation, Rotation},
};
use rp235x_hal as hal;
use tinybmp::Bmp;

use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// Display dimensions in landscape mode
const DISPLAY_WIDTH: usize = 320;
const DISPLAY_HEIGHT: usize = 240;

// Physics engine grid size
const PHY_DISP_RATIO: usize = 2; // Physical cell size in pixels
const PHY_WIDTH: usize = DISPLAY_WIDTH / PHY_DISP_RATIO;
const PHY_HEIGHT: usize = DISPLAY_HEIGHT / PHY_DISP_RATIO;

// Grid storage (1 bit per cell)
const BITS_PER_CELL: usize = 1;
const CELLS_PER_BYTE: usize = 8 / BITS_PER_CELL;
const GRID_TOTAL_CELLS: usize = PHY_WIDTH * PHY_HEIGHT;
const GRID_SIZE_BYTES: usize = GRID_TOTAL_CELLS / CELLS_PER_BYTE;

const FLAKE_SIZE: i32 = 2; // Small 2x2 pixel snowflakes
const SNOW_COLOR: Rgb565 = Rgb565::WHITE;

// Simple PRNG state
static RNG_STATE: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(12345));

// Simple pseudo-random number generator
fn random_range(min: i32, max: i32) -> i32 {
    cortex_m::interrupt::free(|cs| {
        let mut state = RNG_STATE.borrow(cs).borrow_mut();
        // Linear congruential generator
        *state = state.wrapping_mul(1103515245).wrapping_add(12345);
        let random = (*state / 65536) % 32768;
        min + (random as i32 % (max - min))
    })
}

struct SnowGrid {
    grid: [u8; GRID_SIZE_BYTES],
}

impl SnowGrid {
    fn new() -> Self {
        Self {
            grid: [0u8; GRID_SIZE_BYTES],
        }
    }

    fn get_cell(&self, row: usize, col: usize) -> bool {
        let cell_index = row * PHY_WIDTH + col;
        let byte_index = cell_index / CELLS_PER_BYTE;
        let bit_index = cell_index % CELLS_PER_BYTE;
        (self.grid[byte_index] >> bit_index) & 1 == 1
    }

    fn set_cell(&mut self, row: usize, col: usize, value: bool) {
        let cell_index = row * PHY_WIDTH + col;
        let byte_index = cell_index / CELLS_PER_BYTE;
        let bit_index = cell_index % CELLS_PER_BYTE;

        if value {
            self.grid[byte_index] |= 1 << bit_index;
        } else {
            self.grid[byte_index] &= !(1 << bit_index);
        }
    }
}

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

    info!("Initializing ILI9341 TFT LCD display (2.8 inch 240x320)...");

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure SPI pins for ILI9341 display
    let sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    // Control pins
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio20.into_push_pull_output();
    let mut rst = pins.gpio21.into_push_pull_output();

    // Create SPI bus with 40 MHz clock speed
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        40_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    info!("SPI configured at 40 MHz");

    // Create exclusive SPI device with CS pin
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Create display interface
    let di = SPIInterface::new(spi_device, dc);

    // Create a delay using the system timer
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    info!("Initializing display with mipidsi...");

    // Reset the display
    let _ = rst.set_low();
    timer.delay_ms(10);
    let _ = rst.set_high();
    timer.delay_ms(120);

    // Create and initialize display in landscape mode
    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .display_size(240, 320)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .init(&mut timer)
        .unwrap();

    info!("Display initialized in landscape mode (320x240)!");

    // Clear screen to black
    display.clear(Rgb565::BLACK).unwrap();

    info!("Loading Zermatt image (320x240 BMP)...");

    // Load and display the BMP image
    let bmp_data = include_bytes!("zermatt_320x240.bmp");
    let bmp = Bmp::<Rgb565>::from_slice(bmp_data).expect("Failed to load BMP image");

    info!("Drawing Zermatt image...");
    let image = Image::new(&bmp, Point::new(0, 0));
    image.draw(&mut display).unwrap();

    info!("Image displayed! Starting snow animation...");

    // Initialize snow grid
    let mut snow_grid = SnowGrid::new();

    let mut frame_count = 0u32;

    // Main animation loop
    loop {
        // Simulate falling snow (iterate from bottom to top)
        for row in (0..PHY_HEIGHT - 1).rev() {
            for col in 0..PHY_WIDTH {
                if snow_grid.get_cell(row, col) {
                    // Calculate future column with slight randomness
                    let offset = random_range(-1, 2); // -1, 0, or 1
                    let future_col =
                        (col as i32 + offset).max(0).min(PHY_WIDTH as i32 - 1) as usize;

                    // Check if future cell is empty
                    if !snow_grid.get_cell(row + 1, future_col) {
                        // Move snowflake down
                        snow_grid.set_cell(row + 1, future_col, true);
                        render_flake(&mut display, row + 1, future_col);
                    }

                    // Clear current position
                    snow_grid.set_cell(row, col, false);
                    render_void(&mut display, bmp_data, row, col);
                }
            }
        }

        // Clear snowflakes that reached the bottom
        for col in 0..PHY_WIDTH {
            if snow_grid.get_cell(PHY_HEIGHT - 1, col) {
                snow_grid.set_cell(PHY_HEIGHT - 1, col, false);
                render_void(&mut display, bmp_data, PHY_HEIGHT - 1, col);
            }
        }

        // Create new snow at the top
        for col in 0..PHY_WIDTH {
            if random_range(0, 25) < 1 {
                snow_grid.set_cell(0, col, true);
                render_flake(&mut display, 0, col);
            }
        }

        // Delay between frames
        timer.delay_ms(20);

        frame_count += 1;
        if frame_count.is_multiple_of(50) {
            info!("Frame: {}", frame_count);
        }
    }
}

// Render a snowflake at the given grid position
fn render_flake(display: &mut impl DrawTarget<Color = Rgb565>, row: usize, col: usize) {
    let x = (col * PHY_DISP_RATIO) as i32;
    let y = (row * PHY_DISP_RATIO) as i32;

    // Draw a small 2x2 white square for each snowflake
    for dy in 0..FLAKE_SIZE {
        for dx in 0..FLAKE_SIZE {
            display
                .draw_iter(core::iter::once(embedded_graphics::Pixel(
                    Point::new(x + dx, y + dy),
                    SNOW_COLOR,
                )))
                .ok();
        }
    }
}

// Restore the background image at the given grid position
fn render_void(
    display: &mut impl DrawTarget<Color = Rgb565>,
    bmp_data: &[u8],
    row: usize,
    col: usize,
) {
    let x = (col * PHY_DISP_RATIO) as i32;
    let y = (row * PHY_DISP_RATIO) as i32;

    // Load the BMP and extract the pixel region
    if let Ok(bmp) = Bmp::<Rgb565>::from_slice(bmp_data) {
        // Create a small sub-image for this region
        let _sub_image = Image::new(&bmp, Point::new(-x, -y));

        // Use the display's fill_contiguous or draw pixel by pixel
        // For simplicity, redraw pixels one by one
        for dy in 0..FLAKE_SIZE {
            for dx in 0..FLAKE_SIZE {
                let px = x + dx;
                let py = y + dy;

                if px >= 0 && px < DISPLAY_WIDTH as i32 && py >= 0 && py < DISPLAY_HEIGHT as i32 {
                    // Get pixel from BMP
                    if let Some(pixel_color) = bmp.pixel(Point::new(px, py)) {
                        display
                            .draw_iter(core::iter::once(embedded_graphics::Pixel(
                                Point::new(px, py),
                                pixel_color,
                            )))
                            .ok();
                    }
                }
            }
        }
    }
}
