//! # Newxie Digital-to-Analog Thermometer Example
//!
//! It reads temperature and pressure from a BMP580 sensor and displays a
//! graphical thermometer and data on an Adafruit 1.14" 240x135 Color Newxie TFT Display.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Display:** Adafruit 1.14" 240x135 Color Newxie TFT Display (ST7789)
//! - **Sensor:** Adafruit BMP580 (I2C)
//!
//! ## Wiring for Adafruit 1.14" Color Newxie TFT
//!
//! ```
//!      Breakout Pin  ->  RPi Pico 2 GPIO
//!      V+            ->  3.3V
//!      G             ->  GND
//!      CL (Clock)    ->  GPIO18 (SCK)
//!      DA (Data)     ->  GPIO19 (MOSI)
//!      CS (Chip Sel) ->  GPIO17
//!      DC (Data/Cmd) ->  GPIO20
//!      RST (Reset)   ->  GPIO21
//!      BL (B-Light)  ->  GPIO14
//! ```
//!
//! ## Wiring for BMP580 (I2C)
//!
//! ```
//!      Sensor Pin    ->  RPi Pico 2 GPIO
//!      SCL           ->  GPIO5 (I2C0 SCL)
//!      SDA           ->  GPIO4 (I2C0 SDA)
//!      VIN           ->  3.3V
//!      GND           ->  GND
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example newxie_thermometer
//! ```

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_9X15_BOLD},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::ClockSource;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2c, FunctionSpi, Pin};
use hal::pac;
use mipidsi::{
    Builder,
    models::ST7789,
    options::{ColorOrder, Orientation, Rotation},
};
use rp235x_hal as hal;
use rp235x_hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// --- BMP580 Driver (Ported from bmp580_i2c.rs) ---

const BMP580_ADDR: u8 = 0x47;
const REG_CHIP_ID: u8 = 0x01;
const REG_TEMP_DATA_XLSB: u8 = 0x1D;
const REG_OSR_CONFIG: u8 = 0x36;
const REG_ODR_CONFIG: u8 = 0x37;
const REG_CMD: u8 = 0x7E;
const CMD_SOFT_RESET: u8 = 0xB6;
const CHIP_ID_BMP580: u8 = 0x50;

struct Bmp580<I2C> {
    i2c: I2C,
    chip_id: u8,
}

impl<I2C> Bmp580<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new<D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<Self, I2C::Error> {
        let mut sensor = Bmp580 { i2c, chip_id: 0 };
        sensor.init(delay)?;
        Ok(sensor)
    }

    fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I2C::Error> {
        let mut id = [0u8];
        self.i2c.write_read(BMP580_ADDR, &[REG_CHIP_ID], &mut id)?;
        self.chip_id = id[0];

        if self.chip_id != CHIP_ID_BMP580 {
            info!(
                "Unexpected Chip ID: 0x{:x} (expected 0x{:x})",
                self.chip_id, CHIP_ID_BMP580
            );
        } else {
            info!("BMP580 detected (Chip ID: 0x{:x})", self.chip_id);
        }

        self.i2c.write(BMP580_ADDR, &[REG_CMD, CMD_SOFT_RESET])?;
        delay.delay_ms(10);

        self.i2c.write(BMP580_ADDR, &[REG_OSR_CONFIG, 0x50])?;
        self.i2c.write(BMP580_ADDR, &[REG_ODR_CONFIG, 0x81])?;
        delay.delay_ms(10);

        Ok(())
    }

    pub fn read_data(&mut self) -> Result<(f32, f32), I2C::Error> {
        let mut buf = [0u8; 6];
        self.i2c
            .write_read(BMP580_ADDR, &[REG_TEMP_DATA_XLSB], &mut buf)?;

        let t_raw = ((buf[2] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[0] as u32);
        let temperature = (t_raw as f32) / 65536.0;

        let p_raw = ((buf[5] as u32) << 16) | ((buf[4] as u32) << 8) | (buf[3] as u32);
        let pressure = (p_raw as f32) / 64.0 / 100.0;

        Ok((pressure, temperature))
    }
}

// --- Thermometer Graphic ---

#[derive(Clone, Copy)]
struct ThermometerColors {
    bg: Rgb565,
    outline: Rgb565,
    bulb_outline: Rgb565,
    bulb_fill: Rgb565,
    tick_major: Rgb565,
    fill_actual: Rgb565,
}

impl Default for ThermometerColors {
    fn default() -> Self {
        Self {
            bg: Rgb565::BLACK,
            outline: Rgb565::WHITE,
            bulb_outline: Rgb565::WHITE,
            bulb_fill: Rgb565::RED,
            tick_major: Rgb565::WHITE,
            fill_actual: Rgb565::RED,
        }
    }
}

struct ThermometerGraphic {
    anchor: Point,
    width: u32,
    height: u32,
    temp_min: f32,
    temp_max: f32,
    colors: ThermometerColors,
}

impl ThermometerGraphic {
    fn new(anchor: Point, width: u32, height: u32, temp_min: f32, temp_max: f32) -> Self {
        Self {
            anchor,
            width,
            height,
            temp_min,
            temp_max,
            colors: ThermometerColors::default(),
        }
    }

    fn draw_static<D>(&self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        Rectangle::new(self.anchor, Size::new(self.width, self.height))
            .into_styled(PrimitiveStyle::with_fill(self.colors.bg))
            .draw(target)?;

        let bulb_radius: i32 = 12;
        let bulb_center =
            self.anchor + Point::new(self.width as i32 / 2, self.height as i32 - bulb_radius - 10);

        Circle::with_center(bulb_center, (bulb_radius * 2) as u32)
            .into_styled(PrimitiveStyle::with_fill(self.colors.bulb_fill))
            .draw(target)?;

        Circle::with_center(bulb_center, (bulb_radius * 2) as u32)
            .into_styled(PrimitiveStyle::with_stroke(self.colors.bulb_outline, 1))
            .draw(target)?;

        let tube_width: u32 = 10;
        let tube_height = self.height - (bulb_radius as u32 * 2) - 20;
        let tube_top_left =
            self.anchor + Point::new((self.width as i32 / 2) - (tube_width as i32 / 2), 10);

        Rectangle::new(tube_top_left, Size::new(tube_width, tube_height))
            .into_styled(PrimitiveStyle::with_stroke(self.colors.outline, 1))
            .draw(target)?;

        let margin_top: i32 = 20;
        let margin_bottom: i32 = bulb_radius * 2 + 25;
        let active_height = self.height as i32 - margin_top - margin_bottom;
        let px_per_degree = active_height as f32 / (self.temp_max - self.temp_min);

        for temp in (self.temp_min as i32..=self.temp_max as i32).step_by(10) {
            let y = (self.anchor.y + margin_top + active_height)
                - ((temp as f32 - self.temp_min) * px_per_degree) as i32;
            Line::new(
                Point::new(tube_top_left.x - 5, y),
                Point::new(tube_top_left.x, y),
            )
            .into_styled(PrimitiveStyle::with_stroke(self.colors.tick_major, 1))
            .draw(target)?;
        }

        Ok(())
    }

    fn update_temp<D>(&self, target: &mut D, temp_f: f32) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        let bulb_radius: i32 = 12;
        let margin_top: i32 = 20;
        let margin_bottom: i32 = bulb_radius * 2 + 25;
        let active_height = self.height as i32 - margin_top - margin_bottom;
        let px_per_degree = active_height as f32 / (self.temp_max - self.temp_min);

        let tube_width: i32 = 6;
        let tube_top_left =
            self.anchor + Point::new((self.width as i32 / 2) - (tube_width / 2), 10);

        let inner_tube_height = self.height - (bulb_radius as u32 * 2) - 22;
        Rectangle::new(
            tube_top_left + Point::new(1, 1),
            Size::new((tube_width - 2) as u32, inner_tube_height),
        )
        .into_styled(PrimitiveStyle::with_fill(self.colors.bg))
        .draw(target)?;

        let fill_height =
            ((temp_f - self.temp_min) * px_per_degree).clamp(0.0, active_height as f32);
        let fill_y = (self.anchor.y + margin_top + active_height) - fill_height as i32;

        Rectangle::new(
            Point::new(tube_top_left.x + 2, fill_y),
            Size::new((tube_width - 4) as u32, fill_height as u32),
        )
        .into_styled(PrimitiveStyle::with_fill(self.colors.fill_actual))
        .draw(target)?;

        Ok(())
    }
}

// --- Main ---

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

    info!("Initializing Newxie Thermometer...");

    // 1. Initialize I2C for BMP580
    let sda_pin: Pin<_, FunctionI2c, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2c, _> = pins.gpio5.reconfigure();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut bmp = match Bmp580::new(i2c, &mut timer) {
        Ok(s) => s,
        Err(_) => {
            info!("Failed to initialize BMP580");
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    // 2. Initialize SPI for ST7789 Display
    let sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();

    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio20.into_push_pull_output();

    // RST and BL are optional in some 6-pin setups
    let _rst = pins.gpio21.into_push_pull_output();
    let mut bl = pins.gpio14.into_push_pull_output();

    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.get_freq(),
        40_000_000.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let di = SPIInterface::new(spi_device, dc);

    // Initial state for BL (optional)
    let _ = bl.set_high();

    info!("Building Display Driver (135x240 Portrait)...");
    let mut display = Builder::new(ST7789, di)
        .display_size(135, 240)
        .display_offset(52, 40)
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .orientation(Orientation {
            rotation: Rotation::Deg0,
            mirrored: false,
        })
        .color_order(ColorOrder::Rgb)
        .init(&mut timer)
        .unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    // 3. Setup Thermometer Graphic
    let therm = ThermometerGraphic::new(Point::new(10, 5), 115, 180, 40.0, 100.0);
    therm.draw_static(&mut display).unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X15_BOLD)
        .text_color(Rgb565::WHITE)
        .build();

    info!("Starting main loop...");

    let mut cycle: i32 = 0;
    let mut last_temp_c: f32 = 22.0;
    let mut last_press_hpa: f32 = 1013.25;

    loop {
        if let Ok((press, temp)) = bmp.read_data() {
            last_temp_c = temp;
            last_press_hpa = press;
        }

        info!(
            "Temp: {}.{} C | Pressure: {}.{} hPa",
            last_temp_c as i32,
            ((last_temp_c % 1.0).abs() * 10.0) as u32,
            last_press_hpa as i32,
            ((last_press_hpa % 1.0).abs() * 10.0) as u32
        );

        let temp_f = (last_temp_c * 9.0 / 5.0) + 32.0;
        therm.update_temp(&mut display, temp_f).unwrap();

        cycle = (cycle + 1) % 4;

        // Clear label area at the bottom
        Rectangle::new(Point::new(0, 190), Size::new(135, 50))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(&mut display)
            .unwrap();

        let mut buf = [0u8; 32];
        let mut writer = Writer {
            buf: &mut buf,
            pos: 0,
        };

        match cycle {
            0 => {
                let _ = core::fmt::write(&mut writer, format_args!("{:.1} F", temp_f));
            }
            1 => {
                let _ = core::fmt::write(&mut writer, format_args!("{:.1} C", last_temp_c));
            }
            2 => {
                let _ = core::fmt::write(&mut writer, format_args!("{:.1} hPa", last_press_hpa));
            }
            _ => {
                let _ = core::fmt::write(&mut writer, format_args!("BMP ID: 0x{:x}", bmp.chip_id));
            }
        }

        Text::with_baseline(
            writer.as_str(),
            Point::new(10, 200),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        timer.delay_ms(2000);
    }
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.pos]).unwrap_or("")
    }
}

impl<'a> core::fmt::Write for Writer<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let len = bytes.len();
        if self.pos + len <= self.buf.len() {
            self.buf[self.pos..self.pos + len].copy_from_slice(bytes);
            self.pos += len;
            Ok(())
        } else {
            Err(core::fmt::Error)
        }
    }
}
