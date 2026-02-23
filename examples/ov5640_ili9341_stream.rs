//! OV5640 Camera to ILI9341 Display Stream Example
//!
//! Wiring (matches Adafruit PiCowbell Camera + ILI9341 Display Breakout):
//! OV5640:  VSync=GPIO0, PowerDN=GPIO1, HRef=GPIO2, PCLK=GPIO3
//!          I2C SDA=GPIO4, I2C SCL=GPIO5
//!          D0-D7=GPIO6-GPIO13
//!          Reset=GPIO14, XCLK=Not connected (Uses PiCowbell onboard 16MHz oscillator)
//! ILI9341: SCK=GPIO18, MOSI=GPIO19, MISO=GPIO16, CS=GPIO17, RST=GPIO21, DC=GPIO20
//!
//! Camera outputs 240x320 RGB565 in portrait mode natively.
//! The PIO captures the data and the software rotates it 90 degrees to output
//! to the display in landscape mode (320x240).
//!
//! Physical Orientation & Coordinates:
//! - Display: ILI9341 Breakout. The "bottom" side is where the pin headers are located.
//! - Camera: Adafruit PiCowbell. Parallel to the Pico 2 board.
//! - Pico 2: Oriented with the USB port facing "down" (towards the user).
//!
//! Because the camera is physically parallel to a vertical Pico but the display
//! is used horizontally, the software maps the portrait camera buffer into
//! a landscape display view using a 90-degree clockwise rotation.
//!
//! Note on Performance:
//! Because the camera streams 16-bit pixels at 16MHz relentlessly, the RP2350 CPU
//! must drain the PIO RX FIFO quickly to avoid dropped pixels (which causes a slanted
//! color mesh artifact). This example MUST be compiled with optimizations enabled
//! (e.g., `--release` or `opt-level = 2` in dev profile) to prevent FIFO overflow.

#![no_std]
#![no_main]
#![allow(clippy::deref_addrof)]

use defmt_rtt as _;
use panic_probe as _;

use display_interface_spi::SPIInterface;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::{MonoTextStyleBuilder, ascii::FONT_10X20};
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionSpi, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::{Clock, Sio, Watchdog, clocks::init_clocks_and_plls};
use mipidsi::{
    Builder,
    models::ILI9341Rgb565,
    options::{ColorOrder, Orientation},
};
use rp235x_hal as hal;

mod ov5640 {
    // OV5640 Camera Driver Module for RP2-series Microcontrollers
    //!
    //! This driver is specifically designed for the Raspberry Pi RP2040 and RP2350.
    //! It relies on the RP2-series Programmable I/O (PIO) peripheral to handle the
    //! high-speed Digital Video Port (DVP) parallel data stream from the OV5640 sensor.
    //!
    //! Because the camera blasts 16-bit pixels at ~16MHz, standard bit-banging or
    //! generic I/O is insufficient to capture the data without dropping bytes.
    //! The PIO handles the critical timing of PCLK, HREF, and VSYNC signals in hardware.
    //!
    //! Note on Performance:
    //! Funtionality depends on the host application consuming the RX FIFO fast enough.
    //! Always compile applications using this driver with optimizations enabled (opt-level >= 2).

    #![allow(unused)]

    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::I2c;

    pub struct OV5640<I2C> {
        i2c: I2C,
        address: u8,
    }

    #[derive(Debug, Clone, Copy)]
    pub enum OutputFormat {
        Rgb565,
        Yuv422,
    }

    impl<I2C> OV5640<I2C>
    where
        I2C: I2c,
    {
        pub fn new(i2c: I2C) -> Self {
            Self { i2c, address: 0x3C }
        }

        pub fn get_id(&mut self) -> u16 {
            let high = self.read_reg(0x300A);
            let low = self.read_reg(0x300B);
            ((high as u16) << 8) | (low as u16)
        }

        pub fn write_reg(&mut self, reg: u16, val: u8) {
            let msg = [(reg >> 8) as u8, (reg & 0xFF) as u8, val];
            let _ = self.i2c.write(self.address, &msg);
        }

        pub fn read_reg(&mut self, reg: u16) -> u8 {
            let mut buf = [0u8; 1];
            let reg_msg = [(reg >> 8) as u8, (reg & 0xFF) as u8];
            let _ = self.i2c.write_read(self.address, &reg_msg, &mut buf);
            buf[0]
        }

        pub fn write_reg_2(&mut self, reg: u16, d1: u16, d2: u16) {
            self.write_reg(reg, (d1 >> 8) as u8);
            self.write_reg(reg + 1, (d1 & 0xFF) as u8);
            self.write_reg(reg + 2, (d2 >> 8) as u8);
            self.write_reg(reg + 3, (d2 & 0xFF) as u8);
        }

        pub fn init<D: DelayNs>(&mut self, delay: &mut D) {
            // Full initialization sequence from reference Waveshare/Adafruit drivers
            self.write_reg(0x3008, 0x82); // software reset
            delay.delay_ms(10);
            self.write_reg(0x3008, 0x42); // power down
            self.write_reg(0x3103, 0x13);
            self.write_reg(0x3017, 0xff);
            self.write_reg(0x3018, 0xff);
            self.write_reg(0x302c, 0xc3);
            self.write_reg(0x4740, 0x21); // CLOCK_POL_CONTROL: VSync Active High
            self.write_reg(0x4713, 0x02);
            self.write_reg(0x5001, 0x83);
            self.write_reg(0x3000, 0x20);
            delay.delay_ms(10);
            self.write_reg(0x3002, 0x1c);
            self.write_reg(0x3004, 0xff);
            self.write_reg(0x3006, 0xc3);
            self.write_reg(0x5000, 0xa7);
            self.write_reg(0x5001, 0xa3);
            self.write_reg(0x5003, 0x08);
            self.write_reg(0x370c, 0x02);
            self.write_reg(0x3634, 0x40);
            self.write_reg(0x3a02, 0x03);
            self.write_reg(0x3a03, 0xd8);
            self.write_reg(0x3a08, 0x01);
            self.write_reg(0x3a09, 0x27);
            self.write_reg(0x3a0a, 0x00);
            self.write_reg(0x3a0b, 0xf6);
            self.write_reg(0x3a0d, 0x04);
            self.write_reg(0x3a0e, 0x03);
            self.write_reg(0x3a0f, 0x30);
            self.write_reg(0x3a10, 0x28);
            self.write_reg(0x3a11, 0x60);
            self.write_reg(0x3a13, 0x43);
            self.write_reg(0x3a14, 0x03);
            self.write_reg(0x3a15, 0xd8);
            self.write_reg(0x3a18, 0x00);
            self.write_reg(0x3a19, 0xf8);
            self.write_reg(0x3a1b, 0x30);
            self.write_reg(0x3a1e, 0x26);
            self.write_reg(0x3a1f, 0x14);
            self.write_reg(0x3600, 0x08);
            self.write_reg(0x3601, 0x33);
            self.write_reg(0x3c01, 0xa4);
            self.write_reg(0x3c04, 0x28);
            self.write_reg(0x3c05, 0x98);
            self.write_reg(0x3c06, 0x00);
            self.write_reg(0x3c07, 0x08);
            self.write_reg(0x3c08, 0x00);
            self.write_reg(0x3c09, 0x1c);
            self.write_reg(0x3c0a, 0x9c);
            self.write_reg(0x3c0b, 0x40);
            self.write_reg(0x460c, 0x22);
            self.write_reg(0x4001, 0x02);
            self.write_reg(0x4004, 0x02);
            self.write_reg(0x5180, 0xff);
            self.write_reg(0x5181, 0xf2);
            self.write_reg(0x5182, 0x00);
            self.write_reg(0x5183, 0x14);
            self.write_reg(0x5184, 0x25);
            self.write_reg(0x5185, 0x24);
            self.write_reg(0x5186, 0x09);
            self.write_reg(0x5187, 0x09);
            self.write_reg(0x5188, 0x09);
            self.write_reg(0x5189, 0x75);
            self.write_reg(0x518a, 0x54);
            self.write_reg(0x518b, 0xe0);
            self.write_reg(0x518c, 0xb2);
            self.write_reg(0x518d, 0x42);
            self.write_reg(0x518e, 0x3d);
            self.write_reg(0x518f, 0x56);
            self.write_reg(0x5190, 0x46);
            self.write_reg(0x5191, 0xf8);
            self.write_reg(0x5192, 0x04);
            self.write_reg(0x5193, 0x70);
            self.write_reg(0x5194, 0xf0);
            self.write_reg(0x5195, 0xf0);
            self.write_reg(0x5196, 0x03);
            self.write_reg(0x5197, 0x01);
            self.write_reg(0x5198, 0x04);
            self.write_reg(0x5199, 0x12);
            self.write_reg(0x519a, 0x04);
            self.write_reg(0x519b, 0x00);
            self.write_reg(0x519c, 0x06);
            self.write_reg(0x519d, 0x82);
            self.write_reg(0x519e, 0x38);
            self.write_reg(0x5381, 0x1e);
            self.write_reg(0x5382, 0x5b);
            self.write_reg(0x5383, 0x08);
            self.write_reg(0x5384, 0x0a);
            self.write_reg(0x5385, 0x7e);
            self.write_reg(0x5386, 0x88);
            self.write_reg(0x5387, 0x7c);
            self.write_reg(0x5388, 0x6c);
            self.write_reg(0x5389, 0x10);
            self.write_reg(0x538a, 0x01);
            self.write_reg(0x538b, 0x98);
            self.write_reg(0x5300, 0x10);
            self.write_reg(0x5301, 0x10);
            self.write_reg(0x5302, 0x18);
            self.write_reg(0x5303, 0x19);
            self.write_reg(0x5304, 0x10);
            self.write_reg(0x5305, 0x10);
            self.write_reg(0x5306, 0x08);
            self.write_reg(0x5307, 0x16);
            self.write_reg(0x5308, 0x40);
            self.write_reg(0x5309, 0x10);
            self.write_reg(0x530a, 0x10);
            self.write_reg(0x530b, 0x04);
            self.write_reg(0x530c, 0x06);
            self.write_reg(0x5480, 0x01);
            self.write_reg(0x5481, 0x00);
            self.write_reg(0x5482, 0x1e);
            self.write_reg(0x5483, 0x3b);
            self.write_reg(0x5484, 0x58);
            self.write_reg(0x5485, 0x66);
            self.write_reg(0x5486, 0x71);
            self.write_reg(0x5487, 0x7d);
            self.write_reg(0x5488, 0x83);
            self.write_reg(0x5489, 0x8f);
            self.write_reg(0x548a, 0x98);
            self.write_reg(0x548b, 0xa6);
            self.write_reg(0x548c, 0xb8);
            self.write_reg(0x548d, 0xca);
            self.write_reg(0x548e, 0xd7);
            self.write_reg(0x548f, 0xe3);
            self.write_reg(0x5490, 0x1d);
            self.write_reg(0x5580, 0x04);
            self.write_reg(0x5583, 0x40);
            self.write_reg(0x5584, 0x10);
            self.write_reg(0x5586, 0x20);
            self.write_reg(0x5587, 0x00);
            self.write_reg(0x5588, 0x01);
            self.write_reg(0x5589, 0x10);
            self.write_reg(0x558a, 0x00);
            self.write_reg(0x558b, 0xf8);
            self.write_reg(0x501d, 0x40);
            self.write_reg(0x3008, 0x02);
            self.write_reg(0x3c00, 0x04);
            delay.delay_ms(300);

            // Exact C demo portrait registers (240x320) — proven to produce a real image.
            // Sensor window: X=352..1792 (1440 wide), Y=26..1946 (1920 tall)
            // Output: 240 wide × 320 tall (portrait)
            // HTS=2592, VTS=1944 (matched to this PLL config)
            self.write_reg(0x3800, 0x01); // X start high
            self.write_reg(0x3801, 0x60); // X start low  (= 352)
            self.write_reg(0x3802, 0x00); // Y start high
            self.write_reg(0x3803, 0x1A); // Y start low  (= 26)
            self.write_reg(0x3804, 0x07); // X end high
            self.write_reg(0x3805, 0x00); // X end low    (= 1792)
            self.write_reg(0x3806, 0x07); // Y end high
            self.write_reg(0x3807, 0x9A); // Y end low    (= 1946)
            self.write_reg(0x3808, 0x00); // DVP out width high
            self.write_reg(0x3809, 0xF0); // DVP out width low   (= 240)
            self.write_reg(0x380a, 0x01); // DVP out height high
            self.write_reg(0x380b, 0x40); // DVP out height low  (= 320)
            self.write_reg(0x380c, 0x0A); // HTS high
            self.write_reg(0x380d, 0x20); // HTS low    (= 2592)
            self.write_reg(0x380e, 0x07); // VTS high
            self.write_reg(0x380f, 0x98); // VTS low    (= 1944)
            self.write_reg(0x3810, 0x00); // ISP X offset high
            self.write_reg(0x3811, 0x10); // ISP X offset low  (= 16)
            self.write_reg(0x3812, 0x00); // ISP Y offset high
            self.write_reg(0x3813, 0x0E); // ISP Y offset low  (= 14)

            let dat = self.read_reg(0x5001) | 0x20; // ISP scaling enable
            self.write_reg(0x5001, dat);
            delay.delay_ms(50);

            self.write_reg(0x3820, 0x01); // from C demo
            self.write_reg(0x3821, 0x00);
            self.write_reg(0x4514, 0xAA);
            self.write_reg(0x4520, 0x0B);
            self.write_reg(0x3814, 0x31); // x subsample (same as C demo)
            self.write_reg(0x3815, 0x31); // y subsample
            delay.delay_ms(50);

            // PLL Settings (Match C demo closely)
            self.write_reg(0x3034, 0x1A);
            self.write_reg(0x3035, 0x11);
            self.write_reg(0x3036, 11);
            self.write_reg(0x3037, 0x01);
            self.write_reg(0x3108, 0x16);
            self.write_reg(0x3824, 0x04);
            self.write_reg(0x460c, 0x22);
            self.write_reg(0x3103, 0x13);
            delay.delay_ms(50);

            // set_colorspace RGB565 and BGR order
            self.write_reg(0x501F, 0x01);
            self.write_reg(0x4300, 0x61); // RGB565 + BGR order
            delay.delay_ms(50);
        }

        pub fn set_mirror(&mut self, enabled: bool) {
            let val = self.read_reg(0x3821);
            if enabled {
                self.write_reg(0x3821, val | 0x06);
            } else {
                self.write_reg(0x3821, val & !0x06);
            }
        }

        pub fn set_flip(&mut self, enabled: bool) {
            let val = self.read_reg(0x3820);
            if enabled {
                self.write_reg(0x3820, val | 0x06);
            } else {
                self.write_reg(0x3820, val & !0x06);
            }
        }

        pub fn set_resolution(&mut self, width: u16, height: u16) {
            self.write_reg_2(0x3808, width, height);
            let dat = self.read_reg(0x5001) | 0x20;
            self.write_reg(0x5001, dat);
            self.write_reg_2(0x380c, 2592, 1944);
        }

        pub fn set_output_format(&mut self, format: OutputFormat) {
            match format {
                OutputFormat::Rgb565 => {
                    self.write_reg(0x501F, 0x01);
                    self.write_reg(0x4300, 0x61);
                }
                OutputFormat::Yuv422 => {
                    self.write_reg(0x501F, 0x00);
                    self.write_reg(0x4300, 0x30);
                }
            }
        }
    }
}

static mut FRAME_BUFFER: [u16; 240 * 320] = [0u16; 240 * 320];

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[hal::entry]
fn main() -> ! {
    defmt::info!("Program Started!");
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

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // --- XCLK for Camera ---
    // PiCowbell has an onboard 16MHz oscillator. By default, it ignores external XCLK
    // unless a jumper is cut on the back of the board.
    // We do NOT need to generate a PWM clock from the RP2350. Doing so without cutting
    // the jumper creates clock contention/noise, resulting in vertical "mesh" tearing.
    defmt::info!("Using PiCowbell onboard 16MHz oscillator for XCLK...");

    // Give the camera time to stabilise after power up
    timer.delay_ms(10);

    // --- OV5640 Reset and Power pins (PiCowbell) ---
    let mut cam_reset = pins.gpio14.into_push_pull_output();
    let mut cam_pwdn = pins.gpio1.into_push_pull_output();
    // PowerDN low = active (powered up), Reset high = running
    let _ = cam_pwdn.set_low();
    let _ = cam_reset.set_low();
    timer.delay_ms(5);
    let _ = cam_reset.set_high();
    timer.delay_ms(5);

    // --- OV5640 I2C: SDA=GPIO4, SCL=GPIO5 on I2C0 (PiCowbell) ---
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        pins.gpio4
            .into_function::<hal::gpio::FunctionI2c>()
            .into_pull_type::<hal::gpio::PullUp>(),
        pins.gpio5
            .into_function::<hal::gpio::FunctionI2c>()
            .into_pull_type::<hal::gpio::PullUp>(),
        400_000_u32.Hz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    defmt::info!("Initialising OV5640...");
    let mut cam = ov5640::OV5640::new(i2c);
    cam.init(&mut timer);

    let id = cam.get_id();
    defmt::info!("OV5640 Chip ID: 0x{:04X}", id);
    if id != 0x5640 {
        panic!("Camera not detected! Check wiring. ID was 0x{:04X}", id);
    }
    defmt::info!("Camera OK!");

    // --- ILI9341 Display: SPI0 (Zermatt pins) ---
    defmt::info!("Initialising ILI9341 display...");
    let spi_sclk: Pin<_, FunctionSpi, _> = pins.gpio18.into_function::<FunctionSpi>();
    let spi_mosi: Pin<_, FunctionSpi, _> = pins.gpio19.into_function::<FunctionSpi>();
    let spi_miso: Pin<_, FunctionSpi, _> = pins.gpio16.into_function::<FunctionSpi>();
    let spi = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        60_000_000_u32.Hz(), // Fast SPI to reduce visible scanline
        embedded_hal::spi::MODE_0,
    );
    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio20.into_push_pull_output();
    let rst = pins.gpio21.into_push_pull_output();
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let di = SPIInterface::new(spi_device, dc);

    // Physical display is 240x320, we use landscape 320x240
    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .display_size(240, 320)
        .orientation(
            Orientation::new()
                .rotate(mipidsi::options::Rotation::Deg90)
                .flip_horizontal(),
        )
        .color_order(ColorOrder::Bgr)
        .init(&mut timer)
        .unwrap();

    let _ = display.clear(Rgb565::BLACK);
    defmt::info!("Display ready.");

    // --- Test pattern: RGB boxes + labels for landscape 320x240 ---
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::WHITE)
        .build();

    let _ = Rectangle::new(Point::new(0, 0), Size::new(320, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(&mut display);
    let _ = Rectangle::new(Point::new(0, 80), Size::new(320, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        .draw(&mut display);
    let _ = Rectangle::new(Point::new(0, 160), Size::new(320, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
        .draw(&mut display);
    let _ = Text::new("RED", Point::new(130, 40), text_style).draw(&mut display);
    let _ = Text::new("GREEN", Point::new(120, 120), text_style).draw(&mut display);
    let _ = Text::new("BLUE", Point::new(128, 200), text_style).draw(&mut display);

    defmt::info!("Test pattern shown. Starting stream in 5s...");
    timer.delay_ms(5000_u32);
    let _ = display.clear(Rgb565::BLACK);

    // Pins: VSYNC=GPIO0, HREF=GPIO2, PCLK=GPIO3, DATA=GPIO6-GPIO13 (PiCowbell)
    let _vsync = pins.gpio0.into_function::<hal::gpio::FunctionPio0>();
    let _href = pins.gpio2.into_function::<hal::gpio::FunctionPio0>();
    let _pclk = pins.gpio3.into_function::<hal::gpio::FunctionPio0>();
    let _d0 = pins.gpio6.into_function::<hal::gpio::FunctionPio0>();
    let _d1 = pins.gpio7.into_function::<hal::gpio::FunctionPio0>();
    let _d2 = pins.gpio8.into_function::<hal::gpio::FunctionPio0>();
    let _d3 = pins.gpio9.into_function::<hal::gpio::FunctionPio0>();
    let _d4 = pins.gpio10.into_function::<hal::gpio::FunctionPio0>();
    let _d5 = pins.gpio11.into_function::<hal::gpio::FunctionPio0>();
    let _d6 = pins.gpio12.into_function::<hal::gpio::FunctionPio0>();
    let _d7 = pins.gpio13.into_function::<hal::gpio::FunctionPio0>();

    // PIO program: capture 2 bytes per pixel
    // DATA pins start at GPIO6 so in_pin_base = 6.
    let pio_program = pio_proc::pio_asm!(
        ".wrap_target"
        "    out x, 32"            // Arduino instruction 0
        "    out y, 32"            // Arduino instruction 1
        "sync_vsync:"
        "    wait 0 gpio, 0"       // 2 VSync Low
        "    wait 1 gpio, 0"       // 3 VSync High
        "row_start:"
        "    mov x, y"             // 4
        "    wait 0 gpio, 2"       // 5 HREF Low
        "pixel_loop:"
        "    wait 1 gpio, 2"       // 6 HREF High (Ignore PCLK if HREF is low!)
        "    wait 1 gpio, 3"       // 7 PCLK High (Byte 1)
        "    in pins, 8"           // 8
        "    wait 0 gpio, 3"       // 9 PCLK Low
        "    wait 1 gpio, 3"       // 10 PCLK High (Byte 2)
        "    in pins, 8"           // 11
        "    wait 0 gpio, 3"       // 12 PCLK Low
        "    push block"           // 13
        "    jmp x-- pixel_loop"   // 14
        "    wait 0 gpio, 2"       // 15 HREF Low
        "    jmp row_start"        // 16
        ".wrap"
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&pio_program.program).unwrap();
    let (mut sm, mut rx_fifo, mut tx_fifo) =
        hal::pio::PIOBuilder::from_installed_program(installed)
            .in_pin_base(6) // data bits start at GPIO6
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .autopush(false)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .autopull(true)
            .pull_threshold(32)
            .build(sm0);

    // all DVP pins as inputs
    sm.set_pindirs((6u8..=13u8).map(|p| (p, hal::pio::PinDir::Input)));
    sm.set_pindirs([
        (0u8, hal::pio::PinDir::Input), // VSync
        (2u8, hal::pio::PinDir::Input), // HRef
        (3u8, hal::pio::PinDir::Input), // PCLK
    ]);
    sm.start();

    // --- Main capture + display loop ---
    const W_CAM: usize = 240;
    const H_CAM: usize = 320;
    defmt::info!(
        "Streaming started! Camera: {}x{}, Display: {}x{}",
        W_CAM,
        H_CAM,
        H_CAM,
        W_CAM
    );

    loop {
        tx_fifo.write(0); // Dummy for out x, 32
        tx_fifo.write((W_CAM * H_CAM - 1) as u32); // Total pixels

        // Use raw pointer to avoid static mut reference lints and closure move issues
        let fb_mut = unsafe { &mut *(&raw mut FRAME_BUFFER) };

        // Fast, purely sequential read loop to prevent PIO RX FIFO overflow!
        // The array is filled directly with the raw camera data.
        for p in fb_mut.iter_mut() {
            let pixel_word = loop {
                if let Some(w) = rx_fifo.read() {
                    break w;
                }
            };

            // ShiftDirection::Right pushes bytes into the top 16 bits.
            *p = ((pixel_word >> 16) as u16).swap_bytes();
        }

        defmt::info!("Frame captured, drawing...");
        let fb_ref = unsafe { &*(&raw const FRAME_BUFFER) };

        // Push the landscape buffer straight to the display while applying the software rotation
        let _ = display.set_pixels(
            0,
            0,
            (H_CAM - 1) as u16, // 319 (Width of landscape display)
            (W_CAM - 1) as u16, // 239 (Height of landscape display)
            (0..W_CAM).flat_map(|y_disp| {
                (0..H_CAM).map(move |x_disp| {
                    // Rotate 90 degrees clockwise
                    let cam_x = (W_CAM - 1) - y_disp;
                    let cam_y = x_disp;

                    let p = fb_ref[cam_y * W_CAM + cam_x];

                    let r = ((p >> 11) & 0x1F) as u8;
                    let g = ((p >> 5) & 0x3F) as u8;
                    let b = (p & 0x1F) as u8;
                    Rgb565::new(r, g, b)
                })
            }),
        );
    }
}
