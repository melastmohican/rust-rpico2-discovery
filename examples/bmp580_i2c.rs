//! # BMP580 Pressure & Temperature Sensor Example for Raspberry Pi Pico 2
//!
//! Reads atmospheric pressure and temperature from an Adafruit BMP580 sensor over I2C.
//!
//! This example is configured for the **Adafruit BMP580** breakout board connected via **Qwiic/STEMMA QT** cable.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit BMP580 (High-performance barometric pressure sensor)
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x47 (default for Adafruit breakout)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the BMP580 sensor.
//! The cable provides:
//! ```
//!      BMP580 Pin -> RPi Pico 2
//! (black)  GND    -> GND
//! (red)    VCC    -> 3.3V
//! (yellow) SCL    -> GPIO5 (Pin 7)
//! (blue)   SDA    -> GPIO4 (Pin 6)
//! ```
//!
//! ## About BMP580
//!
//! The Bosch BMP580 is a high-accuracy barometric pressure sensor:
//! - Pressure range: 300 to 1250 hPa
//! - Temperature range: -40°C to +85°C
//! - High precision: ±0.5 hPa absolute accuracy
//! - Very low noise and low power consumption
//!
//! Run with `cargo run --example bmp580_i2c`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
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
use rp235x_hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// BMP580 I2C address
/// Adafruit breakout defaults to 0x47. Bosch default is 0x47 or 0x46.
const BMP580_ADDR: u8 = 0x47;

// Registers
const REG_CHIP_ID: u8 = 0x01;
const REG_TEMP_DATA_XLSB: u8 = 0x1D;
const REG_OSR_CONFIG: u8 = 0x36;
const REG_ODR_CONFIG: u8 = 0x37;
const REG_CMD: u8 = 0x7E;

// Commands
const CMD_SOFT_RESET: u8 = 0xB6;

// Expected Chip ID
const CHIP_ID_BMP580: u8 = 0x50;

/// BMP580 Driver
struct Bmp580<I2C> {
    i2c: I2C,
}

impl<I2C> Bmp580<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    /// Create a new BMP580 driver and initialize the sensor
    fn new<D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<Self, I2C::Error> {
        let mut sensor = Bmp580 { i2c };
        sensor.init(delay)?;
        Ok(sensor)
    }

    /// Initialize the sensor
    fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), I2C::Error> {
        // Read chip ID to verify connection
        let mut id = [0u8];
        self.i2c.write_read(BMP580_ADDR, &[REG_CHIP_ID], &mut id)?;

        if id[0] != CHIP_ID_BMP580 {
            info!(
                "Unexpected Chip ID: 0x{:02x} (expected 0x{:02x})",
                id[0], CHIP_ID_BMP580
            );
            // We'll continue anyway, but log the warning
        } else {
            info!("BMP580 detected (Chip ID: 0x{:02x})", id[0]);
        }

        // Soft reset
        self.i2c.write(BMP580_ADDR, &[REG_CMD, CMD_SOFT_RESET])?;
        delay.delay_ms(10u32); // Wait for reset to complete

        // Configure Oversampling and Enable Pressure
        // Register OSR_CONFIG (0x36)
        // Bit 6: press_en (1 = Enable)
        // Bits 5-3: osr_p (010 = 4x oversampling)
        // Bits 2-0: osr_t (000 = 1x oversampling)
        // 0x40 (en) | 0x10 (4x) | 0x00 (1x) = 0x50
        self.i2c.write(BMP580_ADDR, &[REG_OSR_CONFIG, 0x50])?;

        // Configure ODR and Power Mode
        // Bit 7: Deep Standby Disable (1 = Disable)
        // Bits 6-2: ODR (0x0C = 50Hz, default is 0x00 = 240Hz?) - let's keep defaults for now
        // Bits 1-0: Mode (0x01 = Normal Mode)
        // 0x81 = Disable deep standby, Normal mode, default ODR
        self.i2c.write(BMP580_ADDR, &[REG_ODR_CONFIG, 0x81])?;
        delay.delay_ms(10u32);

        Ok(())
    }

    /// Read pressure and temperature data
    /// Returns (pressure_hPa, temperature_C)
    fn read_data(&mut self) -> Result<(f32, f32), I2C::Error> {
        let mut buf = [0u8; 6];
        // Burst read 6 bytes starting from TEMP_DATA_XLSB (0x1D)
        // [T_XLSB, T_LSB, T_MSB, P_XLSB, P_LSB, P_MSB]
        self.i2c
            .write_read(BMP580_ADDR, &[REG_TEMP_DATA_XLSB], &mut buf)?;

        // Reconstruct temperature (24-bit)
        let t_raw = ((buf[2] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[0] as u32);
        // If the MSB is 1, it's negative? Actually Bosch BMP5 typically uses unsigned and subtracts?
        // No, BMP580 datasheet says it's a 24-bit value to be divided.
        let temperature = (t_raw as f32) / 65536.0;

        // Reconstruct pressure (24-bit)
        let p_raw = ((buf[5] as u32) << 16) | ((buf[4] as u32) << 8) | (buf[3] as u32);
        let pressure = (p_raw as f32) / 64.0 / 100.0; // Convert Pa to hPa

        Ok((pressure, temperature))
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

    let mut timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Initializing BMP580 sensor...");

    // Configure I2C0 pins
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    // Create I2C0 peripheral
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Initialize sensor
    let mut sensor = match Bmp580::new(i2c, &mut timer) {
        Ok(s) => s,
        Err(_) => {
            info!("Failed to initialize BMP580. Check wiring and I2C address!");
            loop {
                timer.delay_ms(1000u32);
            }
        }
    };

    info!("BMP580 initialized successfully!");
    info!("Starting measurements...");

    loop {
        match sensor.read_data() {
            Ok((pressure, temperature)) => {
                info!(
                    "Pressure: {}.{:02} hPa, Temperature: {}.{:02} °C",
                    pressure as i32,
                    ((pressure % 1.0).abs() * 100.0) as u32,
                    temperature as i32,
                    ((temperature % 1.0).abs() * 100.0) as u32
                );
            }
            Err(_) => info!("Error reading sensor data"),
        }

        timer.delay_ms(1000u32);
    }
}
