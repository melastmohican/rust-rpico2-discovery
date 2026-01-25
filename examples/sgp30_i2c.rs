//! # SGP30 Air Quality Sensor Example for Raspberry Pi Pico 2
//!
//! Reads eCO2 (equivalent CO2) and TVOC (Total Volatile Organic Compounds) from an SGP30 sensor over I2C.
//!
//! This example is configured for the Adafruit SGP30 breakout board connected via Qwiic/STEMMA QT connector.
//!
//! ## Hardware
//!
//! - **Sensor:** Adafruit SGP30 Air Quality Sensor Breakout - VOC and eCO2
//! - **Connection:** Qwiic/STEMMA QT cable (I2C)
//! - **I2C Address:** 0x58 (fixed address)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Simply connect the Qwiic/STEMMA QT cable between the board and the SGP30 sensor.
//! The cable provides:
//! ```
//!      SGP30 -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7)
//! (blue)   SDA -> GPIO4 (Pin 6)
//! ```
//!
//! ## I2C Address
//!
//! The SGP30 has a fixed I2C address of 0x58 (cannot be changed).
//!
//! ## About the SGP30
//!
//! The SGP30 is a digital multi-pixel gas sensor designed for indoor air quality applications.
//! It features:
//! - eCO2 (equivalent CO2) measurement: 400-60000 ppm
//! - TVOC (Total Volatile Organic Compounds) measurement: 0-60000 ppb
//! - On-chip humidity compensation
//! - Dynamic baseline compensation algorithm
//! - Low power consumption
//! - 15-second initial warm-up period
//!
//! **Important:** The measure() function must be called at regular 1-second intervals
//! to ensure proper operation of the dynamic baseline compensation algorithm.
//! The baseline values can be stored and restored to speed up warm-up after power cycling.
//!
//! **Warm-up Timeline:**
//! - First 15 seconds: Fixed baseline values (400 ppm CO2, 0 ppb TVOC)
//! - Next 20 minutes: Sensor settling period for reliable readings
//! - First 12 hours: Baseline calibration period
//! - Brand new sensors: 48-hour factory burn-in recommended
//!
//! **Tips for faster calibration:**
//! - Expose sensor to fresh outdoor air for 10 minutes (helps establish baseline)
//! - Test VOC detection by breathing near sensor or using hand sanitizer
//! - Save baseline values after 12+ hours and restore on next power-up
//!
//! Run with `cargo run --example sgp30_i2c`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::block::ImageDef;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;
use rp235x_hal::entry;
use sgp30::Sgp30;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
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

    let delay = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Initializing SGP30 air quality sensor...");

    // Configure I2C0 pins
    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    // Create I2C0 peripheral with default configuration (100kHz)
    // SGP30 supports up to 400kHz (Fast mode)
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Initialize SGP30 sensor
    // Address 0x58 is fixed and cannot be changed
    let mut sgp30 = Sgp30::new(i2c, 0x58, delay);

    // Read and display the sensor's serial number
    match sgp30.serial() {
        Ok(serial) => {
            info!(
                "SGP30 Serial: {:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                serial[0], serial[1], serial[2], serial[3], serial[4], serial[5]
            );
        }
        Err(_) => {
            info!("Failed to read SGP30 serial number!");
            info!("Check wiring and I2C address (0x58)");
            loop {
                cortex_m::asm::wfi();
            }
        }
    }

    // Initialize the air quality measurement
    match sgp30.init() {
        Ok(_) => {
            info!("SGP30 initialized successfully!");
        }
        Err(_) => {
            info!("Failed to initialize SGP30!");
            loop {
                cortex_m::asm::wfi();
            }
        }
    }

    info!("Starting air quality measurements...");
    info!("");
    info!("=== SGP30 Warm-up Information ===");
    info!("1. First ~15 seconds: Fixed baseline (400 ppm CO2, 0 ppb TVOC)");
    info!("2. Next ~20 minutes: Sensor settling period");
    info!("3. First 12 hours: Baseline calibration period");
    info!("4. If brand new: 48-hour factory burn-in recommended");
    info!("");
    info!("Tip: Expose sensor to fresh outdoor air for 10 minutes to help calibration");
    info!("Tip: Breathe near sensor or use hand sanitizer to test VOC detection");
    info!("");

    let mut measurement_count = 0;
    let mut last_co2 = 400;
    let mut last_tvoc = 0;

    loop {
        // Take a measurement
        // IMPORTANT: This must be called every 1 second for proper baseline compensation
        match sgp30.measure() {
            Ok(measurement) => {
                measurement_count += 1;

                // Detect changes to provide feedback
                let co2_changed = measurement.co2eq_ppm != last_co2;
                let tvoc_changed = measurement.tvoc_ppb != last_tvoc;

                // Always show first 20 measurements, then only on change or every 60 seconds
                if measurement_count <= 20
                    || co2_changed
                    || tvoc_changed
                    || measurement_count % 60 == 0
                {
                    info!(
                        "--- Measurement {} ({} min {} sec) ---",
                        measurement_count,
                        measurement_count / 60,
                        measurement_count % 60
                    );
                    info!(
                        "eCO2:  {} ppm {}",
                        measurement.co2eq_ppm,
                        if co2_changed { "(CHANGED!)" } else { "" }
                    );
                    info!(
                        "TVOC:  {} ppb {}",
                        measurement.tvoc_ppb,
                        if tvoc_changed { "(CHANGED!)" } else { "" }
                    );

                    // Provide contextual feedback
                    if measurement_count < 20 {
                        info!("Status: Initial baseline period");
                    } else if measurement_count < 1200 {
                        info!(
                            "Status: Sensor settling (~{} min remaining)",
                            (1200 - measurement_count) / 60
                        );
                    } else if measurement_count < 43200 {
                        info!(
                            "Status: Baseline calibration (~{} hours remaining)",
                            (43200 - measurement_count) / 3600
                        );
                    } else {
                        info!("Status: Fully calibrated");
                    }
                }

                last_co2 = measurement.co2eq_ppm;
                last_tvoc = measurement.tvoc_ppb;

                // Every 5 minutes (300 seconds), show baseline info
                if measurement_count % 300 == 0 {
                    match sgp30.get_baseline() {
                        Ok(baseline) => {
                            info!("");
                            info!("=== Baseline Values (store for faster startup) ===");
                            info!(
                                "eCO2 baseline: 0x{:04X} ({})",
                                baseline.co2eq, baseline.co2eq
                            );
                            info!("TVOC baseline: 0x{:04X} ({})", baseline.tvoc, baseline.tvoc);
                            info!(
                                "To restore: sgp30.set_baseline(&Baseline {{ co2eq: {}, tvoc: {} }});",
                                baseline.co2eq, baseline.tvoc
                            );
                            info!("");
                        }
                        Err(_) => {
                            info!("Failed to read baseline values");
                        }
                    }
                }
            }
            Err(_) => {
                info!("Error reading sensor data!");
            }
        }

        // Wait 1 second between measurements (required for baseline algorithm)
        // Note: SGP30 driver holds the delay internally, so we use WFI here
        cortex_m::asm::delay(125_000_000); // Approximately 1 second at 125 MHz
    }
}
