//! # Grove Moisture Sensor Example for Raspberry Pi Pico 2
//!
//! Reads an analog moisture sensor value and prints it to the console with an interpretation.
//! This example takes two readings; the first is a "dummy" read to stabilize the sensor,
//! and the second is the one reported. This helps avoid capacitive drift.
//!
//! ## Hardware
//!
//! - **Sensor:** Grove - Moisture Sensor
//! - **Connection:** Analog
//!
//! ## Wiring
//!
//! ```
//!           Raspberry Pi Pico 2
//!         +--------------------------+
//!         |                          |
//!         | [ ] 1   40 [ ] USB       |
//!         | [ ] 2   39 [ ]           |
//!         | [ ] 3   38 [G]ND --------+ (black)
//!         | [ ] 4   37 [ ]           |
//!         | [ ] 5   36 [3]V3(OUT) ---+ (red)
//!         | [ ]...  ...[ ]           |
//!         | [ ] 29  32 [A]GPIO27 ----+ (yellow)
//!         | [ ]...  ...[ ]           |
//!         +--------------------------+
//!                |   |   |
//!                |   |   |
//!         +------|---|---|------------+
//!         |      G   V   S           |
//!         |      N   C   I           |
//!         |      D   C   G           |
//!         |                          |
//!         |   Grove Moisture Sensor  |
//!         +--------------------------+
//! ```
//!
//! **Connections:**
//!
//! *   **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38 is a convenient choice).
//! *   **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
//! *   **SIG (yellow wire):** Connects to an ADC-capable pin. In this example, this is `GPIO27` (Pin 32, ADC1).
//!
//! Run with `cargo run --example grove_moisture_sensor_adc`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::pac;
use rp235x_hal as hal;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embedded_hal_0_2::adc::OneShot;
use hal::adc::{Adc, AdcPin};
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// These thresholds are empirical and may need to be adjusted for your specific sensor and environment.
// Observed ADC ranges (approximate):
// - Dry (in air): ~0-150
// - Humid (damp soil/slight moisture): ~150-900
// - Wet (in water): ~900-1700 (or higher, depending on water conductivity)
// Observe the ADC readings for "dry", "humid", and "in water" states to calibrate.
const DRY_THRESHOLD: u16 = 150;
const HUMID_THRESHOLD: u16 = 900;

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

    // Set up the ADC
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO27 as an ADC input
    let mut adc_pin_1 = AdcPin::new(pins.gpio27.into_floating_input()).unwrap();

    info!("Moisture Sensor Example Started!");

    loop {
        // Take a "dummy" reading to stabilize the sensor
        let _: u16 = nb::block!(adc.read(&mut adc_pin_1)).unwrap();
        timer.delay_ms(100); // Wait a short time

        // Take the real reading
        let adc_raw_value: u16 = nb::block!(adc.read(&mut adc_pin_1)).unwrap();

        let moisture_status = if adc_raw_value < DRY_THRESHOLD {
            "Dry"
        } else if adc_raw_value < HUMID_THRESHOLD {
            "Humid"
        } else {
            "Wet (in water)"
        };

        info!("Raw ADC: {} - Status: {}", adc_raw_value, moisture_status);

        // Wait 1 second between measurements
        timer.delay_ms(1000);
    }
}
