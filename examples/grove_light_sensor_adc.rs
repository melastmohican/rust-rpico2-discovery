//! # Grove Light Sensor v1.2 Example for Raspberry Pi Pico 2
//!
//! Reads an analog light sensor value and prints it to the console.
//!
//! ## Hardware
//!
//! - **Sensor:** Grove - Light Sensor v1.2
//! - **Connection:** Analog
//!
//! ## Wiring
//!
//! The Grove Light Sensor is an analog sensor and must be connected to an ADC pin.
//! On the Raspberry Pi Pico 2, ADC pins are GPIO26, GPIO27, and GPIO28.
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
//!         | [ ]...  ...[ ]           |
//!         | [ ] 30  31 [A]GPIO26 ----+ (yellow)
//!         | [ ] 29  32 [ ]           |
//!         | [ ]...  ...[ ]           |
//!         | [ ]...  ...[ ]           |
//!         +--------------------------+
//!                |   |   |
//!                |   |   |
//!         +------|---|---|------------+
//!         |      G   V   S           |
//!         |      N   C   I           |
//!         |      D   C   G           |
//!         |                          |
//!         |  Grove Light Sensor v1.2 |
//!         +--------------------------+
//! ```
//! **Connections:**
//!
//! *   **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38 is a convenient choice).
//! *   **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
//! *   **SIG (yellow wire):** Connects to an ADC-capable pin. In the example code, this is `GPIO26` (Pin 31).
//!
//! Run with `cargo run --example grove_light_sensor_adc`.

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
use micromath::F32Ext;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

const ADC_MAX_VALUE: f32 = 4095.0;
const VCC: f32 = 3.3;
const R2: f32 = 10000.0; // Fixed resistor in the voltage divider (10k Ohms)
// These constants (B and M) are approximate for a generic photoresistor.
// For precise measurements, they should be calibrated for the specific sensor.
const B_CONST: f32 = 1.3e7;
const M_CONST: f32 = -1.4;

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

    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    info!("ADC Example Started!");

    loop {
        // Read the ADC value from the pin
        let adc_raw_value: u16 = nb::block!(adc.read(&mut adc_pin_0)).unwrap();
        let adc_f32: f32 = adc_raw_value as f32;

        // Convert ADC value to voltage (Vout)
        let v_out = (adc_f32 * VCC) / ADC_MAX_VALUE;

        // With the Grove Light Sensor v1.2, the photoresistor is connected between VCC and SIG,
        // and the fixed resistor is between SIG and GND. This means in bright light, the
        // photoresistor's resistance is low, pulling the SIG voltage (v_out) low.
        // The correct formula for the photoresistor's resistance (r1) is:
        let r1 = R2 / (VCC / v_out - 1.0);

        // Calculate Lux using power-law formula (approximate)
        let lux = B_CONST * r1.powf(M_CONST);

        info!(
            "Raw ADC: {}, Resistance: {} Ohms, Lux: {}",
            adc_raw_value, r1, lux
        );

        // Wait 1 second between measurements
        timer.delay_ms(1000);
    }
}
