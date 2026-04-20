//! # Sharp GP2Y1010AU0F Dust Sensor Example
//!
//! Reads dust density (PM2.5/PM10) from the Sharp GP2Y1010AU0F optical dust sensor.
//! This example is configured for the Waveshare Dust Sensor module.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** Waveshare Dust Sensor (Sharp GP2Y1010AU0F)
//!
//! ## Wiring
//!
//! The Waveshare breakout includes all required components (150Ω resistor, 220µF capacitor,
//! voltage regulator). Connect the 4-wire cable directly:
//!
//! ```
//!      Waveshare Dust Sensor -> Raspberry Pi Pico 2
//! (red)    VCC               -> 3V3(OUT) (Pin 36)
//! (black)  GND               -> GND (Pin 38)
//! (yellow) AOUT              -> GPIO26 (Pin 31, ADC0)
//! (blue)   ILED              -> GPIO22 (Pin 29)
//! ```
//!
//! ## How it Works
//!
//! 1. LED is pulsed ON for 0.32ms every 10ms.
//! 2. After 0.28ms delay, the analog output is sampled.
//! 3. Dust particles reflect light, increasing output voltage (0.5V per 0.1mg/m³).
//!
//! ## Run
//!
//! ```bash
//! cargo run --example gp2y1010au0f_dust
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::{FunctionSioOutput, Pin, PullDown};
use hal::pac;
use rp235x_hal as hal;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use embedded_hal::digital::OutputPin;
use embedded_hal_0_2::adc::OneShot;
use hal::adc::{Adc, AdcPin};
use hal::block::ImageDef;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// Timing constants (in microseconds)
const LED_PULSE_WIDTH_US: u32 = 320; // LED on time: 0.32ms
const SAMPLING_DELAY_US: u32 = 280; // Wait before reading: 0.28ms
const CYCLE_TIME_MS: u32 = 10; // Total cycle: 10ms

// ADC constants
const ADC_MAX: u32 = 4095; // 12-bit ADC
const ADC_VOLTAGE: f32 = 3.3; // Reference voltage

// Calibration constants
// The Waveshare board has voltage offset - measure in clean air to calibrate
// Based on typical readings: ~1.35V in clean air
const VOLTAGE_OFFSET: f32 = 1.35; // Clean air baseline voltage (adjust if needed)

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

    info!("Initializing Sharp GP2Y1010AU0F Dust Sensor...");

    // Configure LED control pin (ILED) - GPIO22
    // The sensor has an internal transistor, so LOW turns LED ON, HIGH turns LED OFF
    let mut led_pin: Pin<_, FunctionSioOutput, PullDown> = pins.gpio22.into_push_pull_output();
    led_pin.set_high().ok();
    info!("LED control pin configured (GPIO22)");

    // Configure ADC for analog output (AOUT) - GPIO26 (ADC0)
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin = AdcPin::new(pins.gpio26).unwrap();
    info!("ADC configured (GPIO26)");

    info!("");
    info!("=== Dust Sensor Information ===");
    info!("Sensor: Sharp GP2Y1010AU0F (Waveshare breakout)");
    info!("Measurement: PM2.5 and PM10 dust particles");
    info!("Timing: LED pulse 0.32ms, sample at 0.28ms, 10ms cycle");
    info!("");
    info!("Waveshare board includes all components onboard:");
    info!("- 150 Ohm resistor, 220 uF capacitor");
    info!("- PT1301 DC/DC converter (2.5V-5.5V input)");
    info!("- Signal conditioning circuitry");
    info!("");
    info!(
        "Calibration: Voltage offset = {} V (clean air baseline)",
        VOLTAGE_OFFSET as u32
    );
    info!("Note: If readings seem off, adjust VOLTAGE_OFFSET in code");
    info!("      to match your sensor's clean air voltage reading");
    info!("");

    timer.delay_ms(1000u32);

    info!("Starting dust measurements...");

    let mut sample_count: u32 = 0;
    let mut running_avg: f32 = 0.0;

    loop {
        // 1. Turn LED ON (LOW signal due to internal transistor)
        led_pin.set_low().ok();

        // 2. Wait for sensor to stabilize (0.28ms)
        timer.delay_us(SAMPLING_DELAY_US);

        // 3. Read analog value
        let raw_value: u16 = adc.read(&mut adc_pin).unwrap();

        // 4. Turn LED OFF (HIGH signal)
        led_pin.set_high().ok();

        // Convert ADC reading to voltage
        let voltage = (raw_value as f32 / ADC_MAX as f32) * ADC_VOLTAGE;

        // Subtract baseline voltage to get dust signal
        let voltage_dust = if voltage > VOLTAGE_OFFSET {
            voltage - VOLTAGE_OFFSET
        } else {
            0.0
        };

        // Calculate dust density in mg/m³
        // Formula from datasheet: sensitivity = 0.5V per 0.1mg/m³
        // Therefore: dust_density = (voltage_dust / 0.5V) * 0.1mg/m³
        // Simplified: dust_density = voltage_dust * 0.2
        let dust_density = voltage_dust * 0.2;

        // Update running average
        sample_count += 1;
        running_avg =
            (running_avg * (sample_count - 1) as f32 + dust_density) / sample_count as f32;

        // Determine air quality level
        let air_quality = if dust_density < 0.035 {
            "Good"
        } else if dust_density < 0.075 {
            "Moderate"
        } else {
            "Poor"
        };

        // Format voltage for display
        let v_int = voltage as u32;
        let v_frac = ((voltage % 1.0) * 1000.0) as u32;

        // Format dust density for display
        let d_int = dust_density as u32;
        let d_frac = ((dust_density % 1.0) * 1000.0) as u32;

        // Format running average for display
        let avg_int = running_avg as u32;
        let avg_frac = ((running_avg % 1.0) * 1000.0) as u32;

        info!("--- Sample {} ---", sample_count);
        info!("Raw ADC:      {}", raw_value);
        info!("Voltage:      {}.{:03} V", v_int, v_frac);
        info!("Dust Density: {}.{:03} mg/m³", d_int, d_frac);
        info!("Avg Density:  {}.{:03} mg/m³", avg_int, avg_frac);
        info!("Air Quality:  {}", air_quality);

        // Reset running average every 100 samples
        if sample_count.is_multiple_of(100) {
            running_avg = 0.0;
            sample_count = 0;
            info!("(Reset running average)");
        }

        // Wait for remainder of 10ms cycle
        let remaining_time_us = (CYCLE_TIME_MS * 1000) - LED_PULSE_WIDTH_US;
        timer.delay_us(remaining_time_us);
    }
}
