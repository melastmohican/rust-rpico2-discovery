//! # PMS5003 PM2.5 Air Quality Sensor (Stable UART) Example
//!
//! Reads particulate matter data from a PMS5003 sensor over hardware UART1
//! using the `pmsx003` driver crate.
//!
//! ## Stability Strategies
//!
//! - **Active Mode**: The sensor is kept in its default "streaming" mode.
//! - **Drain & Hunt**: Before every measurement, the MCU drains the UART hardware FIFO
//!   to clear stale data, then "hunts" for a fresh frame header. This prevents
//!   UART buffer overruns and synchronization errors.
//! - **Wake & Verify**: On startup, the MCU listens for data and sends an explicit
//!   wake-up command if the sensor is silent.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** Adafruit PM2.5 Air Quality Sensor (PMS5003) with breadboard adapter
//!
//! ## Wiring
//!
//! | PMS5003 Pin | Pico 2 Pin     | GPIO | Function         |
//! |-------------|----------------|------|------------------|
//! | VCC         | VBUS (Pin 40)  | -    | 5V Power         |
//! | GND         | GND (Pin 38)   | -    | Ground           |
//! | TXD         | GP9 (Pin 12)   | GPIO9| MCU RX (UART1 RX)|
//! | RXD         | GP8 (Pin 11)   | GPIO8| MCU TX (UART1 TX)|
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pms5003
//! ```
//! ## Run
//!
//! ```bash
//! cargo run --example pms5003
//! ```

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;
use hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral};
use panic_probe as _;
use pmsx003::PmsX003Sensor;
use rp235x_hal as hal;
use rp235x_hal::clocks::ClockSource;
use rp235x_hal::fugit::RateExtU32;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[hal::entry]
fn main() -> ! {
    info!("PMS5003 PM2.5 Air Quality Sensor Example");

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Configure UART1 for PMS5003 communication
    // PMS5003 default baud rate is 9600
    let uart_pins = (
        pins.gpio8.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio9.into_function::<hal::gpio::FunctionUart>(),
    );

    let mut uart = UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.get_freq(),
        )
        .unwrap();

    info!("UART1 initialized at 9600 baud");

    // 1. Initial Wake Up / Detection Phase
    // Listen for data to see if the sensor is already streaming
    info!("Checking if sensor is active...");
    let mut data_detected = false;
    for _ in 0..10 {
        if uart.uart_is_readable() {
            data_detected = true;
            break;
        }
        timer.delay_ms(100);
    }

    if !data_detected {
        info!("Sensor silent. Sending wake-up command (Active Mode)...");
        let mut temp_sensor = PmsX003Sensor::new(&mut uart);
        let _ = temp_sensor.active();
        // Give it a moment to start the fan and laser
        timer.delay_ms(500);
    } else {
        info!("Sensor is already streaming.");
    }

    info!("MCU TX (GPIO8) -> Sensor RX");
    info!("MCU RX (GPIO9) <- Sensor TX");

    info!("PMS5003 sensor initialized (Active Mode)");
    info!("Starting Drain & Hunt polling (every 2 seconds)...");

    loop {
        // 1. Drain stale backlog from MCU UART buffer to ensure we catch the LATEST frame
        while uart.uart_is_readable() {
            use embedded_hal_0_2::serial::Read;
            let _ = uart.read();
        }

        // 2. Initialize "Hunting" for a valid frame
        let mut found = false;

        // Try to find a valid frame with a timeout
        for hunt_attempt in 1..=40 {
            // Defensive check: don't call blocking read unless some data is available
            // This avoids the borrow checker conflict by creating the sensor only when ready
            if uart.uart_is_readable() {
                let mut sensor = PmsX003Sensor::new(&mut uart);
                match sensor.read() {
                    Ok(frame) => {
                        println!("--- PMS5003 Report ---");
                        println!("PM1.0: {} μg/m³", frame.pm1_0);
                        println!("PM2.5: {} μg/m³", frame.pm2_5);
                        println!("PM10:  {} μg/m³", frame.pm10);
                        println!("----------------------");
                        found = true;
                        break;
                    }
                    Err(_e) => {
                        // Mid-packet sync or checksum error, retry quickly
                        timer.delay_ms(10);
                    }
                }
            } else {
                // No data yet, wait a bit
                timer.delay_ms(50);
            }

            if hunt_attempt == 20 && !found {
                // Halfway through and still nothing? Try to wake it up again just in case
                let mut sensor = PmsX003Sensor::new(&mut uart);
                let _ = sensor.active();
            }
        }

        if !found {
            warn!("Sensor timeout: No valid data received in this cycle.");
        }

        // Wait 2 seconds before the next reading cycle
        timer.delay_ms(2000);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"PMS5003 air quality sensor stable UART"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
