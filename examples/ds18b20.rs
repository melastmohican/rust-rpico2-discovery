//! # DS18B20 1-Wire Temperature Sensor Example
//!
//! Reads temperature from a DS18B20 temperature sensor over a 1-Wire bus.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** DS18B20 Waterproof Temperature Sensor Probe (or compatible)
//!
//! ## Wiring Schematic
//!
//! ```text
//!                               Raspberry Pi Pico 2
//!                            +-----------------------+
//!                            |                       |
//!                            | [ ] 1      40 [ ] USB |
//!                            | [ ] 2      39 [ ]     |
//!                            | [ ] 3      38 [G]ND --+-------+ (black)
//!                            | [ ] 4      37 [ ]     |       |
//!                            | [ ] 5      36 [3]V3 --+---+   |
//!                            |  ...        ...       |   |   |
//!                            | [ ] 20     21 [ ] ----+---+---|---+ (white, GPIO16)
//!                            +-----------------------+   |   |   |
//!                                                        |   |   |
//!                                                        |   |   |
//!                    +-----------------------------+     |   |   |
//!                    |     DS18B20 Sensor / Probe  |     |   |   |
//!                    |      (Bottom/Flat Side)     |     |   |   |
//!                    |                             |     |   |   |
//!                    |     [GND]   [DAT]   [VCC]   |     |   |   |
//!                    +-------|-------|-------|-----+     |   |   |
//!                            |       |       |           |   |   |
//!                            |       +-------+--[5K1]----+   | (Pull-Up Resistor
//!                            |       |       |   Resistor    |  between DAT & VCC)
//!                            |       |       +---------------+ (red)
//!                            +-------|-----------------------+ (black)
//!                                    |
//!                                    +--------------------------- (white)
//! ```
//!
//! ## Breadboard Layout Diagram
//!
//! ```text
//!                          Breadboard Columns (e.g. Columns 14-16)
//!                          
//!              Column 16 (GND)    Column 15 (VCC)    Column 14 (DAT)
//!              +-------------+    +-------------+    +-------------+
//!              |             |    |             |    |             |
//!              | [GND Pin]   |    | [VCC Pin]   |    | [DAT Pin]   |  <-- Sensor Breakout Pins
//!              |             |    |             |    |             |
//!              | [GND Wire]  |    | [VCC Wire]  |    | [DAT Wire]  |  <-- To Pico (GND, 3V3, GPIO16)
//!              |   (black)   |    |    (red)    |    |   (white)   |
//!              |             |    |             |    |             |
//!              |             |    |             |    | [Resistor]  |  <-- Pull-Up Resistor connected
//!              +-------------+    +-------------+    +------|------+      between Column 14 (DAT)
//!                                                           |             and the Red (+) Power Rail
//!                                                           |
//!                                                     [Red (+) Rail]
//! ```
//!
//! > [!IMPORTANT]
//! > **Pull-up Resistor Required:**
//! > You MUST connect a **4.7kΩ to 5.1kΩ pull-up resistor** between the Data (yellow)
//! > line and VCC (red) line. Without it, the 1-Wire bus will not function.
//!
//! > [!CAUTION]
//! > **Verify Your Sensor's Pinout Layout!**
//! > - A standalone TO-92 package sensor typically uses: **[GND] [DAT] [VCC]** (flat side facing you).
//! > - However, many pluggable breakout adapter boards use: **[GND] [VCC] [DAT]** (like the one in this example).
//! > Always check your specific hardware's markings! Wiring it incorrectly can cause the sensor to reverse-bias, heat up rapidly, and potentially damage the device.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example ds18b20
//! ```
//!
//! ## Expected Output
//!
//! When successfully connected, you should see logs similar to:
//! ```text
//! [INFO ] DS18B20 1-Wire Temperature Sensor Example
//! [INFO ] Searching for devices on the 1-Wire bus...
//! [INFO ] Found 1-Wire device address: [0x28, 0x93, 0xff, 0x77, 0x0, 0x0, 0x0, 0x11]
//! [INFO ] Device is identified as a DS18B20 temperature sensor!
//! [INFO ] Starting periodic temperature readings...
//! [INFO ] DS18B20 Temperature: 21.88 °C
//! [INFO ] DS18B20 Temperature: 21.81 °C
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;
use hal::gpio::Pins;
use hal::pac;
use rp235x_hal as hal;

use onewire::ds18b20::DS18B20;
use onewire::{DeviceSearch, OneWire};

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// Our 1-Wire pin is implemented using the built-in `hal::gpio::InOutPin` wrapper,
// which is designed specifically to emulate hardware open-drain by toggling the
// output-enable register fast and without runtime typestate reconfiguration overhead.

#[hal::entry]
fn main() -> ! {
    info!("DS18B20 1-Wire Temperature Sensor Example");

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = hal::clocks::init_clocks_and_plls(
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

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO16 (Pin 21) as pull-up input initially
    let wire_pin = pins.gpio16.into_pull_up_input();
    // Use the built-in `InOutPin` wrapper to emulate high-speed open-drain behavior
    let mut ow_pin = hal::gpio::InOutPin::new(wire_pin);

    // Initialize 1-Wire bus. parasite_mode = false (externally powered)
    let mut wire = OneWire::new(&mut ow_pin, false);

    info!("Searching for devices on the 1-Wire bus...");
    let mut search = DeviceSearch::new();
    let mut found_sensor = None;

    // Reset the bus and search for the first device
    if wire.reset(&mut timer).is_ok() {
        while let Ok(Some(device)) = wire.search_next(&mut search, &mut timer) {
            info!(
                "Found 1-Wire device address: {=[u8; 8]:#02x}",
                device.address
            );
            if device.address[0] == onewire::ds18b20::FAMILY_CODE {
                info!("Device is identified as a DS18B20 temperature sensor!");
                if let Ok(sensor) = DS18B20::new(device) {
                    found_sensor = Some(sensor);
                    break;
                }
            }
        }
    } else {
        error!("No devices found on the 1-Wire bus (is the pull-up resistor connected?)");
    }

    let ds18b20 = match found_sensor {
        Some(s) => s,
        None => {
            error!("DS18B20 sensor not found. Halting.");
            loop {
                cortex_m::asm::wfi();
            }
        }
    };

    info!("Starting periodic temperature readings...");

    loop {
        // Request temperature measurement
        match ds18b20.measure_temperature(&mut wire, &mut timer) {
            Ok(resolution) => {
                // Wait for measurement to complete (usually 750ms for 12-bit)
                timer.delay_ms(resolution.time_ms() as u32);

                // Read scratchpad to get temperature reading
                match ds18b20.read_temperature(&mut wire, &mut timer) {
                    Ok(temp_raw) => {
                        let (whole, frac) = onewire::ds18b20::split_temp(temp_raw);
                        // Convert ten-thousandths to rounded hundredths (2 decimal places)
                        let rounded_frac = (frac.abs() + 50) / 100;
                        if whole < 0 || (whole == 0 && frac < 0) {
                            info!(
                                "DS18B20 Temperature: -{}.{:02} °C",
                                whole.abs(),
                                rounded_frac
                            );
                        } else {
                            info!("DS18B20 Temperature: {}.{:02} °C", whole, rounded_frac);
                        }
                    }
                    Err(e) => {
                        error!("Failed to read temperature: {:?}", e);
                    }
                }
            }
            Err(e) => {
                error!("Failed to start temperature measurement: {:?}", e);
            }
        }

        // Wait 2 seconds before next reading
        timer.delay_ms(2000);
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"DS18B20 Waterproof Temperature Sensor example for RPi Pico 2"
    ),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
