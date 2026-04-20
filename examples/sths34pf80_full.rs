//! # STHS34PF80 Full Feature Example
//!
//! This example provides full parity with official Arduino/C drivers for the STHS34PF80.
//! It accesses low-level registers for raw presence and motion data.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Sensor:** Adafruit STHS34PF80 IR Presence / Motion Sensor
//!
//! ## Wiring
//!
//! ```
//!      STHS34PF80 -> RPi Pico 2
//! (black)  GND    -> GND
//! (red)    VCC    -> 3.3V
//! (yellow) SCL    -> GPIO5 (Pin 7)
//! (blue)   SDA    -> GPIO4 (Pin 6)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example sths34pf80_full
//! ```
//!
//! ## Expected Output
//!
//! ```text
//! [INFO ] Amb: 23.64°C | Pres: 1536 | Mot: 0 | Obj: 7849 | Comp: 7849
//! [INFO ] Amb: 23.68°C | Pres: 1536 | Mot: 0 | Obj: 7805 | Comp: 7805
//! [INFO ] Amb: 23.59°C | Pres: 1536 | Mot: 0 | Obj: 7912 | Comp: 7912
//! ```

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embedded_hal::i2c::I2c as _;
use hal::block::ImageDef;
use sths34pf80::Sths34pf80;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

// Register Map from Datasheet
const STHS34PF80_ADDR: u8 = 0x5A;
const REG_PRESENCE_L: u8 = 0x22;
const REG_MOTION_L: u8 = 0x24;
const REG_TOBJECT_L: u8 = 0x26;
const REG_TAMBIENT_L: u8 = 0x28;
const REG_TCOMP_L: u8 = 0x38;

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

    info!("Initializing STHS34PF80 sensor (Full Example)...");

    // 1. USE CRATE FOR INITIALIZATION
    // We pass &mut timer so we don't lose ownership of the timer peripheral.
    info!("Phase 1: Initializing via sths34pf80 crate...");
    let mut sensor_driver = Sths34pf80::new(i2c, &mut timer);

    if let Err(e) = sensor_driver.initialize() {
        error!(
            "Failed to initialize via crate: {:?}",
            defmt::Debug2Format(&e)
        );
        loop {
            cortex_m::asm::wfi();
        }
    }
    info!("Sensor initialized successfully!");

    // 2. RELEASE I2C FOR MANUAL ACCESS
    // This gives us the I2c peripheral back so we can read any register we want.
    // The delay was passed as a reference, so it stays with us anyway.
    let mut i2c = sensor_driver.release();

    info!("Phase 2: Starting Manual Data Retrieval (Arduino Parity)...");

    loop {
        // Read buffer for 2-byte registers (Little Endian)
        let mut buf = [0u8; 2];

        // -- AMBIENT TEMPERATURE (0x28) --
        // Scale: 100 LSB/°C
        let amb_temp = if i2c
            .write_read(STHS34PF80_ADDR, &[REG_TAMBIENT_L], &mut buf)
            .is_ok()
        {
            let raw = i16::from_le_bytes(buf);
            raw as f32 / 100.0
        } else {
            0.0
        };

        // -- PRESENCE (0x22) --
        let presence = if i2c
            .write_read(STHS34PF80_ADDR, &[REG_PRESENCE_L], &mut buf)
            .is_ok()
        {
            i16::from_le_bytes(buf)
        } else {
            0
        };

        // -- MOTION (0x24) --
        let motion = if i2c
            .write_read(STHS34PF80_ADDR, &[REG_MOTION_L], &mut buf)
            .is_ok()
        {
            i16::from_le_bytes(buf)
        } else {
            0
        };

        // -- RAW OBJECT IR (0x26) --
        let obj_raw = if i2c
            .write_read(STHS34PF80_ADDR, &[REG_TOBJECT_L], &mut buf)
            .is_ok()
        {
            i16::from_le_bytes(buf)
        } else {
            0
        };

        // -- COMPENSATED OBJECT (0x38) --
        let obj_comp = if i2c
            .write_read(STHS34PF80_ADDR, &[REG_TCOMP_L], &mut buf)
            .is_ok()
        {
            i16::from_le_bytes(buf)
        } else {
            0
        };

        info!(
            "Amb: {}.{:02} C | Pres: {} | Mot: {} | Obj: {} | Comp: {}",
            amb_temp as i32,
            ((amb_temp.abs() % 1.0) * 100.0) as u32,
            presence,
            motion,
            obj_raw,
            obj_comp,
        );

        timer.delay_ms(1000);
    }
}
