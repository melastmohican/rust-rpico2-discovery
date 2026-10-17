//! # Arduino Modulino Motors Example
//!
//! Demonstrates control of DC and stepper motors using the Arduino Modulino Motors module over I2C.
//!
//! This example provides complete feature parity with the official Arduino Modulino Motors examples:
//! - **DC Motor Basic Control** (`Motors_Basic.ino`): Ramping motor A and B forward and reverse with slow decay.
//! - **PWM Frequency Sweep** (`Motors_Frequency.ino`): Dynamically changing DC motor PWM frequencies (400 Hz to 20 kHz).
//! - **Stepper Control by RPM** (`Motors_Stepper_RPM.ino`): Position and RPM-based stepper movement in full-step and half-step modes, demonstrating holding torque versus coil release delays.
//! - **Telemetry & Current Sensing** (`Motors_Telemetry.ino`): Real-time bus telemetry, busy flags, decay modes, and comparison between Full-Scale and Half-Full-Scale (HFS) current sensing.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** Arduino Modulino Motors
//! - **Power:** When driving actual DC or stepper motors, connect appropriate external motor power (VIN/GND screw terminals)
//!
//! ## Wiring with Qwiic/STEMMA QT
//!
//! Connect the Qwiic/STEMMA QT cable between the Raspberry Pi Pico 2 and Modulino Motors:
//! ```
//!      Modulino Motors -> RPi Pico 2
//! (black)  GND -> GND
//! (red)    VCC -> 3.3V
//! (yellow) SCL -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example modulino_motors_i2c
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

use embedded_hal::delay::DelayNs;
use hal::block::ImageDef;

// Import from modulino library
use modulino::{DecayMode, Motors};
use rust_rpico2_discovery::Fmt;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Polls telemetry until the motor controller reports it is no longer busy.
fn wait_until_idle<I2C, E>(motors: &mut Motors<I2C>, delay: &mut impl DelayNs)
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    loop {
        if motors.update().is_ok() && !motors.busy() {
            break;
        }
        delay.delay_ms(10);
    }
}

/// Commands a stepper movement and waits for completion.
fn run_stepper_move<I2C, E>(
    motors: &mut Motors<I2C>,
    delay: &mut impl DelayNs,
    label: &str,
    steps: i32,
    rpm: f32,
    release_delay_ms: u8,
) where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    info!("{} | release_delay_ms={}", label, release_delay_ms);
    if let Err(e) = motors.move_stepper_rpm(steps, rpm, release_delay_ms) {
        error!("Failed to command stepper move: {:?}", Debug2Format(&e));
        return;
    }
    wait_until_idle(motors, delay);
}

/// Reads telemetry and prints current state and current-sensing values.
fn print_telemetry<I2C, E>(motors: &mut Motors<I2C>, label: &str)
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    if let Err(e) = motors.update() {
        error!("Telemetry read failed: {:?}", Debug2Format(&e));
        return;
    }

    info!(
        "{}: busy={}, mode={}, half_step={}, decay={:?}, HFS={}, I_A={} mA, I_B={} mA",
        label,
        motors.busy(),
        if motors.stepper_mode_enabled() {
            "stepper"
        } else {
            "dc"
        },
        motors.half_step_enabled(),
        motors.decay_mode(),
        motors.half_full_scale_enabled(),
        Fmt(motors.sensed_current_a()),
        Fmt(motors.sensed_current_b()),
    );
}

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

    let sda_pin: Pin<_, FunctionI2C, _> = pins.gpio4.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.gpio5.reconfigure();

    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    info!("Initializing Arduino Modulino Motors...");

    let mut motors = match Motors::new(i2c) {
        Ok(m) => m,
        Err(e) => {
            error!(
                "Failed to initialize Modulino Motors: {:?}",
                Debug2Format(&e)
            );
            loop {
                timer.delay_ms(1000);
            }
        }
    };

    info!(
        "Modulino Motors initialized at address 0x{:02X}!",
        motors.address()
    );

    loop {
        // ====================================================================
        // 1. Basic DC Motor Control (Motors_Basic.ino)
        // ====================================================================
        info!("--- Demo 1: Basic DC Motor Control ---");
        motors.set_stepper_mode_enabled(false).ok();
        motors.set_decay(DecayMode::Slow).ok();

        info!("Motor A forward at 55%");
        motors.set_invert_a(false).ok();
        motors.set_speed_a(55).ok();
        motors.set_speed_b(0).ok();
        timer.delay_ms(1200);

        info!("Motor A reverse at 55%");
        motors.set_invert_a(true).ok();
        motors.set_speed_a(55).ok();
        timer.delay_ms(1200);

        motors.set_speed_a(0).ok();
        timer.delay_ms(300);

        info!("Motor B forward at 55%");
        motors.set_invert_b(false).ok();
        motors.set_speed_b(55).ok();
        timer.delay_ms(1200);

        info!("Motor B reverse at 55%");
        motors.set_invert_b(true).ok();
        motors.set_speed_b(55).ok();
        timer.delay_ms(1200);

        info!("Stop both motors");
        motors.stop().ok();
        timer.delay_ms(1500);

        // ====================================================================
        // 2. DC Motor Frequency Sweep (Motors_Frequency.ino)
        // ====================================================================
        info!("--- Demo 2: DC Motor PWM Frequency Sweep ---");
        motors.set_stepper_mode_enabled(false).ok();
        motors.set_decay(DecayMode::Fast).ok();
        motors.set_invert_a(false).ok();
        motors.set_invert_b(false).ok();
        motors.set_speed_a(30).ok();
        motors.set_speed_b(30).ok();

        let frequencies = [400u16, 800, 1200, 2000, 8000, 20000];
        for &freq in &frequencies {
            info!("Set frequency: {} Hz", freq);
            motors.set_frequency(freq).ok();
            timer.delay_ms(1200);
        }

        motors.stop().ok();
        timer.delay_ms(1500);

        // Restore default frequency
        motors.set_frequency(20000).ok();

        // ====================================================================
        // 3. Stepper RPM Movement (Motors_Stepper_RPM.ino)
        // ====================================================================
        info!("--- Demo 3: Stepper Motor RPM Movement ---");
        motors.set_steps_per_revolution(200).ok();
        motors.set_stepper_mode_enabled(true).ok();
        motors.set_half_step_enabled(false).ok(); // Full-step mode (200 steps/rev)
        motors.set_decay(DecayMode::Fast).ok();

        run_stepper_move(
            &mut motors,
            &mut timer,
            "Forward: 1 rev at 60 RPM, hold at target",
            200,
            60.0,
            0,
        );
        timer.delay_ms(600);

        run_stepper_move(
            &mut motors,
            &mut timer,
            "Backward: 1 rev at 120 RPM, release after 50ms",
            -200,
            120.0,
            50,
        );
        timer.delay_ms(600);

        info!("Half-step mode: hold, then release");
        motors.set_half_step_enabled(true).ok(); // Effective steps/rev doubles to 400
        run_stepper_move(
            &mut motors,
            &mut timer,
            "Half-step forward: 1 rev at 90 RPM",
            400,
            90.0,
            0,
        );
        timer.delay_ms(600);

        run_stepper_move(
            &mut motors,
            &mut timer,
            "Half-step backward: 1 rev at 45 RPM",
            -400,
            45.0,
            50,
        );
        timer.delay_ms(600);

        motors.set_half_step_enabled(false).ok();
        timer.delay_ms(1200);

        // ====================================================================
        // 4. Telemetry and Current Sensing (Motors_Telemetry.ino)
        // ====================================================================
        info!("--- Demo 4: Telemetry & Current Sensing ---");
        motors.set_stepper_mode_enabled(false).ok();
        motors.set_decay(DecayMode::Fast).ok();
        motors.set_speed_a(50).ok();
        motors.set_speed_b(50).ok();

        motors.set_half_full_scale_enabled(false).ok();
        timer.delay_ms(200);
        print_telemetry(&mut motors, "Full-scale");

        motors.set_half_full_scale_enabled(true).ok();
        timer.delay_ms(200);
        print_telemetry(&mut motors, "Half-scale");

        motors.stop().ok();
        timer.delay_ms(300);
        print_telemetry(&mut motors, "After stop");

        motors.set_half_full_scale_enabled(false).ok();
        timer.delay_ms(2000);
    }
}
