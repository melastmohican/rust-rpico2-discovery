//! # Onboard LED Blinky Example for Raspberry Pi Pico 2 W
//!
//! This example blinks the onboard LED of the Raspberry Pi Pico 2 W.
//!
//! ### Why Embassy?
//! The Pico 2 W uses a CYW43439 wireless chip to control the onboard LED and provide WiFi/Bluetooth.
//! Embassy is used here because:
//! 1. **Async/Await**: The `cyw43` driver is fully asynchronous, allowing the CPU to perform other tasks
//!    (or sleep) while waiting for the wireless chip to respond.
//! 2. **DMA & PIO**: Embassy-rp provides seamless integration with DMA and PIO, which are required
//!    to implement the specialized SPI protocol used by the Pico W/2W.
//! 3. **Power Efficiency**: The Embassy executor automatically puts the RP2350 into a low-power
//!    state when no tasks are ready to run.
//! 4. **Concurrency**: It's easy to run wireless tasks alongside application logic without a heavy RTOS.
//!
//! Unlike the standard Pico 2, the onboard LED on the Pico 2 W is connected to the
//! CYW43439 wireless chip rather than a standard GPIO.
//!
//! This example uses the `embassy` asynchronous framework and the `cyw43` driver
//! to communicate with the wireless chip and toggle the LED.
//!
//! ## Run with:
//! `cargo run --example blinky_pico2w`

#![no_std]
#![no_main]

use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Use the official firmware binaries from the Embassy repository.
    // We use the aligned_bytes! macro to ensure the data is properly aligned in flash.
    let fw = cyw43::aligned_bytes!("../43439A0.bin");
    let clm = cyw43::aligned_bytes!("../43439A0_clm.bin");
    let nvram = cyw43::aligned_bytes!("../nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let dma = embassy_rp::dma::Channel::new(p.DMA_CH0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    // spawner.spawn(...) returns () in this version, while the task call returns a Result<SpawnToken>.
    spawner.spawn(cyw43_task(runner).unwrap());

    // Initialize the chip with the Country Locale Matrix (CLM)
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    defmt::info!("CYW43 initialized. Starting blinky loop...");

    defmt::info!("Pico 2 W Onboard Blinky starting...");

    loop {
        // WL_GPIO0 is the onboard LED on the CYW43439 chip
        control.gpio_set(0, true).await;
        defmt::info!("LED ON");
        Timer::after(Duration::from_millis(500)).await;

        control.gpio_set(0, false).await;
        defmt::info!("LED OFF");
        Timer::after(Duration::from_millis(500)).await;
    }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 5] = [
    embassy_rp::binary_info::rp_cargo_bin_name!(),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_description!(c"Blinky example for Pico 2 W onboard LED"),
    embassy_rp::binary_info::rp_cargo_homepage_url!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];
