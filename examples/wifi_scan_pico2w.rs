//! # WiFi Scan Example for Raspberry Pi Pico 2 W
//!
//! This example scans for WiFi networks using the Raspberry Pi Pico 2 W (CYW43439).
//! It uses the Embassy framework with the `cyw43` driver.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2 W
//!
//! ## Why Embassy?
//!
//! The CYW43439 is a complex chip that communicates over a custom SPI interface.
//! Embassy is the preferred choice for this hardware because:
//! 1. **Non-blocking I/O**: Scanning for networks is a slow operation.
//! 2. **Hardware Abstraction**: Handles the low-level PIO/DMA complexity.
//! 3. **Networking Stack**: Optimized for the `cyw43` driver.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example wifi_scan_pico2w
//! ```

#![no_std]
#![no_main]

use core::str;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("WiFi Scan Example starting...");

    let p = embassy_rp::init(Default::default());

    // Use official firmware binaries downloaded from the Embassy repository.
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
    spawner.spawn(cyw43_task(runner).unwrap());

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    loop {
        info!("Scanning for networks...");
        let mut scanner = control.scan(Default::default()).await;
        while let Some(bss) = scanner.next().await {
            if let Ok(ssid_str) = str::from_utf8(&bss.ssid) {
                info!("scanned {} == {:x}", ssid_str, bss.bssid);
            }
        }
        info!("Scan finished. Waiting 5s...");
        embassy_time::Timer::after_secs(5).await;
    }
}
