//! This example scans for BLE devices using the RP Pico 2 W (CYW43439).
//! It uses the 'trouble' BLE stack and 'bt-hci' crate.
//!
//! ### Why Embassy?
//! Bluetooth LE on the Pico 2 W requires a Host/Controller architecture where the RP2350 (Host)
//! manages the BLE stack and communicates with the CYW43439 (Controller).
//! Embassy is critical here because:
//! 1. **Multiplexing**: The CYW43439 chip handles both WiFi and Bluetooth over the same SPI bus.
//!    Embassy allows the `cyw43` runner to multiplex these streams concurrently and efficiently.
//! 2. **Memory Safety**: The `trouble` BLE stack is a pure-Rust host implementation that
//!    leverages Embassy's async primitives for safe, zero-copy packet handling.
//! 3. **Resource Efficiency**: `trouble` is designed to run with fixed-size buffers and
//!    no heap, perfectly matching Embassy's philosophy for constrained microcontrollers.

#![no_std]
#![no_main]

use bt_hci::controller::ExternalController;
use bt_hci::param::{BdAddr, LeAdvReportsIter};
use core::cell::RefCell;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Timer};
use heapless::Deque;
use static_cell::StaticCell;
use trouble::prelude::*;
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
    info!("BLE Scan Example starting...");

    let p = embassy_rp::init(Default::default());

    // Use official firmware binaries downloaded from the Embassy repository.
    let fw = cyw43::aligned_bytes!("../43439A0.bin");
    let clm = cyw43::aligned_bytes!("../43439A0_clm.bin");
    let btfw = cyw43::aligned_bytes!("../43439A0_btfw.bin");
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

    // new_with_bluetooth returns (NetDriver, BtDriver, Control, Runner)
    let (_net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await;
    spawner.spawn(cyw43_task(runner).unwrap());

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // --- BLE Stack Setup ---
    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    // Using a fixed "random" address for testing
    let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);
    info!("Our address = {:?}", address);

    static RESOURCES: StaticCell<HostResources<DefaultPacketPool, 1, 1>> = StaticCell::new();
    let res = RESOURCES.init(HostResources::new());

    let stack = trouble::new(controller, res).set_random_address(address);
    let Host {
        central,
        mut runner,
        ..
    } = stack.build();

    let printer = Printer {
        seen: RefCell::new(Deque::new()),
    };
    let mut scanner = Scanner::new(central);

    info!("Starting BLE scan...");

    let _ = join(runner.run_with_handler(&printer), async {
        let config = ScanConfig {
            active: true,
            interval: Duration::from_millis(100),
            window: Duration::from_millis(100),
            ..Default::default()
        };

        // scan() returns a session that must be kept alive
        let mut _session = scanner.scan(&config).await.unwrap();

        loop {
            Timer::after(Duration::from_secs(10)).await;
            info!("Running...");
        }
    })
    .await;
}

struct Printer {
    seen: RefCell<Deque<BdAddr, 16>>,
}

impl EventHandler for Printer {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut seen = self.seen.borrow_mut();
        while let Some(Ok(report)) = it.next() {
            if seen.iter().find(|b| b.raw() == report.addr.raw()).is_none() {
                info!("Discovered: {:?} (RSSI: {})", report.addr, report.rssi);
                if seen.is_full() {
                    seen.pop_front();
                }
                seen.push_back(report.addr).ok();
            }
        }
    }
}
