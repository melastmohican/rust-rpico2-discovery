//! # PN532 NFC Reader: Mifare Classic Dump over I2C
//!
//! Detects a Mifare Classic card, finds a working key for every sector from a list of
//! well-known default keys, and dumps the readable blocks. Read-only: nothing is written.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** PN532 NFC/RFID module (Elechouse V3 or compatible) with a Qwiic connector
//! - **Card:** Mifare Classic 1K (SAK 0x08) or Mini (SAK 0x09). 4K cards (SAK 0x18) are
//!   dumped up to sector 15 only.
//! - **I2C Address:** 0x24
//!
//! Set the module's mode switches to I2C (channel 1 ON, channel 2 OFF) and
//! power-cycle the module, because the mode is read only at power-up.
//!
//! ## Wiring with Qwiic
//!
//! ```text
//!        Module -> RPi Pico 2
//! (black)  GND  -> GND
//! (red)    VCC  -> 3.3V
//! (yellow) SCL  -> GPIO5 (Pin 7) (I2C0 SCL)
//! (blue)   SDA  -> GPIO4 (Pin 6) (I2C0 SDA)
//! ```
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pn532_mifare_classic_i2c
//! ```
//!
//! ## How it works
//!
//! The `pn532` crate has no Mifare Classic API, so the example builds `InDataExchange`
//! requests by hand: `0x60` authenticates with key A, `0x30` reads a 16-byte block.
//! A failed authentication deselects the card, so the card is selected again before
//! every key attempt.
//!
//! The sector trailer (last block of each sector) is shown as read from the card: key A
//! always reads back as zeros, and key B is readable only if the access bits allow it.
//!
//! ## Troubleshooting
//!
//! - **The module keeps its state across Pico resets.** After an interrupted run, power-cycle
//!   the module (unplug the Qwiic cable) before judging a result.
//! - **First I2C read hangs.** The PN532 is holding SDA low. The example clocks SCL up to
//!   9 times at startup to release it and logs `SDA stuck low` when it had to.
//! - **`BadAck`.** The stock `pn532::i2c::I2CInterface` splits status byte and frame into
//!   two transfers on this HAL. This example uses its own single-read `Pn532I2c`.
//! - **`no default key`.** The sector uses a custom key. Add it to `KEYS`.

#![no_std]
#![no_main]

use core::convert::Infallible;
use core::task::Poll;
use core::time::Duration;

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{Error as _, ErrorKind};
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use hal::block::ImageDef;
use pn532::requests::{Command, SAMMode};
use pn532::{CountDown, Interface, Pn532, Request};

/// Well-known default keys, tried in order for every sector.
const KEYS: [[u8; 6]; 8] = [
    [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF], // factory default
    [0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5], // MAD key A
    [0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7], // NDEF key A
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5],
    [0x4D, 0x3A, 0x99, 0xC3, 0x51, 0xDD],
    [0x1A, 0x98, 0x2C, 0x7E, 0x45, 0x9A],
    [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
];

/// `InDataExchange` sub-commands for Mifare Classic
const MIFARE_AUTH_KEY_A: u8 = 0x60;
const MIFARE_READ: u8 = 0x30;

/// 7-bit I2C address of the PN532
const PN532_ADDRESS: u8 = 0x24;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

/// Adapts the HAL timer to the `pn532::CountDown` trait.
struct PnTimer {
    timer: Timer<hal::timer::CopyableTimer1>,
    deadline_us: u64,
}

impl CountDown for PnTimer {
    type Time = Duration;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        let d: Duration = count.into();
        self.deadline_us = self.timer.get_counter().ticks() + d.as_micros() as u64;
    }

    fn wait(&mut self) -> nb::Result<(), Infallible> {
        if self.timer.get_counter().ticks() >= self.deadline_us {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// PN532 I2C link with a single contiguous read per frame.
///
/// Every PN532 I2C read starts with a status byte (0x01 = ready) followed by the frame.
/// The stock `pn532::i2c::I2CInterface` issues the status byte and the frame as two
/// separate read operations, which some HALs send as two transfers. That makes the
/// second status byte show up inside the frame and the ACK fails to parse (`BadAck`).
/// Reading status + frame in one transfer and dropping the first byte avoids that.
struct Pn532I2c<I2C> {
    i2c: I2C,
    /// Used for a short pause before each command, see `write`
    delay: Timer<hal::timer::CopyableTimer1>,
}

impl<I2C: embedded_hal::i2c::I2c> Interface for Pn532I2c<I2C> {
    type Error = I2C::Error;

    fn write(&mut self, frame: &mut [u8]) -> Result<(), Self::Error> {
        // The module can miss a command that follows its previous response too closely,
        // which shows up as `TimeoutAck`. A short pause before each command avoids that.
        self.delay.delay_ms(5);
        self.i2c.write(PN532_ADDRESS, frame)
    }

    fn wait_ready(&mut self) -> Poll<Result<(), Self::Error>> {
        let mut status = [0u8];
        match self.i2c.read(PN532_ADDRESS, &mut status) {
            Ok(()) if status[0] == 0x01 => Poll::Ready(Ok(())),
            // Not ready, or the module did not ACK its address while busy
            Ok(()) => Poll::Pending,
            Err(e) => match e.kind() {
                ErrorKind::NoAcknowledge(_) => Poll::Pending,
                _ => Poll::Ready(Err(e)),
            },
        }
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        // Largest frame is bounded by the `Pn532<_, _, 32>` buffer, plus the status byte
        let mut raw = [0u8; 33];
        let raw = &mut raw[..buf.len() + 1];
        self.i2c.read(PN532_ADDRESS, raw)?;
        buf.copy_from_slice(&raw[1..]);
        Ok(())
    }
}

type Pn<I2C> = Pn532<Pn532I2c<I2C>, PnTimer, 32>;

/// A card found by `select_card`.
struct Card {
    uid: [u8; 10],
    uid_len: usize,
    sak: u8,
}

/// Polls once for an ISO14443A card and selects it as target 1.
///
/// Returns `None` when no card answered. A poll that times out is aborted, because the
/// PN532 otherwise keeps polling and ignores the next command.
fn select_card<I2C: embedded_hal::i2c::I2c>(pn: &mut Pn<I2C>, timeout_ms: u64) -> Option<Card> {
    // Response: [targets, tg, ATQA(2), SAK, uid_len, uid...]
    match pn.process(
        &Request::INLIST_ONE_ISO_A_TARGET,
        17,
        Duration::from_millis(timeout_ms),
    ) {
        Ok(r) if r.len() >= 6 && r[0] >= 1 => {
            let uid_len = (r[5] as usize).min(r.len() - 6).min(10);
            let mut uid = [0u8; 10];
            uid[..uid_len].copy_from_slice(&r[6..6 + uid_len]);
            Some(Card {
                uid,
                uid_len,
                sak: r[4],
            })
        }
        Ok(_) => None,
        Err(pn532::Error::TimeoutResponse) => {
            if let Err(e) = pn.abort() {
                warn!("Abort failed: {:?}", defmt::Debug2Format(&e));
            }
            None
        }
        Err(e) => {
            warn!("Poll failed: {:?}", defmt::Debug2Format(&e));
            None
        }
    }
}

/// Authenticates `block` with key A. Returns `true` on success.
///
/// Request data: `[target, 0x60, block, key(6), uid(4)]`. The PN532 wants exactly four
/// UID bytes, so a 7-byte UID is passed as its last four bytes.
fn authenticate<I2C: embedded_hal::i2c::I2c>(
    pn: &mut Pn<I2C>,
    block: u8,
    key: &[u8; 6],
    card: &Card,
) -> bool {
    let mut data = [0u8; 13];
    data[0] = 1;
    data[1] = MIFARE_AUTH_KEY_A;
    data[2] = block;
    data[3..9].copy_from_slice(key);
    data[9..13].copy_from_slice(&card.uid[card.uid_len.saturating_sub(4)..card.uid_len.max(4)]);
    // Response: [status]. 0x00 = authenticated, 0x14 = wrong key.
    matches!(
        pn.process(
            &Request::new(Command::InDataExchange, data),
            1,
            Duration::from_millis(200)
        ),
        Ok(r) if r.first() == Some(&0x00)
    )
}

/// Reads one 16-byte block. Requires a prior successful authentication of its sector.
fn read_block<I2C: embedded_hal::i2c::I2c>(pn: &mut Pn<I2C>, block: u8) -> Option<[u8; 16]> {
    // Response: [status, 16 data bytes]
    match pn.process(
        &Request::new(Command::InDataExchange, [1, MIFARE_READ, block]),
        17,
        Duration::from_millis(200),
    ) {
        Ok(r) if r.len() >= 17 && r[0] == 0x00 => {
            let mut out = [0u8; 16];
            out.copy_from_slice(&r[1..17]);
            Some(out)
        }
        Ok(r) => {
            warn!(
                "Read block {} rejected, status 0x{=u8:02x}",
                block,
                r.first().copied().unwrap_or(0xFF)
            );
            None
        }
        Err(e) => {
            warn!("Read block {} failed: {:?}", block, defmt::Debug2Format(&e));
            None
        }
    }
}

/// Number of sectors for the card's SAK. 4K cards are dumped up to sector 15 only.
fn sector_count(sak: u8) -> Option<u8> {
    match sak {
        0x09 => Some(5),         // Mini
        0x08 | 0x18 => Some(16), // 1K, and the first 16 sectors of a 4K
        _ => None,
    }
}

/// Finds a working key for each sector and logs its four blocks.
fn dump_card<I2C: embedded_hal::i2c::I2c>(pn: &mut Pn<I2C>, card: &Card, sectors: u8) {
    let mut readable = 0u8;
    // The card is already selected by the poll that found it. Only a failed
    // authentication deselects it, so select again after one, not before every attempt.
    let mut needs_select = false;
    for sector in 0..sectors {
        let first_block = sector * 4;
        let mut opened = false;
        for key in KEYS.iter() {
            if needs_select {
                if select_card(pn, 300).is_none() {
                    warn!("Card lost, remove and present it again");
                    return;
                }
                needs_select = false;
            }
            if !authenticate(pn, first_block, key, card) {
                needs_select = true;
                continue;
            }
            info!("Sector {=u8:02}: key A {=[u8]:02x}", sector, &key[..]);
            for block in first_block..first_block + 4 {
                match read_block(pn, block) {
                    Some(data) => info!("  block {=u8:02}: {=[u8]:02x}", block, data),
                    None => info!("  block {=u8:02}: <read failed>", block),
                }
            }
            opened = true;
            readable += 1;
            break;
        }
        if !opened {
            info!("Sector {=u8:02}: no default key", sector);
        }
    }
    info!("Dump done: {} of {} sectors readable", readable, sectors);
}

#[hal::entry]
fn main() -> ! {
    info!("PN532 Mifare Classic Example for RP2350");

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

    let mut delay = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    let timer1 = Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);
    let pn_timer = PnTimer {
        timer: timer1,
        deadline_us: 0,
    };

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Bus recovery. The PN532 can be left holding SDA low (for example after the Pico was
    // reset in the middle of a transaction). The first I2C read then blocks forever, so
    // clock SCL up to 9 times until the module releases SDA, then send a manual STOP.
    let mut sda_raw = pins.gpio4.into_pull_up_input();
    let mut scl_raw = pins.gpio5.into_pull_up_input();
    if !sda_raw.is_high().unwrap_or(true) {
        warn!("SDA stuck low, clocking SCL to free the bus");
        let mut scl_out = scl_raw.into_push_pull_output();
        for _ in 0..9 {
            let _ = scl_out.set_low();
            delay.delay_us(5);
            let _ = scl_out.set_high();
            delay.delay_us(5);
        }
        scl_raw = scl_out.into_pull_up_input();
    }
    let mut scl_out = scl_raw.into_push_pull_output();
    let mut sda_out = sda_raw.into_push_pull_output();
    let _ = scl_out.set_low();
    delay.delay_us(5);
    let _ = sda_out.set_low();
    delay.delay_us(5);
    let _ = scl_out.set_high();
    delay.delay_us(5);
    let _ = sda_out.set_high();
    delay.delay_us(5);

    // Configure I2C0 pins
    let sda_pin: Pin<_, FunctionI2C, _> = sda_out.into_pull_up_input().reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = scl_out.into_pull_up_input().reconfigure();

    // 100kHz: the PN532 stretches the clock, so stay at the standard speed
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut pn532: Pn<_> = Pn532::new(Pn532I2c { i2c, delay: timer1 }, pn_timer);

    // Let the module finish its power-up before the first command
    delay.delay_ms(100);

    match pn532.process(
        &Request::GET_FIRMWARE_VERSION,
        4,
        Duration::from_millis(100),
    ) {
        Ok(fw) => info!("PN5{=u8:x} firmware {}.{}", fw[0], fw[1], fw[2]),
        Err(e) => {
            error!(
                "No PN532 found (check I2C DIP switches and power-cycle): {:?}",
                defmt::Debug2Format(&e)
            );
            loop {
                cortex_m::asm::wfi();
            }
        }
    }

    // Normal mode: the SAM is not used, the PN532 acts as a plain reader
    if let Err(e) = pn532.process(
        &Request::sam_configuration(SAMMode::Normal, false),
        0,
        Duration::from_millis(100),
    ) {
        error!("SAM configuration failed: {:?}", defmt::Debug2Format(&e));
        loop {
            cortex_m::asm::wfi();
        }
    }

    info!("Waiting for a Mifare Classic card... hold it against the antenna");

    let mut last_uid = [0u8; 10];
    let mut last_len = 0usize;

    loop {
        match select_card(&mut pn532, 500) {
            Some(card) => {
                // Handle each card once, not on every poll while it stays in the field
                if card.uid_len != last_len || card.uid[..card.uid_len] != last_uid[..last_len] {
                    last_uid = card.uid;
                    last_len = card.uid_len;
                    info!(
                        "Card: UID {=[u8]:02x}, SAK 0x{=u8:02x}",
                        &card.uid[..card.uid_len],
                        card.sak
                    );
                    match sector_count(card.sak) {
                        Some(sectors) => dump_card(&mut pn532, &card, sectors),
                        None => warn!(
                            "SAK 0x{=u8:02x} is not a Mifare Classic (0x08, 0x09 or 0x18), skipping",
                            card.sak
                        ),
                    }
                }
            }
            None => last_len = 0,
        }

        delay.delay_ms(250);
    }
}
