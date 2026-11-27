//! # PN532 NFC Reader: Card UID over I2C
//!
//! Polls an ISO14443A card/tag (Mifare Classic, NTAG, ...) and prints its UID.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** PN532 NFC/RFID module (Elechouse V3 or compatible) with a Qwiic connector
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
//! The module has no IRQ or RSTO line connected, so readiness is polled over I2C.
//! Hold the card flat against the antenna loop at one end of the board.
//!
//! ## Run
//!
//! ```bash
//! cargo run --example pn532_uid_i2c
//! ```
//!
//! ## Troubleshooting
//!
//! - **The module keeps its state across Pico resets.** After an interrupted run, a poll
//!   can still be pending inside the PN532. Power-cycle the module (unplug the Qwiic cable)
//!   before judging a result. The first poll may log `TimeoutAck` once for the same reason.
//! - **First I2C read hangs, nothing logged after startup.** The PN532 is holding SDA low.
//!   The example clocks SCL up to 9 times at startup to release it and logs
//!   `SDA stuck low` when it had to.
//! - **`BadAck` on the first command.** The stock `pn532::i2c::I2CInterface` reads the
//!   status byte and the frame as two operations, which the rp235x HAL sends as two
//!   transfers. This example uses its own `Pn532I2c` with one contiguous read instead.
//! - **No firmware version at all.** Check the mode switches (ch1 ON, ch2 OFF) and
//!   power-cycle, because the mode is read only at power-up.

#![no_std]
#![no_main]

use core::convert::Infallible;
use core::time::Duration;

use defmt_rtt as _;
use panic_probe as _;

use defmt::*;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use core::task::Poll;
use embedded_hal::i2c::{Error as _, ErrorKind};
use hal::block::ImageDef;
use pn532::requests::SAMMode;
use pn532::{CountDown, Interface, Pn532, Request};

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

#[hal::entry]
fn main() -> ! {
    info!("PN532 UID Example for RP2350");

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

    let mut pn532: Pn532<_, _, 32> = Pn532::new(Pn532I2c { i2c, delay: timer1 }, pn_timer);

    // Let the module finish its power-up before the first command
    delay.delay_ms(100);

    match pn532.process(
        &Request::GET_FIRMWARE_VERSION,
        4,
        Duration::from_millis(100),
    ) {
        Ok(fw) => info!(
            "PN5{=u8:x} firmware {}.{}, support 0x{=u8:02x}",
            fw[0], fw[1], fw[2], fw[3]
        ),
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

    info!("Waiting for a card... hold it against the antenna");

    let mut last_uid = [0u8; 10];
    let mut last_len = 0usize;

    loop {
        // Response: [targets, tg, ATQA(2), SAK, uid_len, uid...]
        match pn532.process(
            &Request::INLIST_ONE_ISO_A_TARGET,
            17,
            Duration::from_millis(500),
        ) {
            Ok(r) if r.len() >= 6 && r[0] >= 1 => {
                let atqa = u16::from_be_bytes([r[2], r[3]]);
                let sak = r[4];
                let uid_len = (r[5] as usize).min(r.len() - 6).min(last_uid.len());
                let uid = &r[6..6 + uid_len];

                // Report each card once, not on every poll while it stays in the field
                if uid_len != last_len || uid != &last_uid[..last_len] {
                    info!(
                        "Card: UID {=[u8]:02x} ({} bytes), ATQA 0x{=u16:04x}, SAK 0x{=u8:02x}",
                        uid, uid_len as u32, atqa, sak
                    );
                    last_uid[..uid_len].copy_from_slice(uid);
                    last_len = uid_len;
                }
            }
            Ok(_) => {
                // No card in the field
                last_len = 0;
            }
            Err(pn532::Error::TimeoutResponse) => {
                // By default the PN532 keeps polling until a card shows up, so "no card"
                // arrives as a response timeout. Send the ACK frame to abort the pending
                // poll, otherwise the module stays busy and ignores the next command.
                if let Err(e) = pn532.abort() {
                    warn!("Abort failed: {:?}", defmt::Debug2Format(&e));
                }
                last_len = 0;
            }
            Err(e) => {
                warn!("Poll failed: {:?}", defmt::Debug2Format(&e));
                last_len = 0;
            }
        }

        delay.delay_ms(250);
    }
}
