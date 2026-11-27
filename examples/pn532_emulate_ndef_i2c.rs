//! # PN532 NFC Tag Emulation over I2C
//!
//! The PN532 pretends to be an NFC Forum Type 4 tag holding one NDEF URI record. A phone
//! that reads it opens `https://www.rust-lang.org`.
//!
//! **Experimental.** The tag activates and a phone sends its first SELECT over I2C, but the
//! I2C bus is unreliable while a reader's RF field is close, and a full NDEF read was not
//! seen end to end on the test hardware (Elechouse V3 clone). Stopping a run also leaves the
//! module hanging the next first transfer until it is power-cycled. For reliable emulation
//! use the module over HSU (UART) or SPI instead.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** PN532 NFC/RFID module (Elechouse V3 or compatible) with a Qwiic connector
//! - **Reader:** an NFC phone with an NDEF reader app (for example NFC Tools on iPhone)
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
//! cargo run --example pn532_emulate_ndef_i2c
//! ```
//!
//! Open the reader app first, then hold the phone against the antenna. On an iPhone, the
//! system Wallet prompt can open instead of the app when the tag looks like an ISO-DEP
//! target, so start the scan from NFC Tools (or the Control Center NFC tag reader).
//!
//! ## How it works
//!
//! `TgInitAsTarget` makes the PN532 answer as a passive ISO14443-4 target and handles the
//! link layer. The reader's APDUs then arrive through `TgGetData` and are answered with
//! `TgSetData`. The example implements the Type 4 tag read path: select the NDEF
//! application (AID `D2760000850101`), select and read the capability container (file
//! `E103`), select the NDEF file (`E104`) and read it. Writes are refused.
//!
//! ## Troubleshooting
//!
//! - **The module keeps its state across Pico resets.** After an interrupted run, power-cycle
//!   the module (unplug the Qwiic cable) before judging a result.
//! - **First I2C read hangs.** The PN532 is holding SDA low. The example clocks SCL up to
//!   9 times at startup to release it and logs `SDA stuck low` when it had to.
//! - **`BadAck`.** The stock `pn532::i2c::I2CInterface` splits status byte and frame into
//!   two transfers on this HAL. This example uses its own single-read `Pn532I2c`.
//! - **I2C `ArbitrationLoss` / `Other` while a reader is close.** The module can hold SDA
//!   low during RF activity. `Pn532I2c::recover` frees the pins, clocks SCL, sends a STOP
//!   and rebuilds the I2C block, then the example carries on.

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
use embedded_hal::i2c::{Error as _, ErrorKind, I2c as _};
use hal::I2C;
use hal::Sio;
use hal::Timer;
use hal::Watchdog;
use hal::clocks::Clock as _;
use hal::clocks::init_clocks_and_plls;
use hal::fugit::HertzU32;
use hal::fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin};
use hal::pac;
use rp235x_hal as hal;

use hal::block::ImageDef;
use pn532::requests::{Command, SAMMode};
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
struct Pn532I2c<'a> {
    i2c: Option<Bus>,
    /// Used for short pauses, see `write` and `recover`
    delay: Timer<hal::timer::CopyableTimer1>,
    /// Needed to rebuild the I2C block in `recover`
    resets: &'a mut pac::RESETS,
    system_clock_hz: HertzU32,
}

impl Pn532I2c<'_> {
    /// Release a bus the module is holding and restart the I2C block.
    ///
    /// While a reader's RF field is active the module can hold SDA low in the middle of a
    /// transfer. The controller then reports `ArbitrationLoss` on every later transfer, so
    /// take the pins back, clock SCL until SDA is released, send a STOP and rebuild the block.
    fn recover(&mut self) {
        let Some(i2c) = self.i2c.take() else {
            return;
        };
        let (block, (sda, scl)) = i2c.free(self.resets);
        let sda_in = sda.into_pull_up_input();
        let mut scl_out = scl.into_push_pull_output();
        for _ in 0..9 {
            let _ = scl_out.set_low();
            self.delay.delay_us(5);
            let _ = scl_out.set_high();
            self.delay.delay_us(5);
        }
        let mut sda_out = sda_in.into_push_pull_output();
        let _ = sda_out.set_low();
        self.delay.delay_us(5);
        let _ = sda_out.set_high();
        self.delay.delay_us(5);
        let sda: Pin<_, FunctionI2C, _> = sda_out.into_pull_up_input().reconfigure();
        let scl: Pin<_, FunctionI2C, _> = scl_out.into_pull_up_input().reconfigure();
        self.i2c = Some(I2C::new_controller(
            block,
            sda,
            scl,
            100.kHz(),
            self.resets,
            self.system_clock_hz,
        ));
    }

    fn bus(&mut self) -> &mut Bus {
        self.i2c.as_mut().unwrap()
    }
}

impl Interface for Pn532I2c<'_> {
    type Error = hal::i2c::Error;

    fn write(&mut self, frame: &mut [u8]) -> Result<(), Self::Error> {
        // The module can miss a command that follows its previous response too closely,
        // which shows up as `TimeoutAck`. A short pause before each command avoids that.
        self.delay.delay_ms(5);
        let r = self.bus().write(PN532_ADDRESS, frame);
        if let Err(e) = &r
            && matches!(e.kind(), ErrorKind::ArbitrationLoss | ErrorKind::Other)
        {
            warn!("write bus error, recovering");
            self.recover();
        }
        r
    }

    fn wait_ready(&mut self) -> Poll<Result<(), Self::Error>> {
        let mut status = [0u8];
        match self.bus().read(PN532_ADDRESS, &mut status) {
            Ok(()) if status[0] == 0x01 => Poll::Ready(Ok(())),
            // Not ready, or the module did not ACK its address while busy
            Ok(()) => Poll::Pending,
            Err(e) => match e.kind() {
                ErrorKind::NoAcknowledge(_) => Poll::Pending,
                // While a reader's RF field is active the module can glitch the bus.
                // Recover it and keep polling, the command timeout bounds the wait.
                ErrorKind::ArbitrationLoss | ErrorKind::Other => {
                    warn!("wait_ready bus error, recovering");
                    self.recover();
                    Poll::Pending
                }
                _ => Poll::Ready(Err(e)),
            },
        }
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        // Largest frame is bounded by the `Pn532<_, _, 64>` buffer, plus the status byte
        let mut raw = [0u8; 65];
        let raw = &mut raw[..buf.len() + 1];
        let r = self.bus().read(PN532_ADDRESS, raw);
        if let Err(e) = &r
            && matches!(e.kind(), ErrorKind::ArbitrationLoss | ErrorKind::Other)
        {
            warn!("read bus error, recovering");
            self.recover();
        }
        r?;
        buf.copy_from_slice(&raw[1..]);
        Ok(())
    }
}

/// I2C0 on GPIO4 (SDA) / GPIO5 (SCL)
type Bus = I2C<
    pac::I2C0,
    (
        Pin<hal::gpio::bank0::Gpio4, FunctionI2C, hal::gpio::PullUp>,
        Pin<hal::gpio::bank0::Gpio5, FunctionI2C, hal::gpio::PullUp>,
    ),
>;

type Pn<'a> = Pn532<Pn532I2c<'a>, PnTimer, 64>;
use pn532::requests::BorrowedRequest;

/// NDEF Tag Application AID
const NDEF_AID: [u8; 7] = [0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01];
const FILE_ID_CC: [u8; 2] = [0xE1, 0x03];
const FILE_ID_NDEF: [u8; 2] = [0xE1, 0x04];

/// Capability container: length 15, mapping version 2.0, max read/write APDU 48 bytes,
/// NDEF file control TLV (file E104, max size 64, read free, write denied).
const CAPABILITY_CONTAINER: [u8; 15] = [
    0x00, 0x0F, 0x20, 0x00, 0x30, 0x00, 0x30, 0x04, 0x06, 0xE1, 0x04, 0x00, 0x40, 0x00, 0xFF,
];

/// Maximum NDEF file size, including the two NLEN bytes
const NDEF_FILE_MAX: usize = 64;

/// URI served to the reader: prefix code 0x02 means "https://www."
const URI_PREFIX_CODE: u8 = 0x02;
const URI_REST: &[u8] = b"rust-lang.org";

/// Status words
const SW_OK: [u8; 2] = [0x90, 0x00];
const SW_FILE_NOT_FOUND: [u8; 2] = [0x6A, 0x82];
const SW_WRONG_OFFSET: [u8; 2] = [0x6B, 0x00];
const SW_NOT_ALLOWED: [u8; 2] = [0x69, 0x82];
const SW_INS_UNSUPPORTED: [u8; 2] = [0x6D, 0x00];

/// What the reader has selected so far in this session.
#[derive(Clone, Copy, PartialEq)]
enum Selected {
    Nothing,
    Application,
    CapabilityContainer,
    NdefFile,
}

/// Builds the NDEF file: `NLEN (2 bytes)` followed by one short URI record.
/// Returns the file and its used length.
fn build_ndef_file() -> ([u8; NDEF_FILE_MAX], usize) {
    let mut file = [0u8; NDEF_FILE_MAX];
    let payload_len = 1 + URI_REST.len();
    // Record: header D1 (MB|ME|SR|TNF=1), type length 1, payload length, type 'U', payload
    let message_len = 4 + payload_len;
    file[0] = 0x00;
    file[1] = message_len as u8;
    file[2] = 0xD1;
    file[3] = 0x01;
    file[4] = payload_len as u8;
    file[5] = b'U';
    file[6] = URI_PREFIX_CODE;
    file[7..7 + URI_REST.len()].copy_from_slice(URI_REST);
    (file, 2 + message_len)
}

/// Answers one APDU. Writes the response (data + status word) into `out` and returns its
/// length.
fn handle_apdu(
    apdu: &[u8],
    selected: &mut Selected,
    ndef_file: &[u8],
    out: &mut [u8; 64],
) -> usize {
    let status = |out: &mut [u8; 64], sw: [u8; 2]| {
        out[..2].copy_from_slice(&sw);
        2
    };
    if apdu.len() < 4 {
        return status(out, SW_INS_UNSUPPORTED);
    }
    let (ins, p1, p2) = (apdu[1], apdu[2], apdu[3]);
    match ins {
        // SELECT
        0xA4 => {
            let lc = apdu.get(4).copied().unwrap_or(0) as usize;
            let data = apdu.get(5..5 + lc).unwrap_or(&[]);
            match (p1, data) {
                (0x04, d) if d.len() == 7 && d[..6] == NDEF_AID[..6] && d[6] <= 0x01 => {
                    // Some readers (NFC Tools) first select the ...0100 variant, accept both
                    *selected = Selected::Application;
                    status(out, SW_OK)
                }
                (0x00, d) if *selected != Selected::Nothing && d == FILE_ID_CC => {
                    *selected = Selected::CapabilityContainer;
                    status(out, SW_OK)
                }
                (0x00, d) if *selected != Selected::Nothing && d == FILE_ID_NDEF => {
                    *selected = Selected::NdefFile;
                    status(out, SW_OK)
                }
                _ => status(out, SW_FILE_NOT_FOUND),
            }
        }
        // READ BINARY
        0xB0 => {
            let file: &[u8] = match *selected {
                Selected::CapabilityContainer => &CAPABILITY_CONTAINER,
                Selected::NdefFile => ndef_file,
                _ => return status(out, SW_FILE_NOT_FOUND),
            };
            let offset = ((p1 as usize) << 8) | p2 as usize;
            // Le of 0 would mean 256 bytes; the reader never needs more than MLe (48)
            let le = (apdu.get(4).copied().unwrap_or(0) as usize).clamp(1, 48);
            if offset > file.len() {
                return status(out, SW_WRONG_OFFSET);
            }
            let end = (offset + le).min(file.len());
            let n = end - offset;
            out[..n].copy_from_slice(&file[offset..end]);
            out[n..n + 2].copy_from_slice(&SW_OK);
            n + 2
        }
        // UPDATE BINARY: the tag is read-only
        0xD6 => status(out, SW_NOT_ALLOWED),
        _ => status(out, SW_INS_UNSUPPORTED),
    }
}

/// `TgInitAsTarget` parameters (UM0701 7.3.14): passive PICC-only target emulating an
/// ISO14443-4 (Type 4) tag: mode, SENS_RES, NFCID1t, SEL_RES, FeliCa params, NFCID3t,
/// no general bytes, no historical bytes.
fn tg_init_as_target_data() -> [u8; 37] {
    let mut d = [0u8; 37];
    d[0] = 0x05; // PICC only, passive only
    d[1] = 0x04; // SENS_RES (ATQA) 0x0004
    d[2] = 0x00;
    d[3] = 0x12; // NFCID1t, 3 bytes (the PN532 prepends 0x08)
    d[4] = 0x34;
    d[5] = 0x56;
    d[6] = 0x20; // SEL_RES (SAK): ISO14443-4 compliant
    // d[7..25]: FeliCa parameters, unused (zeros)
    // d[25..35]: NFCID3t, unused (zeros)
    // d[35]: length of general bytes = 0, d[36]: length of historical bytes = 0
    d
}

/// Serves APDUs until the reader leaves or an error occurs.
fn run_session(pn: &mut Pn<'_>, ndef_file: &[u8]) -> Result<(), ()> {
    let mut selected = Selected::Nothing;
    loop {
        // Response: [status, APDU bytes...]. A non-zero status means the reader is gone.
        let mut apdu = [0u8; 40];
        let apdu_len = match pn.process(
            &Request::new(Command::TgGetData, []),
            40,
            Duration::from_millis(3000),
        ) {
            Ok(r) if !r.is_empty() && r[0] == 0x00 => {
                let n = r.len() - 1;
                apdu[..n].copy_from_slice(&r[1..]);
                n
            }
            Ok(r) => {
                info!(
                    "Reader left (TgGetData status 0x{=u8:02x})",
                    r.first().copied().unwrap_or(0xFF)
                );
                return Ok(());
            }
            Err(pn532::Error::TimeoutResponse) => {
                let _ = pn.abort();
                info!("Reader timed out");
                return Ok(());
            }
            Err(e) => {
                warn!("TgGetData failed: {:?}", defmt::Debug2Format(&e));
                return Err(());
            }
        };
        let apdu = &apdu[..apdu_len];
        info!("APDU  {=[u8]:02x}", apdu);

        let mut out = [0u8; 64];
        let out_len = handle_apdu(apdu, &mut selected, ndef_file, &mut out);
        info!("reply {=[u8]:02x}", &out[..out_len]);

        match pn.process(
            BorrowedRequest::new(Command::TgSetData, &out[..out_len]),
            1,
            Duration::from_millis(500),
        ) {
            Ok(r) if r.first() == Some(&0x00) => {}
            Ok(r) => {
                warn!(
                    "TgSetData status 0x{=u8:02x}",
                    r.first().copied().unwrap_or(0xFF)
                );
                return Ok(());
            }
            Err(e) => {
                warn!("TgSetData failed: {:?}", defmt::Debug2Format(&e));
                return Err(());
            }
        }
    }
}

#[hal::entry]
fn main() -> ! {
    info!("PN532 NDEF tag emulation for RP2350");
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

    let mut pn532: Pn<'_> = Pn532::new(
        Pn532I2c {
            i2c: Some(i2c),
            delay: timer1,
            resets: &mut pac.RESETS,
            system_clock_hz: clocks.system_clock.freq(),
        },
        pn_timer,
    );

    // Let the module finish its power-up before the first command
    delay.delay_ms(100);

    // A previous run may have left TgInitAsTarget pending inside the module. An ACK frame
    // cancels it, so no power-cycle is needed between runs. Errors are expected when idle.
    let _ = pn532.abort();
    delay.delay_ms(50);

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

    let (ndef_file, ndef_len) = build_ndef_file();
    let ndef_file = &ndef_file[..ndef_len];
    info!("Emulating a Type 4 tag with https://www.rust-lang.org, waiting for a reader");

    loop {
        let request = Request::new(Command::TgInitAsTarget, tg_init_as_target_data());
        match pn532.process(&request, 20, Duration::from_millis(10_000)) {
            Ok(r) => {
                info!(
                    "Reader activated the target (mode 0x{=u8:02x})",
                    r.first().copied().unwrap_or(0)
                );
                let _ = run_session(&mut pn532, ndef_file);
                // Let the bus and the module settle before starting a new round
                delay.delay_ms(500);
            }
            Err(pn532::Error::TimeoutResponse) => {
                // No reader within 10 s. The target command is still pending inside the
                // module, so cancel it and start a new round.
                if let Err(e) = pn532.abort() {
                    warn!("Abort failed: {:?}", defmt::Debug2Format(&e));
                }
            }
            Err(e) => {
                warn!("TgInitAsTarget failed: {:?}", defmt::Debug2Format(&e));
                delay.delay_ms(200);
            }
        }
    }
}
