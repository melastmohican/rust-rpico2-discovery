//! # PN532 NFC Reader: NTAG Dump and NDEF over I2C
//!
//! Detects an NTAG213/215/216 tag, dumps its pages and decodes the NDEF message stored
//! on it (URI or text record). Optionally writes a URI record to the tag.
//!
//! ## Hardware
//!
//! - **Board:** Raspberry Pi Pico 2
//! - **Module:** PN532 NFC/RFID module (Elechouse V3 or compatible) with a Qwiic connector
//! - **Tag:** NTAG213, NTAG215 or NTAG216 (Mifare Classic cards are not supported here)
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
//! cargo run --example pn532_ntag_i2c
//! ```
//!
//! Reading is the default. To write a URI record, set `WRITE_URI` to `true`. That
//! **overwrites the NDEF message on the tag**.
//!
//! ## Troubleshooting
//!
//! - **The module keeps its state across Pico resets.** After an interrupted run, power-cycle
//!   the module (unplug the Qwiic cable) before judging a result.
//! - **First I2C read hangs.** The PN532 is holding SDA low. The example clocks SCL up to
//!   9 times at startup to release it and logs `SDA stuck low` when it had to.
//! - **`BadAck`.** The stock `pn532::i2c::I2CInterface` splits status byte and frame into
//!   two transfers on this HAL. This example uses its own single-read `Pn532I2c`.
//! - **Not an NTAG.** The example checks SAK 0x00 and the capability container (page 3).

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
use pn532::requests::SAMMode;
use pn532::{CountDown, Interface, Pn532, Request};

/// Set to `true` to write a URI record to the tag. Overwrites the existing NDEF message.
const WRITE_URI: bool = false;

/// URI written when `WRITE_URI` is set: prefix code 0x02 means "https://www."
const URI_PREFIX_CODE: u8 = 0x02;
const URI_REST: &[u8] = b"rust-lang.org";

/// 7-bit I2C address of the PN532
const PN532_ADDRESS: u8 = 0x24;

/// First user-memory page of an NTAG
const FIRST_USER_PAGE: u8 = 4;

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

/// Reads 4 pages (16 bytes) starting at `page`. Returns `None` on any error.
fn read_pages<I2C: embedded_hal::i2c::I2c>(pn: &mut Pn<I2C>, page: u8) -> Option<[u8; 16]> {
    // InDataExchange response: [status, 16 data bytes]
    match pn.process(&Request::ntag_read(page), 17, Duration::from_millis(200)) {
        Ok(r) if r.len() >= 17 && r[0] == 0x00 => {
            let mut out = [0u8; 16];
            out.copy_from_slice(&r[1..17]);
            Some(out)
        }
        Ok(r) => {
            warn!(
                "Read page {} rejected, status 0x{=u8:02x}",
                page,
                r.first().copied().unwrap_or(0xFF)
            );
            None
        }
        Err(e) => {
            warn!("Read page {} failed: {:?}", page, defmt::Debug2Format(&e));
            None
        }
    }
}

/// Writes one 4-byte page. Returns `true` when the tag accepted it.
fn write_page<I2C: embedded_hal::i2c::I2c>(pn: &mut Pn<I2C>, page: u8, data: &[u8; 4]) -> bool {
    match pn.process(
        &Request::ntag_write(page, data),
        1,
        Duration::from_millis(300),
    ) {
        Ok(r) if r.first() == Some(&0x00) => true,
        Ok(r) => {
            warn!(
                "Write page {} rejected, status 0x{=u8:02x}",
                page,
                r.first().copied().unwrap_or(0xFF)
            );
            false
        }
        Err(e) => {
            warn!("Write page {} failed: {:?}", page, defmt::Debug2Format(&e));
            false
        }
    }
}

/// Tag name from the capability container size byte (page 3, byte 2 = size / 8).
fn ntag_name(cc_size: u8) -> &'static str {
    match cc_size {
        0x12 => "NTAG213 (144 bytes)",
        0x3E => "NTAG215 (496 bytes)",
        0x6D => "NTAG216 (872 bytes)",
        _ => "unknown NTAG size",
    }
}

/// URI abbreviation for the first byte of a URI record payload.
fn uri_prefix(code: u8) -> &'static str {
    match code {
        0x01 => "http://www.",
        0x02 => "https://www.",
        0x03 => "http://",
        0x04 => "https://",
        0x05 => "tel:",
        0x06 => "mailto:",
        _ => "",
    }
}

/// Decodes the NDEF records out of the tag's user memory and logs them.
///
/// Layout: TLV `03 <len> <records> FE`. Only short records (SR flag) with a one-byte
/// type and no ID are handled, which covers URI ('U') and well-known text ('T') records.
fn log_ndef(mem: &[u8]) {
    // Skip NULL TLVs (0x00) before the NDEF TLV
    let Some(start) = mem.iter().position(|&b| b != 0x00) else {
        info!("NDEF: tag is empty");
        return;
    };
    if mem[start] != 0x03 {
        info!("NDEF: no NDEF TLV (first TLV tag 0x{=u8:02x})", mem[start]);
        return;
    }
    let Some(&len) = mem.get(start + 1) else {
        return;
    };
    if len == 0 {
        info!("NDEF: message is empty");
        return;
    }
    if len == 0xFF {
        info!("NDEF: message longer than 254 bytes, not decoded");
        return;
    }
    let msg_end = (start + 2 + len as usize).min(mem.len());
    let mut rec = &mem[(start + 2).min(mem.len())..msg_end];
    if msg_end < start + 2 + len as usize {
        warn!("NDEF: message is longer than the buffer, later records are cut off");
    }

    // Walk the records until the message end flag (ME, bit 6) or the buffer runs out
    while rec.len() >= 4 {
        let header = rec[0];
        let short_record = header & 0x10 != 0;
        let has_id = header & 0x08 != 0;
        let type_len = rec[1] as usize;
        if !short_record || has_id || type_len != 1 {
            info!("NDEF: record not decoded (header 0x{=u8:02x})", header);
            return;
        }
        let payload_len = rec[2] as usize;
        let kind = rec[3];
        let end = (4 + payload_len).min(rec.len());
        let payload = &rec[4..end];
        match kind {
            b'U' if !payload.is_empty() => {
                let rest = core::str::from_utf8(&payload[1..]).unwrap_or("<not utf-8>");
                info!("NDEF URI: {}{}", uri_prefix(payload[0]), rest);
            }
            b'T' if !payload.is_empty() => {
                // First byte: bit 7 = UTF-16 flag, bits 0-5 = language code length
                let lang_len = (payload[0] & 0x3F) as usize;
                let text = payload
                    .get(1 + lang_len..)
                    .and_then(|t| core::str::from_utf8(t).ok())
                    .unwrap_or("<not utf-8>");
                info!("NDEF text: {}", text);
            }
            other => info!(
                "NDEF: record type 0x{=u8:02x}, {} bytes",
                other, payload_len as u32
            ),
        }
        if header & 0x40 != 0 {
            return;
        }
        rec = &rec[end..];
    }
}

/// Builds the NDEF TLV for `URI_PREFIX_CODE` + `URI_REST` into `buf` and returns its length.
fn build_uri_tlv(buf: &mut [u8; 32]) -> usize {
    let payload_len = 1 + URI_REST.len();
    // Record: header D1 (MB|ME|SR|TNF=1), type length 1, payload length, type 'U', payload
    let record_len = 4 + payload_len;
    buf[0] = 0x03;
    buf[1] = record_len as u8;
    buf[2] = 0xD1;
    buf[3] = 0x01;
    buf[4] = payload_len as u8;
    buf[5] = b'U';
    buf[6] = URI_PREFIX_CODE;
    buf[7..7 + URI_REST.len()].copy_from_slice(URI_REST);
    let end = 7 + URI_REST.len();
    buf[end] = 0xFE; // terminator TLV
    end + 1
}

#[hal::entry]
fn main() -> ! {
    info!("PN532 NTAG Example for RP2350");

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

    info!("Waiting for an NTAG... hold it against the antenna");

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
                let sak = r[4];
                let uid_len = (r[5] as usize).min(r.len() - 6).min(last_uid.len());
                let mut uid = [0u8; 10];
                uid[..uid_len].copy_from_slice(&r[6..6 + uid_len]);

                // Handle each tag once, not on every poll while it stays in the field
                if uid_len != last_len || uid[..uid_len] != last_uid[..last_len] {
                    last_uid = uid;
                    last_len = uid_len;
                    info!(
                        "Tag: UID {=[u8]:02x}, SAK 0x{=u8:02x}",
                        &uid[..uid_len],
                        sak
                    );
                    if sak != 0x00 {
                        warn!(
                            "SAK 0x{=u8:02x} is not an NTAG (expected 0x00), skipping",
                            sak
                        );
                    } else {
                        handle_ntag(&mut pn532, &mut delay);
                    }
                }
            }
            Ok(_) => {
                // No tag in the field
                last_len = 0;
            }
            Err(pn532::Error::TimeoutResponse) => {
                // By default the PN532 keeps polling until a tag shows up, so "no tag"
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

/// Identifies the tag, optionally writes the URI record, then dumps memory and decodes NDEF.
fn handle_ntag<I2C: embedded_hal::i2c::I2c>(pn532: &mut Pn<I2C>, delay: &mut impl DelayNs) {
    // Page 3 is the capability container: E1 <version> <size/8> <access>
    let Some(cc) = read_pages(pn532, 0) else {
        return;
    };
    let cc_page = &cc[12..16];
    if cc_page[0] != 0xE1 {
        warn!(
            "Capability container 0x{=u8:02x} is not 0xE1: tag is blank or not NDEF-formatted",
            cc_page[0]
        );
    }
    info!("Type: {}", ntag_name(cc_page[2]));
    let user_pages = (cc_page[2] as u16 * 8 / 4) as u8;

    if WRITE_URI {
        let mut tlv = [0u8; 32];
        let len = build_uri_tlv(&mut tlv);
        info!(
            "Writing URI https://www.{}",
            core::str::from_utf8(URI_REST).unwrap_or("")
        );
        for (i, chunk) in tlv[..len].chunks(4).enumerate() {
            let mut page = [0u8; 4];
            page[..chunk.len()].copy_from_slice(chunk);
            // The PN532 can miss the next command right after a write, so retry a few times
            let mut written = false;
            for _ in 0..3 {
                if write_page(pn532, FIRST_USER_PAGE + i as u8, &page) {
                    written = true;
                    break;
                }
                delay.delay_ms(20);
            }
            if !written {
                warn!("Giving up, the NDEF message on the tag is incomplete");
                return;
            }
            delay.delay_ms(10);
        }
        info!("Write done, reading back");
    }

    // Dump header pages 0-3, then the user area. NTAG reads return 4 pages at a time.
    let mut ndef_mem = [0u8; 64];
    let last_page = FIRST_USER_PAGE + user_pages;
    let mut page = 0u8;
    while page < last_page {
        let Some(data) = read_pages(pn532, page) else {
            return;
        };
        for (i, p) in data.chunks(4).enumerate() {
            let n = page + i as u8;
            if n < last_page {
                info!("page {=u8:02}: {=[u8]:02x}", n, p);
            }
        }
        if page >= FIRST_USER_PAGE {
            let off = ((page - FIRST_USER_PAGE) as usize) * 4;
            if off < ndef_mem.len() {
                let n = (ndef_mem.len() - off).min(16);
                ndef_mem[off..off + n].copy_from_slice(&data[..n]);
            }
        }
        page += 4;
    }

    log_ndef(&ndef_mem);
}
