# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A collection of standalone embedded Rust examples for the Raspberry Pi Pico 2 / Pico 2 W (RP2350, Cortex-M33). Nearly all real content lives in `examples/*.rs` — each file is a complete `#![no_std] #![no_main]` firmware image. `src/lib.rs` is a tiny shared utility crate (`Fmt` for 2-decimal `defmt` float formatting); `src/main.rs` is a clocks-only boot skeleton, not the product.

There are no unit/integration tests — hardware examples are verified by compiling and by flashing to a board.

## Commands

```bash
# Build / flash / stream defmt logs over SWD (probe-rs is the cargo runner)
cargo run --example <name>
cargo run --example <name> --release   # required for timing-sensitive examples (e.g. ov5640_ili9341_stream)

cargo build --example <name>
cargo check --example <name>
```

Full local CI mirror — all must pass before committing (see `AGENTS.md` and `.skills/definition-of-done/SKILL.md`):

```bash
cargo fmt --all -- --check && \
cargo clippy --workspace --examples --all-features -- -D warnings && \
cargo build --release
```

Pico 2 W examples (`blinky_pico2w`, `wifi_scan_pico2w`, `ble_scan_pico2w`) need CYW43439 firmware blobs first:

```bash
./download_firmware.sh   # writes 43439A0*.bin, nvram_rp2040.bin — gitignored (*.bin)
```

CI (`.github/workflows/rust_ci.yml`) runs fmt, `cargo clippy --examples -- -D warnings`, `cargo check --examples`, and a release build of the lib plus all examples, after running `download_firmware.sh`.

## Build/target setup

- Default target is `thumbv8m.main-none-eabihf` (hard-float Cortex-M33), set in `.cargo/config.toml`. Soft-float Arm and `riscv32imac-unknown-none-elf` (Hazard3) configs also exist there; RISC-V uses `rp235x_riscv.x` instead of `link.x`.
- `build.rs` copies `memory.x` (Arm) and `rp235x_riscv.x` (RISC-V) into `OUT_DIR` for the linker. `memory.x` defines the `.start_block` section that carries the boot-ROM image header.
- Runner is `probe-rs run --chip RP235x --protocol swd`; `Embed.toml` configures `cargo-embed`. `DEFMT_LOG=info` is set via `.cargo/config.toml` `[env]`.
- All example-only crates (drivers, embassy, embedded-graphics) live in `[dev-dependencies]`; only the HAL/runtime/defmt core is in `[dependencies]`.

## Two coexisting HAL styles

Pick based on hardware, not preference:

- **`rp235x-hal` (blocking, default)** — used by every non-wireless example. Boilerplate every such example repeats: `IMAGE_DEF` in `.start_block`, `XTAL_FREQ_HZ = 12_000_000`, `init_clocks_and_plls`, `Sio::new`, `gpio::Pins::new`, `Timer::new_timer0`, `#[hal::entry]`, and a `PICOTOOL_ENTRIES` array in `.bi_entries` for `picotool info`.
- **`embassy-rp` (async)** — only for Pico 2 W wireless examples, because the `cyw43` driver is async and needs PIO+DMA for its SPI link. These use `embassy_rp::block::ImageDef`, `bind_interrupts!`, `#[embassy_executor::main]`, a spawned `cyw43_task`, and embed firmware with `cyw43::aligned_bytes!("../43439A0.bin")`.

`defmt_rtt` + `panic_probe` are the logging/panic pair in both styles.

## Conventions for new examples

- Filename encodes the bus: `<chip>_i2c.rs`, `<chip>_spi.rs`, `<chip>_uart.rs`, `<chip>.rs` or `<chip>_adc.rs` for analog/digital. Existing files deviate; if unsure of a name, ask.
- Start from the closest existing example rather than from scratch — copy its boilerplate and `defmt` logging shape.
- Every example opens with a `//!` doc header: title, `## Hardware`, wiring table or ASCII pinout, and a `## Run` block with the `cargo run --example <name>` command.
- Default I2C is I2C0 on GPIO4 (SDA) / GPIO5 (SCL) — matches common Qwiic/STEMMA QT adapters. `examples/i2c_scan.rs` finds unknown addresses.
- Add a matching section to `README.md` (level-4 header, run command, wiring table) — the README is the user-facing catalog of every example.
- Use `Debug2Format(&e)` for driver errors that only implement `Debug`, and `Fmt(x)` from the lib crate for floats.

## Repo-specific skills

`.skills/` is the canonical location for task playbooks; `.claude/skills` and `.gemini/skills` are symlinks to it, so edit `.skills/` only. Worth reading before the matching work: `add-i2c-sensor-example`, `implement-sensor-example`, `microcontroller-example-conversion` (porting to/from ESP32-C3), `definition-of-done`, `update-changelog` (a global `CHANGELOG.md` lives in the *parent* directory, spanning sibling discovery repos).

`definition-of-done` also mandates: after checks pass, show `git status`/`git diff` and explicitly ask before committing or pushing; never stage `target/`, `.claude/`, `.skills/`, `.gemini/`, or `*.bin` firmware.

## E-paper examples

The `epdsi`-based examples (`ssd168*`, `ssd1677*`, `uc8253*`, `jd79661*`, `pdi_*`) drive
real panels, and panel state makes them behave unlike ordinary firmware.

**Power-cycle, run once, do not interrupt, then judge. Connect and disconnect FPCs with
the board unpowered.**

E-paper retains its last write, and the controller can be left latched busy by an
interrupted run or a hot-swapped panel. The *next* run then hits the driver's 60 s busy
timeouts and looks broken — shifted content, refreshes returning instantly, or refreshes
that appear to hang. `hard_reset` does not clear it; only removing power does. During
bring-up of the sibling C3 repository, nearly every apparent `epdsi` defect traced back
to this rather than to code.

Reference timings, so a deviation is recognisable as one:

| Panel | Full refresh | Partial refresh |
| :--- | ---: | ---: |
| GDEQ0426T82 4.26" mono | ~3.9 s | ~1.0 s |
| GDEM0213B74 2.13" mono | ~3.9 s | ~1.0 s |
| GDEM0154Z90 1.54" tri-colour | **~14 s** | **~14 s** |

Colour panels have no fast waveform — the pigment needs the full OTP waveform — so every
update takes seconds and the tri-colour example runs about 90 s in total. That is correct
behaviour, not a hang. Never select a `Partial` refresh mode on a colour panel: the fast
LUT exists only for monochrome, so it is slow *and* discards the colour plane.

### Debugging discipline

- **Suspect hardware and panel state before software.** Running stock Arduino GxEPD2
  across several boards has twice found in one experiment what hours of driver analysis
  did not.
- **Validate any new diagnostic against a known measurement.** A refresh reported as
  100 ms when the panel demonstrably takes 3891 ms means the instrument is broken.
- **Watch the panel, not the log.** A hand-rolled trigger sequence once reported entirely
  plausible timings while never driving the display at all.
- **Never reason from an interrupted run**, or from any run after one, until the board has
  been power-cycled.

Fuller notes, including known board/panel incompatibilities, live in the sibling
repository:
<https://github.com/melastmohican/xiao-esp32c3-blinky/blob/main/BRINGUP.md>

### Keeping the ports in step

These examples are the originals; `xiao-esp32c3-blinky` ports several of them to the
XIAO ESP32-C3. Those ports keep everything above `main()` byte-identical, with only board
bring-up differing — that correspondence demonstrates `epdsi`'s HAL independence. When
changing an example here, prefer edits that keep it portable.
