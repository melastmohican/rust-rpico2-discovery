# Rust RPico2 Discovery

This repository contains examples for the Raspberry Pi Pico 2 board, written in Rust.

## Project generated

```shell
cargo generate --git https://github.com/ImplFerris/pico2-template.git --name rust-rpico2-discovery
```

## Hardware

**Board:** Raspberry Pi Pico 2

- **MCU:** RP2350 (Dual-core Arm Cortex-M33 and RISC-V cores)
- **On-board peripherals:**
  - LED on GPIO25
- **I2C pins:**
  - **I2C0 SDA:** GPIO4
  - **I2C0 SCL:** GPIO5
  - **I2C1 SDA:** GPIO2
  - **I2C1 SCL:** GPIO3

## Examples

### Basic Examples

#### blinky

Blinks the on-board LED on GPIO25 to verify your setup is working.

```bash
cargo run --example blinky
```

### I2C Examples

#### i2c_scan

Scans both I2C buses (`I2C0` and `I2C1`) for connected devices and prints their addresses. This is useful for debugging I2C connections and discovering device addresses.

```bash
cargo run --example i2c_scan
```

**Wiring:**

You can connect I2C devices to either or both of the default I2C ports.

- **I2C0:**
  - SDA: GPIO4 (Pin 6)
  - SCL: GPIO5 (Pin 7)
- **I2C1:**
  - SDA: GPIO2 (Pin 4)
  - SCL: GPIO3 (Pin 5)


#### bme280_i2c

Reads temperature, humidity, and atmospheric pressure from an external BME280 sensor. This example is configured for devices on `I2C0` with an address of `0x77`.

```bash
cargo run --example bme280_i2c
```

**Wiring:**

```
BME280 Pin -> RPi Pico 2
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (blue)  -> GPIO4 (Pin 6)
```

**I2C Address:**

- **0x77**: Used by default in this example (`BME280::new_secondary()`). Common on Adafruit BME280 boards.
- **0x76**: Can be used by changing the code to `BME280::new_primary()`.




#### bmp280_i2c

Reads temperature and atmospheric pressure from an external BMP280 sensor. This example is configured for devices on `I2C0` with an address of `0x77`.

```bash
cargo run --example bmp280_i2c
```

**Wiring:**

```
BMP280 Pin -> RPi Pico 2
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (blue)  -> GPIO4 (Pin 6)
```

**I2C Address:**

- **0x77**: Used by default in this example (`BMP280::new_secondary()`). Common on Adafruit BMP280 boards.
- **0x76**: Can be used by changing the code to `BMP280::new_primary()`.




#### hs3003_i2c

Reads temperature and humidity from an HS3003 sensor using a custom driver on the Raspberry Pi Pico 2. This example is configured for devices on `I2C0` with a fixed address of `0x44`.

```bash
cargo run --example hs3003_i2c
```

**Wiring:**

```
HS3003 Pin -> RPi Pico 2
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7) (I2C0 SCL)
SDA (blue)  -> GPIO4 (Pin 6) (I2C0 SDA)
```

#### bh1750_i2c

Reads ambient light levels in lux from an external BH1750 sensor. This example is configured for devices on `I2C0` with an address of `0x23`.

```bash
cargo run --example bh1750_i2c
```

**Wiring:**

```
BH1750 Pin -> RPi Pico 2
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (blue)  -> GPIO4 (Pin 6)
```

**I2C Address:**

- **0x23**: (ADDR pin to GND) - Used by default in this example.
- **0x5C**: (ADDR pin to VCC) - Can be used by changing `Address::Low` to `Address::High` in the code.

#### modulino_pixels_i2c

Controls the 8 RGB LEDs on an Arduino Modulino Pixels module, demonstrating various color animations. This example is configured for devices on `I2C0` with an address of `0x36`.

```bash
cargo run --example modulino_pixels_i2c
```

**Wiring:**

```
Modulino Pin -> RPi Pico 2
------------    --------------
GND (black)  -> GND
VCC (red)    -> 3.3V
SCL (yellow) -> GPIO5 (Pin 7)
SDA (blue)   -> GPIO4 (Pin 6)
```

### Display Examples (SSD1306 OLED - I2C)

#### ssd1306

Displays a Rust logo image on a 128x64 SSD1306 OLED screen. This example is configured to use `I2C0`.

```bash
cargo run --example ssd1306
```


#### ssd1306_text

Demonstrates text rendering and drawing shapes on the OLED display.

```bash
cargo run --example ssd1306_text
```

**Wiring for SSD1306 Display:**


```
Display Pin -> RPi Pico 2
-----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (green) -> GPIO4 (Pin 6)
```


## Resources

- [Raspberry Pi Pico 2 Pinout](https://pico2.pinout.xyz/)

## Building and Flashing

To build and flash any example, ensure your board is connected and run:

```bash
# Build only
cargo build --example <example_name>

# Build, flash, and view logs
cargo run --example <example_name>
```
The `.cargo/config.toml` is configured to use `probe-rs` as the default runner, which will handle flashing and starting the application. For examples with `defmt` logging, the output will appear in your terminal.