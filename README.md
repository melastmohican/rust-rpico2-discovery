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

### ADC Examples

#### grove_light_sensor_adc

Reads an analog value from a Grove Light Sensor v1.2.

```bash
cargo run --example grove_light_sensor_adc
```

**Wiring:**

The Grove Light Sensor is an analog sensor and must be connected to an ADC pin.
On the Raspberry Pi Pico 2, ADC pins are GPIO26, GPIO27, and GPIO28.

```
          Raspberry Pi Pico 2
        +--------------------------+
        |                          |
        | [ ] 1   40 [ ] USB       |
        | [ ] 2   39 [ ]           |
        | [ ] 3   38 [G]ND --------+ (black)
        | [ ] 4   37 [ ]           |
        | [ ] 5   36 [3]V3(OUT) ---+ (red)
        | [ ]...  ...[ ]           |
        | [ ]...  ...[ ]           |
        | [ ] 30  31 [A]GPIO26 ----+ (yellow)
        | [ ] 29  32 [ ]           |
        | [ ]...  ...[ ]           |
        | [ ]...  ...[ ]           |
        +--------------------------+
               |   |   |
               |   |   |
        +------|---|---|------------+
        |      G   V   S           |
        |      N   C   I           |
        |      D   C   G           |
        |                          |
        |  Grove Light Sensor v1.2 |
        +--------------------------+
```

**Connections:**

- **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38 is a convenient choice).
- **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
- **SIG (yellow wire):** Connects to an ADC-capable pin. In the example code, this is `GPIO26` (Pin 31).

#### grove_moisture_sensor_adc

Reads an analog moisture sensor value and prints it to the console with an interpretation.

```bash
cargo run --example grove_moisture_sensor_adc
```

**Wiring:**

The Grove Moisture Sensor is an analog sensor and must be connected to an ADC pin.
On the Raspberry Pi Pico 2, ADC pins are GPIO26, GPIO27, and GPIO28.

```
          Raspberry Pi Pico 2
        +--------------------------+
        |                          |
        | [ ] 1   40 [ ] USB       |
        | [ ] 2   39 [ ]           |
        | [ ] 3   38 [G]ND --------+ (black)
        | [ ] 4   37 [ ]           |
        | [ ] 5   36 [3]V3(OUT) ---+ (red)
        | [ ]...  ...[ ]           |
        | [ ]...  ...[ ]           |
        | [ ] 30  31 [ ]           |
        | [ ] 29  32 [A]GPIO27 ----+ (yellow)
        | [ ]...  ...[ ]           |
        | [ ]...  ...[ ]           |
        +--------------------------+
               |   |   |
               |   |   |
        +------|---|---|------------+
        |      G   V   S           |
        |      N   C   I           |
        |      D   C   G           |
        |                          |
        |   Grove Moisture Sensor  |
        +--------------------------+
```

**Connections:**

- **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38 is a convenient choice).
- **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
- **SIG (yellow wire):** Connects to an ADC-capable pin. In this example, this is `GPIO27` (Pin 32, ADC1).

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

#### vl53l4cd_i2c

Reads distance from a VL53L4CD Time-of-Flight Distance Sensor using a blocking driver. This example is configured for devices on `I2C0` with a fixed address of `0x29`.

```bash
cargo run --example vl53l4cd_i2c
```

**Wiring:**

```
VL53L4CD Sensor -> RPi Pico 2
---------------    --------------
GND (black)     -> GND
VCC (red)       -> 3.3V
SCL (yellow)    -> GPIO5 (Pin 7)
SDA (blue)      -> GPIO4 (Pin 6)
```

**Resources:**

- [Arduino Modulino Distance documentation](https://docs.arduino.cc/hardware/modulino-distance/)

#### dht20_i2c

Reads temperature and humidity from a DHT20 sensor. This example is configured for devices on `I2C0` with an address of `0x38`.

```bash
cargo run --example dht20_i2c
```

**Wiring:**

```
DHT20 Pin -> RPi Pico 2
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (blue)  -> GPIO4 (Pin 6)
```

#### lis3dh_i2c

Reads accelerometer data from a LIS3DH sensor. This example is configured for devices on `I2C0` with an address of `0x19`.

```bash
cargo run --example lis3dh_i2c
```

**Wiring:**

```
LIS3DH Pin -> RPi Pico 2
----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (blue)  -> GPIO4 (Pin 6)
```

### Display Examples

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

#### ssd1315

Displays a Rust logo image on a 128x64 SSD1315 OLED screen. This example is configured to use `I2C0`.

```bash
cargo run --example ssd1315
```

#### ssd1315_text

Demonstrates text rendering and drawing shapes on the OLED display.

```bash
cargo run --example ssd1315_text
```

**Wiring for SSD1315 Display:**

```
Display Pin -> RPi Pico 2
-----------    --------------
GND (black) -> GND
VCC (red)   -> 3.3V
SCL (yellow)-> GPIO5 (Pin 7)
SDA (green) -> GPIO4 (Pin 6)
```

#### gc9a01_spi

Displays images (Ferris and Rust logo) on a 240x240 round GC9A01 LCD display over SPI.

```bash
cargo run --example gc9a01_spi
```

#### gc9a01_spi_text

Demonstrates text rendering and drawing shapes on the GC9A01 round LCD display.

```bash
cargo run --example gc9a01_spi_text
```

**Wiring for GC9A01 Display (7-pin modules):**

```
          Raspberry Pi Pico 2
        +--------------------------+
        |                          |
        | [ ] 1   40 [ ] USB       |
        | [ ] 2   39 [ ]           |
        | [ ] 3   38 [G]ND --------+ (GND - black)
        | [ ] 4   37 [ ]           |
        | [ ] 5   36 [3]V3(OUT) ---+ (VCC - red)
        | [ ]...  ...[ ]           |
        | [ ]...  ...[ ]           |
        | [ ]... 27 [G]PIO21 ------+ (RST - purple)
        | [ ]... 26 [G]PIO20 ------+ (DC - gray)
        | [ ]... 25 [G]PIO19 ------+ (SDA/MOSI - yellow)
        | [ ]... 24 [G]PIO18 ------+ (SCL/SCK - orange)
        | [ ]...  ...[ ]           |
        | [ ]... 22 [G]PIO17 ------+ (CS - green)
        | [ ]...  ...[ ]           |
        +--------------------------+
               |   |   |   |   |   |   |
               |   |   |   |   |   |   |
        +------|---|---|---|---|---|---|-------+
        |      G   V   S   S   D   C   R      |
        |      N   C   C   D   C   S   S      |
        |      D   C   L   A           T      |
        |                                     |
        |  GC9A01 240x240 Round LCD Display   |
        +------------------------------------- +
```

**Connections:**

- **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38).
- **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
- **SCL (orange wire):** Connects to `GPIO18` (Pin 24) - SPI0 Clock.
- **SDA (yellow wire):** Connects to `GPIO19` (Pin 25) - SPI0 TX (MOSI).
- **DC (gray wire):** Connects to `GPIO20` (Pin 26) - Data/Command select.
- **CS (green wire):** Connects to `GPIO17` (Pin 22) - Chip Select.
- **RST (purple wire):** Connects to `GPIO21` (Pin 27) - Reset.

**Note:** Despite having pins labeled SCL/SDA, this is an SPI display (not I2C). The DC and CS pins confirm it's SPI - SCL=clock, SDA=MOSI.

#### st7735s_spi

Displays images (Ferris and Rust logo) on an 80x160 ST7735S LCD display (Waveshare 0.96 inch module) over SPI.

```bash
cargo run --example st7735s_spi
```

#### st7735s_spi_text

Demonstrates text rendering and drawing shapes on the ST7735S LCD display.

```bash
cargo run --example st7735s_spi_text
```

**Wiring for ST7735S Display (Waveshare 0.96 inch):**

```
          Raspberry Pi Pico 2
        +--------------------------+
        |                          |
        | [ ] 1   40 [ ] USB       |
        | [ ] 2   39 [ ]           |
        | [ ] 3   38 [G]ND --------+ (GND - black)
        | [ ] 4   37 [ ]           |
        | [ ] 5   36 [3]V3(OUT) ---+ (VCC - red)
        | [ ]...  ...[ ]           |
        | [ ]...  ...[ ]           |
        | [ ]... 27 [G]PIO21 ------+ (RST - purple)
        | [ ]... 26 [G]PIO20 ------+ (DC - gray)
        | [ ]... 25 [G]PIO19 ------+ (DIN/MOSI - yellow)
        | [ ]... 24 [G]PIO18 ------+ (CLK/SCK - orange)
        | [ ]...  ...[ ]           |
        | [ ]... 22 [G]PIO17 ------+ (CS - green)
        | [ ]...  ...[ ]           |
        | [ ]... 19 [G]PIO14 ------+ (BL - white, optional)
        | [ ]...  ...[ ]           |
        +--------------------------+
               |   |   |   |   |   |   |   |
               |   |   |   |   |   |   |   |
        +------|---|---|---|---|---|---|---|------+
        |      G   V   C   D   D   R   B         |
        |      N   C   L   I   C   S   L         |
        |      D   C   K   N       T             |
        |                                        |
        |  Waveshare 0.96" ST7735S LCD (80x160)  |
        +----------------------------------------+
```

**Connections:**

- **GND (black wire):** Connects to any `GND` pin on the Pico 2 (Pin 38).
- **VCC (red wire):** Connects to the `3V3(OUT)` pin on the Pico 2 (Pin 36).
- **CLK (orange wire):** Connects to `GPIO18` (Pin 24) - SPI0 Clock.
- **DIN (yellow wire):** Connects to `GPIO19` (Pin 25) - SPI0 TX (MOSI).
- **DC (gray wire):** Connects to `GPIO20` (Pin 26) - Data/Command select.
- **CS (green wire):** Connects to `GPIO17` (Pin 22) - Chip Select.
- **RST (purple wire):** Connects to `GPIO21` (Pin 27) - Reset.
- **BL (white wire):** Connects to `GPIO14` (Pin 19) - Backlight (optional, can connect to 3.3V).

#### epdk

 Displays images (Ferris and Rust), text, and shapes on a Pervasive Displays E-Paper Display Pico Kit (EPDK) using the EXT3-1 board.

 ```bash
 cargo run --example epdk
 ```

 **Wiring for EPDK (EXT3-1) with 10-way rainbow cable:**

 | Pico Pin       | Cable Color | EXT3 Pin / Function |
 |----------------|-------------|---------------------|
 | 3V3 (Pin 36)   | **Black**   | 1 / VCC             |
 | GPIO18 (Pin 24)| **Brown**   | 2 / SCK (SPI Clock) |
 | GPIO13 (Pin 17)| **Red**     | 3 / BUSY            |
 | GPIO12 (Pin 16)| **Orange**  | 4 / DC (Data/Cmd)   |
 | GPIO11 (Pin 15)| **Yellow**  | 5 / RST (Reset)     |
 | GPIO16 (Pin 21)| **Green**   | 6 / MISO            |
 | GPIO19 (Pin 25)| **Blue**    | 7 / MOSI            |
 | NC             | **Violet**  | 8 / FCSM (Flash CS) |
 | GPIO17 (Pin 22)| **Grey**    | 9 / ECSM (Display CS)|
 | GND (Pin 38)   | **White**   | 10 / GND            |

 **Note:** On the EXT3-1 board, Pin 9 (ECSM) is the Chip Select for the E-Paper Display. Pin 8 (FCSM) connects to the onboard Flash and is left unconnected in this example.

#### gdem0154z90

Displays a tri-color image (`rust.bmp`) on the Dalian Good Display 1.54" E-Ink Display.

```bash
cargo run --example gdem0154z90
```

#### gdem0154z90_text

Displays text, shapes, and rotation demos on the Dalian Good Display 1.54" E-Ink Display.

```bash
cargo run --example gdem0154z90_text
```

**Wiring for DESPI-C02 Adapter (8-pin):**

| DESPI-C02 Pin | Pico Pin       | Function     |
|---------------|----------------|--------------|
| **BUSY**      | GPIO13 (Pin 17)| Busy Signal  |
| **RES**       | GPIO11 (Pin 15)| Reset        |
| **D/C**       | GPIO12 (Pin 16)| Data/Command |
| **CS**        | GPIO17 (Pin 22)| Chip Select  |
| **SCK**       | GPIO18 (Pin 24)| SPI Clock    |
| **SDI**       | GPIO19 (Pin 25)| SPI MOSI     |
| **GND**       | GND (Pin 38)   | Ground       |
| **3.3V**      | 3V3 (Pin 36)   | Power        |

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
