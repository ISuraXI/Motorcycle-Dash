# Motorrad-Dash

A custom motorcycle dashboard built on the **ESP32-S3 N16R8** (16 MB Flash, 8 MB OPI PSRAM), developed with PlatformIO and the Arduino framework. It displays real-time riding data on a 1.51" transparent OLED and integrates multiple sensors, a RaceBox Mini GPS logger, and a Blitzer-Warner radar detector.

---

## Features

| Feature | Details |
|---|---|
| **Oil temperature** | NTC resistor read via ADS1115 (CH0), EMA-smoothed |
| **Lean angle** | BNO085 IMU over I2C, all-time max stored in EEPROM |
| **G-force** | BNO085 linear acceleration, all-time max stored in EEPROM |
| **Battery voltage** | 33 kΩ / 10 kΩ divider on ADS1115 CH2, calibration offset |
| **Outside temperature** | DS18B20 1-Wire sensor, calibration offset |
| **Ambient light / Night mode** | BH1750 lux meter → auto-adjusts OLED contrast & invert |
| **RaceBox Mini** | GPS-fix, BLE-connected and recording-status status via GPIO inputs |
| **Blitzer-Warner** | Digital alert pin + heartbeat LED voltage on ADS1115 CH1 |
| **Auto-brightness** | Smooth contrast fade between day (0xFF) and night (0x35) |

---

## Hardware

### Microcontroller
- **ESP32-S3 DevKitC-1** (N16R8 variant — 16 MB Flash QIO, 8 MB PSRAM OPI)

### GPIO Assignment

| Peripheral | Signal | GPIO |
|---|---|---|
| I2C (shared bus) | SDA | 8 |
| I2C (shared bus) | SCL | 9 |
| OLED SSD1309 (SPI2) | MOSI / DIN | 11 |
| OLED SSD1309 (SPI2) | CLK | 12 |
| OLED SSD1309 (SPI2) | CS | 10 |
| OLED SSD1309 (SPI2) | DC | 5 |
| OLED SSD1309 (SPI2) | RST | 6 |
| DS18B20 (1-Wire) | DATA | 7 |
| Navigation button | INPUT_PULLUP → GND | 4 |
| Blitzer-Warner | Digital alert (INPUT_PULLUP) | 14 |
| RaceBox Mini | GPS fix | 15 |
| RaceBox Mini | BLE connected | 16 |
| RaceBox Mini | Recording | 17 |
| RaceBox Mini | Simulated button (NPN) | 18 |

### I2C Devices

| Device | Address | Purpose |
|---|---|---|
| BNO085 | 0x4A | IMU — lean angle & G-force |
| BH1750 | 0x23 | Ambient light sensor |
| ADS1115 | 0x48 | 4-channel 16-bit ADC |

### ADS1115 Channel Map

| Channel | Signal |
|---|---|
| AIN0 | Oil temperature NTC |
| AIN1 | Blitzer-Warner heartbeat LED (~2.6 V idle / ~3.5 V pulse) |
| AIN2 | Battery voltage divider (33 kΩ / 10 kΩ) |
| AIN3 | Free |

---

## Display Pages

Short button press cycles through pages: **OIL → LEAN → G → RACEBOX → OIL …**

| Page | Content | Long-press action |
|---|---|---|
| OIL | Oil temperature (°C), outside temp, battery voltage | — |
| LEAN | Current & all-time max lean angle | Reset all-time max (EEPROM) |
| G | Current & all-time max G-force | Reset all-time max (EEPROM) |
| RACEBOX | GPS fix / BLE / recording status, auto-start recording | — |

### Boot sequence
1. OLED turns on immediately.
2. Each sensor is initialised and shows **OK** or **FAIL** as it comes up.
3. 2-second hold window: hold the button during this window to enter **calibration mode** (CAL icon top-right).

---

## Software

### Dependencies (managed by PlatformIO)

```
adafruit/Adafruit GFX Library
adafruit/Adafruit SSD1306
adafruit/Adafruit BNO08x
claws/BH1750
adafruit/Adafruit ADS1X15
milesburton/DallasTemperature
paulstoffregen/OneWire
```

### Build & Upload

```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor
```

### PlatformIO environment

```ini
[env:esp32-s3-n16r8]
platform    = espressif32
board       = esp32-s3-devkitc-1
framework   = arduino
monitor_speed = 115200

board_build.flash_size          = 16MB
board_build.partitions          = huge_app.csv
board_build.arduino.memory_type = qio_opi
```

---

## Power Notes

- The motorcycle runs on a **12 V** battery.
- The ADS1115 battery divider (33 kΩ / 10 kΩ) scales up to ~17.6 V — safe for a 12 V system.
- The ESP32-S3 **must** be powered via a regulated 5 V / 3.3 V supply; do **not** connect it directly to the battery rail.
- Battery-low warning threshold defaults to **10.5 V** (configurable at runtime, saved in EEPROM).

---

## License

This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License (CC BY-NC 4.0).

You may use, modify, and share this project for non-commercial purposes only.

Commercial use is not allowed without explicit permission.
