# Motorrad-Dash

A custom motorcycle dashboard built on the **ESP32-S3 N16R8** (16 MB Flash, 8 MB OPI PSRAM), developed with PlatformIO and the Arduino framework. It displays real-time riding data on a 1.51" transparent OLED and integrates multiple sensors, a RaceBox Mini GPS logger, and a Blitzer-Warner radar detector.

---

## Features

| Feature | Details |
|---|---|
| **Oil temperature** | NTC resistor read via ADS1115 (CH0), EMA-smoothed |
| **Coolant temperature** | OBD2 PID 0x05 via CAN bus (TWAI), displayed on Oil page |
| **Lean angle** | BNO085 IMU — `ARVR_STABILIZED_GRV` quaternion, OEM-style 4-mode drift correction, soft deadzone, time-based filter |
| **G-force** | BNO085 linear acceleration, all-time max stored in EEPROM |
| **Battery voltage** | 33 kΩ / 10 kΩ divider on ADS1115 CH2, calibration offset |
| **Outside temperature** | DS18B20 1-Wire sensor, calibration offset |
| **Ambient light / Night mode** | BH1750 lux meter → auto-adjusts OLED contrast; modes: Auto / Tag / Nacht / Sonne (invertiertes Display) |
| **RaceBox Mini** | GPS-fix, BLE-connected and recording-status via GPIO inputs |
| **Blitzer-Warner** | Digital alert pin + heartbeat LED voltage on ADS1115 CH1 |
| **OBD2 via CAN (TWAI)** | RPM 10 Hz, Speed 5 Hz, Coolant/Load/Throttle ~1 Hz; 3 s staleness timeout |

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
| RaceBox Mini | Recording | 2 |
| RaceBox Mini | Simulated button (NPN) | 3 |
| CAN bus (TWAI) | TX | 18 |
| CAN bus (TWAI) | RX | 17 |

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

## Display Pages & Navigation

Two navigation groups, switched by long press (800 ms):

**Primary group** — short press cycles OIL ↔ LEAN  
**Secondary group** — short press cycles G → ENGINE → RACEBOX → G  
Long press on LEAN → enter secondary group  
Long press on any secondary page → back to primary group

| Page | Content | Long-press action |
|---|---|---|
| OIL | Oil temp (°C), coolant temp (OBD2), outside temp, battery voltage | Open settings (hold 5 s) |
| LEAN | Current lean angle, corner peak hold, all-time max L/R; CAN-offline badge top-right when OBD2 speed unavailable | Enter secondary group (800 ms) |
| G | Current & all-time max G-force, quadrant peaks | Enter primary group (800 ms) |
| ENGINE | RPM, coolant, load, throttle, speed, 0–100 km/h timer | Enter primary group (800 ms) |
| RACEBOX | GPS fix / BLE / recording status, auto-start recording | Enter primary group (800 ms) |

### Settings (OIL page hold 5 s)

Short press navigates items (scrolling list). Long press (600 ms) changes value.

| Item | Action |
|---|---|
| Brightness | Cycle Auto → Tag → Nacht → Sonne |
| Sleep Mode | Turn display off (any button press wakes) |
| Lean Flip | Invert lean direction |
| Lean Offset | Capture current roll as zero reference |
| Pitch Offset | Capture current pitch as G-force correction |
| Reset Lean | Reset all-time lean max L/R (EEPROM) |

**Brightness modes:**
- **Auto** — BH1750 adjusts contrast between 0x35 (night) and 0xFF (day)
- **Tag** — always max brightness
- **Nacht** — always min brightness
- **Sonne** — display inverted (white background, black text) for direct sunlight readability

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

### Boot sequence
1. OLED turns on immediately.
2. Each sensor is initialised and shows **OK** or **FAIL** as it comes up.
3. 2-second hold window: hold the button during this window to enter **calibration mode** (CAL icon top-right).

### Lean angle drift correction

The BNO085 uses `SH2_ARVR_STABILIZED_GRV` (no magnetometer) combined with a 4-mode software drift-cancel algorithm:

| Mode | Condition | Action |
|---|---|---|
| 1 | Speed < 10 km/h (OBD2) | Fast correction τ = 1 s |
| 2 | \|ω\| < 0.08 rad/s (going straight) | Correct τ = 4 s |
| 3 | Cornering + OBD2 speed available | Physics model `θ = atan(v · ψ̇_earth / g)`, τ = 3 s |
| 4 | Cornering, no OBD2 speed | Freeze drift estimate |

Mode 3 is the same principle used by OEM moto-IMUs (Bosch BMI / Continental 6DOF) — it computes the expected lean angle from vehicle speed and earth-frame yaw rate, allowing continuous drift correction even through repeated S-bends.

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
