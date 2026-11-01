# AutoHUDESP

An ESP32-based automotive heads-up display that renders vehicle telemetry with
LVGL. The firmware supports an Elecrow 7-inch ESP32-S3 RGB panel and a smaller
ESP32/ILI9341 display configuration for Wokwi.

## Features

- Dashboard for speed, RPM, coolant temperature, engine load, and oil temperature.
- BLE scanning and ELM327-compatible OBD-II polling in vehicle mode.
- Synthetic telemetry mode for display testing.
- LVGL interface rendered with LovyanGFX.

Speed is displayed in MPH and temperatures in Fahrenheit. Oil temperature
depends on support from the vehicle and OBD-II adapter.

## Hardware

### Vehicle setup

- Elecrow CrowPanel 7-inch HMI with ESP32-S3 and RGB display.
- BLE OBD-II adapter that supports ELM327 commands. The firmware scans for
  adapters advertising a name containing `VEEPEAK` or `OBD`.
- Vehicle OBD-II port.

Adapter compatibility varies. The adapter must expose a usable BLE GATT
write/notify transport and support the requested OBD-II PIDs.

### Wokwi display setup

The checked-in [diagram.json](diagram.json) describes an ESP32 DevKit and an
ILI9341 SPI display. Its connections are:

| ESP32 | ILI9341 |
| --- | --- |
| GPIO 15 | CS |
| GPIO 2 | DC |
| GPIO 18 | SCK |
| GPIO 23 | MOSI |

## Build and upload

Install [PlatformIO](https://platformio.org/install) (for example, the
PlatformIO IDE extension for VS Code), then open this project directory.
PlatformIO installs the libraries listed in `platformio.ini` when building.

### Elecrow ESP32-S3 panel

The default PlatformIO environment is `elecrow_7_hmi_s3`:

```sh
pio run -e elecrow_7_hmi_s3
pio run -e elecrow_7_hmi_s3 -t upload
pio device monitor -b 115200
```

Connect the ESP32-S3 to your computer over USB. If more than one serial device
is available, select the board's port in PlatformIO before uploading or opening
the monitor.

### Wokwi / ESP32 DevKit with ILI9341

Build the environment matching the checked-in Wokwi diagram:

```sh
pio run -e elecrow_7_hmi
```

For animated test values instead of BLE/OBD-II polling, change
`SIMULATION_MODE` near the top of `main.cpp` from `0` to `1`, then build again.
Restore it to `0` for vehicle mode. The Wokwi configuration uses the build
artifacts from `.pio/build/elecrow_7_hmi/`; run the project with the Wokwi
extension or simulator after building.

## Safety

This project is experimental and is not a certified automotive instrument.
Always prioritize the vehicle's built-in instruments, follow local laws, and
do not configure or troubleshoot the display while driving.
