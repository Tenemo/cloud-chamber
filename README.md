# Cloud Chamber Controller

Firmware for an ESP32-S3-based cloud chamber controller. It monitors multiple
temperature sensors, drives two DPS5015 power supplies symmetrically, and
implements a safety-first thermal control loop with live display + serial logs.

## What this project contains

- **Thermal control core**: state machine, optimizer, safety checks, metrics.
- **Power layer**: dual DPS5015 Modbus control, emergency shutdown.
- **Sensors**: PT100 via MAX31865, DS18B20 OneWire sensors.
- **UI/logging**: TFT display + serial logging (single Logger interface).
- **Time sync**: optional WiFi boot-time NTP sync.

See `lib/Control/ThermalController.md` for a deeper architecture walkthrough.

## Hardware prerequisites

- **MCU**: DFRobot FireBeetle 2 ESP32-S3 (see `platformio.ini`)
- **Power**: 2x DPS5015 (Modbus RTU, symmetric control)
- **Sensors**:
  - PT100 + MAX31865 (cold plate)
  - DS18B20 x3 (hot plate, ambient, glass top)
- **Display**: DFRobot GDL-compatible TFT (GDI FPC)
- Wiring matches `include/pins.h`

## Software prerequisites

- **PlatformIO** (VS Code extension or CLI)
- USB serial drivers for the ESP32-S3 board
- A serial monitor (PlatformIO monitor or equivalent)

Libraries are fetched automatically via `platformio.ini` (Adafruit GFX,
MAX31865, OneWire, DallasTemperature, DFRobot_GDL, etc.).

## Quick start

1) Install PlatformIO.
2) Open this folder in VS Code (PlatformIO extension) or use the CLI.
3) Copy `include/env.h.sample` to `include/env.h` and fill in WiFi credentials
   if you want time sync. If not, keep `FEATURE_WIFI_TIME_SYNC` disabled.
4) Review and adjust:
   - `include/config.h` (limits, timings, tuning)
   - `include/pins.h` (GPIO map)
   - `platformio.ini` (upload port, board settings)
5) Build and flash:

```bash
pio run -t upload
```

6) Open serial monitor:

```bash
pio device monitor -b 115200
```

## Operating modes

- **Optimizer (default)**: `ThermalController::begin()` uses hill-climbing to
  find/maintain optimal current.
- **Fixed current override**: `ThermalController::fixedCurrent(A)` ramps to a
  target current and holds it indefinitely.
- **Benchmark sequence**: `ThermalController::beginBenchmark(...)` ramps/holds
  a sequence of fixed currents.

## Project layout

- `src/` – entry point (`main.cpp`)
- `include/` – configuration and pin mapping (`config.h`, `pins.h`, `env.h`)
- `lib/Control/` – controller state machine, optimizer, safety, metrics
- `lib/Power/` – DPS5015 + symmetric dual PSU control
- `lib/Sensors/` – PT100/DS18B20 drivers and aggregation
- `lib/Logging/` – display + serial logger
- `lib/Time/` – optional WiFi time sync

## Safety notes

This system controls high current / high power hardware. Double-check wiring,
limits in `include/config.h`, and ensure thermal protections are appropriate
before running unattended.
