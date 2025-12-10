# Thermal Controller Documentation

## Overview

The `ThermalController` is a smart thermal control system for cloud chamber TEC (Thermoelectric Cooler) cooling. It coordinates two DPS5015 power supplies controlling TEC pairs to achieve and maintain the optimal cold plate temperature for cloud chamber operation.

The system uses a **hierarchical state machine with safety-first design**, automatically optimizing current to find the minimum achievable cold plate temperature while protecting hardware from thermal damage.

---

## Hardware Configuration

### Sensors

| Sensor     | Type                         | Purpose                                                    |
| ---------- | ---------------------------- | ---------------------------------------------------------- |
| Cold Plate | PT100 (MAX31865)             | Primary control feedback - measures cold side of TEC stack |
| Hot Plate  | DS18B20                      | Safety monitoring - measures hot side heat rejection       |
| Ambient    | DS18B20 (optional, multiple) | Environmental reference                                    |

### Power Supplies

- **2× DPS5015** programmable power supplies
- Communication: Modbus RTU over UART (9600 baud default)
- Each channel drives one TEC pair in the cascade configuration
- Fixed voltage mode (16V default) with variable current control

### Key Parameters (from `config.h`)

| Parameter                 | Value | Description                    |
| ------------------------- | ----- | ------------------------------ |
| `TEC_VOLTAGE_SETPOINT`    | 16.0V | Fixed voltage for cascade TECs |
| `MAX_CURRENT_PER_CHANNEL` | 10.6A | Maximum current per DPS        |
| `MIN_CURRENT_PER_CHANNEL` | 0.5A  | Minimum operational current    |
| `STARTUP_CURRENT`         | 2.0A  | Initial soft-start current     |
| `HOT_SIDE_FAULT_C`        | 70°C  | Emergency shutdown threshold   |
| `HOT_SIDE_ALARM_C`        | 65°C  | Stop increasing current        |
| `HOT_SIDE_WARNING_C`      | 55°C  | Reduce ramp aggressiveness     |

---

## State Machine

The controller operates as a finite state machine with the following states:

```
                    ┌──────────────────────────────────────────────┐
                    │                                              │
                    ▼                                              │
┌─────────────┐   ┌─────────┐   ┌─────────┐   ┌──────────────┐    │
│ INITIALIZING│──▶│ STARTUP │──▶│ RAMP_UP │──▶│ STEADY_STATE │────┤
└─────────────┘   └─────────┘   └─────────┘   └──────────────┘    │
       │               │             │               │             │
       │               │             │               │             │
       ▼               ▼             ▼               ▼             │
  ┌─────────────────────────────────────────────────────────────┐  │
  │              FAULT STATES (Any state can transition)        │  │
  │  ┌───────────────┐  ┌──────────────┐  ┌──────────────────┐ │  │
  │  │ THERMAL_FAULT │  │ SENSOR_FAULT │  │ DPS_DISCONNECTED │ │  │
  │  └───────────────┘  └──────────────┘  └──────────────────┘ │  │
  └─────────────────────────────────────────────────────────────┘  │
                                                                   │
  ┌─────────────────┐                                              │
  │ MANUAL_OVERRIDE │◀─────────────────────────────────────────────┘
  └─────────────────┘  (detected from any operational state)
```

### State Descriptions

#### INITIALIZING

- **Purpose**: Wait for all hardware to come online
- **Entry condition**: System startup
- **Actions**:
  - Check PT100, DS18B20, and both DPS5015 connections
  - **Hot reset detection**: If DPS is already running at >2A, adopt the current to avoid thermal shock
  - Timeout after 30 seconds to fault state
- **Exit conditions**:
  - All hardware ready → STARTUP
  - Hot reset with high current → RAMP_UP or STEADY_STATE
  - Timeout → SENSOR_FAULT or DPS_DISCONNECTED

#### STARTUP

- **Purpose**: Soft-start with low current to prevent thermal shock
- **Duration**: 60 seconds (`STARTUP_HOLD_DURATION_MS`)
- **Actions**:
  - Set both channels to `STARTUP_CURRENT` (2.0A)
  - Enable outputs with fixed voltage (16V)
  - Run all safety checks
- **Exit conditions**:
  - Duration elapsed → RAMP_UP
  - Safety fault → appropriate fault state
  - Manual dial touched → MANUAL_OVERRIDE

#### RAMP_UP

- **Purpose**: Gradually increase current to find optimal operating point
- **Interval**: 20 seconds between adjustments (`RAMP_ADJUSTMENT_INTERVAL_MS`)
- **Algorithm**:
  1. Increase current by `RAMP_CURRENT_STEP_A` (0.5A) on both channels
  2. Wait 60 seconds for thermal system to stabilize
  3. Evaluate: Did temperature improve?
     - Yes → Record as new optimal, continue ramping
     - No (or warming detected) → Back off to previous current, enter STEADY_STATE
- **Derivative control**: Also monitors cooling _rate_ - if rate degrades significantly even though absolute temperature looks OK, it backs off early
- **Hot side limiting**: Step size halved when in warning zone (55-65°C), ramping stops at alarm zone (≥65°C)
- **Exit conditions**:
  - Reached `MAX_CURRENT_PER_CHANNEL` → STEADY_STATE
  - Hot side limiting → STEADY_STATE
  - Cooling stalled (rate ≈ 0) → STEADY_STATE
  - Past inflection point (warming) → STEADY_STATE

#### STEADY_STATE

- **Purpose**: Maintain optimal operation, periodically probe for improvement
- **Actions**:
  - Hold current at optimal level
  - Track minimum achieved cold plate temperature
  - Every 15 minutes, if conditions favorable, try a small current bump
- **Periodic probing**: If hot side has headroom (<50°C) and not at max current, attempt a half-step increase and evaluate after 60 seconds
- **Exit conditions**:
  - Safety fault → appropriate fault state
  - Manual dial touched → MANUAL_OVERRIDE

#### MANUAL_OVERRIDE

- **Purpose**: Human has taken control - stop all automatic adjustments
- **Entry**: Detected via consecutive current/voltage mismatches between commanded and actual values
- **Actions**:
  - Log "Manual override" event
  - Continue monitoring thermal limits for safety
  - **No commands sent** - respects human intervention
- **Exit**: None (requires power cycle)

#### THERMAL_FAULT

- **Purpose**: Critical temperature exceeded - emergency shutdown
- **Triggers**:
  - Hot side ≥ 70°C (`HOT_SIDE_FAULT_C`)
  - Thermal runaway detected (>5°C/min rise on hot side)
  - Hot side sensor disconnected
- **Actions**:
  - **Hard cut** at 70°C: Immediate output disable (thermal shock acceptable vs. component damage)
  - **Soft ramp** below 70°C: Controlled ramp-down at 2A/sec
- **Exit**: None (latched state, requires power cycle)

#### SENSOR_FAULT

- **Purpose**: Critical sensor failure - reduce power and wait for recovery
- **Actions**:
  - Reduce current to `DEGRADED_MODE_CURRENT / 2` (2.5A)
  - Wait up to 60 seconds for sensor recovery
- **Exit conditions**:
  - Sensors recovered → return to previous state
  - Timeout → THERMAL_FAULT (precautionary shutdown)

#### DPS_DISCONNECTED

- **Purpose**: Lost Modbus communication with PSU(s)
- **Actions**:
  - If one PSU connected, operate in degraded mode (max 5A)
  - Wait for reconnection
- **Exit conditions**:
  - Both PSUs reconnected → STARTUP (safe restart)

---

## Control Algorithm

### Optimal Current Finding

The controller automatically finds the optimal TEC current using an iterative approach:

1. **Start low** (2A) to avoid thermal shock
2. **Ramp up** in 0.5A steps with 60-second evaluation windows
3. **Watch for inflection point**: When increasing current no longer improves cooling (or makes it worse), back off

TECs have an optimal operating current - too little current provides insufficient cooling, but too much current generates excess heat that overwhelms the heat rejection capacity. The controller finds this sweet spot automatically.

### Derivative Control

Beyond absolute temperature, the controller monitors **cooling rate** (°C/min calculated via linear regression over 30-second window):

- If cooling rate degrades significantly after a current increase, the controller backs off _before_ temperature actually warms
- This provides faster response than waiting for thermal mass to reflect the problem

### Temperature Hysteresis

Hot side temperature zones use hysteresis to prevent oscillation:

| Zone    | Entry | Exit  | Action             |
| ------- | ----- | ----- | ------------------ |
| Normal  | <55°C | -     | Full step size     |
| Warning | ≥55°C | <50°C | Half step size     |
| Alarm   | ≥65°C | <60°C | Stop ramping       |
| Fault   | ≥70°C | -     | Emergency shutdown |

---

## Safety Systems

### Multi-Layer Protection

1. **Thermal limits** (`checkThermalLimits()`):

   - Hot side sensor disconnect → immediate fault
   - Temperature ≥70°C → hard cut (immediate disable)
   - Temperature rise >5°C/min → runaway detection

2. **Sensor health** (`checkSensorHealth()`):

   - PT100 error state → SENSOR_FAULT
   - DS18B20 disconnect → SENSOR_FAULT

3. **DPS connection** (`checkDpsConnection()`):

   - Track previous connection state
   - Detect disconnection → DPS_DISCONNECTED

4. **Manual override** (`checkForManualOverride()`):
   - Detect when user adjusts DPS dial
   - Requires **3 consecutive mismatches** to avoid false positives
   - Ignores readings during grace period after ESP sends commands

### Emergency Shutdown

Two variants:

**Non-blocking (normal operation)**:

```cpp
startEmergencyShutdown() → updateEmergencyShutdown()
```

- Ramps down at 2A/sec in 500ms steps
- Disables outputs when current reaches 0

**Blocking (initialization only)**:

```cpp
emergencyShutdown()
```

- Quick ramp-down in ~1 second
- Used when DPS found running on hot reset

> ⚠️ **IMPORTANT**: Emergency shutdown relies on Modbus communication. If Modbus is down, shutdown commands won't reach the PSUs. For true fail-safe operation, external hardware interlocks (thermal cutoff switches, contactors) should be used in addition to software control.

---

## Manual Override Detection

The controller detects when a human physically adjusts the DPS dial:

### Detection Logic

1. **Grace period**: After sending a command, wait 3 seconds (`MANUAL_OVERRIDE_GRACE_MS`) for the DPS to process and for the ESP to read back the new value
2. **Pending writes**: If there are queued Modbus writes, don't check for mismatch
3. **Consecutive mismatches**: Require 3 consecutive read cycles (`MANUAL_OVERRIDE_MISMATCH_COUNT`) showing mismatch before declaring override
4. **Tolerance**: Allow ±0.15A and ±0.15V tolerance for measurement noise

### Pre-write Validation

Before sending a current command, the controller validates that the DPS is in the expected state:

```cpp
if (!_psus[channel].validateStateBeforeWrite(current)) {
    // DPS state doesn't match - user touched the dial
    transitionTo(ThermalState::MANUAL_OVERRIDE);
}
```

This catches overrides even before the post-write mismatch detection.

---

## Hot Reset Handling

If the ESP32 resets (e.g., watchdog timeout, crash) while the DPS units are running:

1. **Detection**: During INITIALIZING, check if DPS output is on with current >2A
2. **Adoption**: Read actual DPS current and adopt it as target (avoids thermal shock from sudden current drop)
3. **Skip startup**: Go directly to RAMP_UP or STEADY_STATE based on adopted current level
4. **Logging**: Log "Hot reset detected" and adopted current value

---

## Channel Imbalance Detection

When both channels are commanded to the same current, the controller monitors for asymmetry:

- **Current imbalance**: |I1 - I2| > 1.0A
- **Power imbalance**: |P1 - P2| > 5.0W

Imbalances are logged (rate-limited to once per minute) as diagnostic information indicating potential issues:

- TEC degradation on one side
- Different TEC thermal resistance
- Mounting issues
- PSU calibration differences

---

## History Buffer and Trend Analysis

### History Buffer

- **Size**: 300 samples (5 minutes at 1Hz)
- **Circular buffer**: Head pointer wraps around
- **Data recorded**:
  - Timestamp
  - Cold plate temperature
  - Hot plate temperature
  - Ambient temperature
  - Current setpoints (commanded)
  - Actual currents (measured)
  - Power per channel

### Cooling Rate Calculation

Linear regression over 30-second window:

```cpp
// Slope in degrees per sample, converted to K/min
cooling_rate = slope * 60.0f;
```

- Negative rate = cooling
- Positive rate = warming
- Near-zero rate = stable

### Trend Classification

```cpp
enum class ThermalTrend {
    COOLING,     // rate < -0.5°C/min
    WARMING,     // rate > +0.5°C/min
    STABLE,      // |rate| < 0.5°C/min
    OSCILLATING, // (not currently implemented)
    ANOMALOUS    // insufficient history
};
```

---

## Display Integration

The controller registers and updates 4 display lines via the Logger:

| Line ID  | Label   | Unit | Content                                 |
| -------- | ------- | ---- | --------------------------------------- |
| TC_STATE | State:  | -    | Current state (INIT, START, RAMP, etc.) |
| TC_RATE  | dT/dt:  | K/m  | Cooling rate (negative = cooling)       |
| TC_I1    | I1 SET: | A    | Channel 1 commanded current             |
| TC_I2    | I2 SET: | A    | Channel 2 commanded current             |

---

## Configuration Reference

All timing and threshold values are defined in `config.h`:

### Timing Parameters

| Parameter                          | Default | Description                        |
| ---------------------------------- | ------- | ---------------------------------- |
| `STARTUP_HOLD_DURATION_MS`         | 60000   | Soft-start duration                |
| `RAMP_ADJUSTMENT_INTERVAL_MS`      | 20000   | Time between current steps         |
| `CURRENT_EVALUATION_DELAY_MS`      | 60000   | Wait time before evaluating change |
| `STEADY_STATE_RECHECK_INTERVAL_MS` | 900000  | Periodic optimization probe        |
| `SENSOR_RECOVERY_TIMEOUT_MS`       | 60000   | Sensor fault recovery window       |
| `INIT_TIMEOUT_MS`                  | 30000   | Hardware initialization timeout    |
| `MANUAL_OVERRIDE_GRACE_MS`         | 3000    | Settling window after command      |
| `EMERGENCY_SHUTDOWN_STEP_MS`       | 500     | Ramp-down step interval            |

### Temperature Thresholds

| Parameter                         | Default   | Description                |
| --------------------------------- | --------- | -------------------------- |
| `HOT_SIDE_WARNING_C`              | 55°C      | Reduce ramp aggressiveness |
| `HOT_SIDE_WARNING_EXIT_C`         | 50°C      | Exit warning (hysteresis)  |
| `HOT_SIDE_ALARM_C`                | 65°C      | Stop increasing current    |
| `HOT_SIDE_ALARM_EXIT_C`           | 60°C      | Exit alarm (hysteresis)    |
| `HOT_SIDE_FAULT_C`                | 70°C      | Emergency shutdown         |
| `HOT_SIDE_RATE_FAULT_C_PER_MIN`   | 5.0°C/min | Runaway detection          |
| `COOLING_STALL_THRESHOLD_C`       | 0.5°C/min | Stall detection            |
| `OVERCURRENT_WARMING_THRESHOLD_C` | 0.3°C     | Back-off threshold         |

### Current Parameters

| Parameter                            | Default | Description                |
| ------------------------------------ | ------- | -------------------------- |
| `TEC_VOLTAGE_SETPOINT`               | 16.0V   | Fixed TEC voltage          |
| `MAX_CURRENT_PER_CHANNEL`            | 10.6A   | Maximum TEC current        |
| `MIN_CURRENT_PER_CHANNEL`            | 0.5A    | Minimum TEC current        |
| `STARTUP_CURRENT`                    | 2.0A    | Initial soft-start current |
| `DEGRADED_MODE_CURRENT`              | 5.0A    | Single-PSU limit           |
| `RAMP_CURRENT_STEP_A`                | 0.5A    | Normal step size           |
| `EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC` | 2.0A/s  | Shutdown ramp rate         |

---

## Usage Example

```cpp
#include "ThermalController.h"

// Hardware instances
Logger logger(display, Serial);
PT100Sensor coldPlate(PIN_MAX31865_CS);
DS18B20Sensor hotPlate(oneWire, DS18B20_1_ADDRESS);
DS18B20Sensor ambientSensors[2] = { ... };
DPS5015 psus[2] = { DPS5015(Serial1), DPS5015(Serial2) };

// Create controller
ThermalController controller(
    logger,
    coldPlate,
    hotPlate,
    ambientSensors, 2,
    psus
);

void setup() {
    // Initialize hardware...
    controller.begin();
}

void loop() {
    controller.update();  // Call frequently (≥1Hz)
    // Other tasks...
}
```

---

## Troubleshooting

### Common Issues

| Symptom                    | Possible Cause          | Solution                             |
| -------------------------- | ----------------------- | ------------------------------------ |
| Stays in INITIALIZING      | Hardware not responding | Check wiring, sensor addresses       |
| Frequent MANUAL_OVERRIDE   | Noisy Modbus, EMI       | Add ferrite cores, check grounding   |
| Never reaches STEADY_STATE | Hot side limiting       | Improve heat rejection (fans, flow)  |
| Thermal runaway            | Insufficient cooling    | Reduce max current, improve heatsink |
| Channel imbalance logs     | TEC degradation         | Replace TECs or adjust mounting      |

### Debug Information

Monitor these for diagnostics:

- Serial output for state transitions and events
- Display shows state, cooling rate, commanded currents
- History buffer contains 5-minute trend data
