# Thermal Controller Documentation

## Overview

The `ThermalController` is a smart thermal control system for cloud chamber TEC (Thermoelectric Cooler) cooling. It coordinates two DPS5015 power supplies controlling TEC pairs to achieve and maintain the optimal cold plate temperature for cloud chamber operation.

The system uses a **hierarchical state machine with safety-first design**, automatically optimizing current to find the minimum achievable cold plate temperature while protecting hardware from thermal damage.

**Key Features:**

- Cross-sensor validation (cold plate must be colder than hot plate)
- NVS metrics persistence (tracks all-time best temperature and runtime)
- DPS self-test on startup (verifies communication and output control)
- Impossible value detection (sanity checks on sensor readings)

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
| `MAX_CURRENT_PER_CHANNEL` | 12.0A | Maximum current per DPS        |
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
┌─────────────┐   ┌───────────┐   ┌─────────┐   ┌─────────┐   ┌──────────────┐
│ INITIALIZING│──▶│ SELF_TEST │──▶│ STARTUP │──▶│ RAMP_UP │──▶│ STEADY_STATE │
└─────────────┘   └───────────┘   └─────────┘   └─────────┘   └──────────────┘
       │               │             │               │             │
       │               │             │               │             │
       ▼               ▼             ▼               ▼             │
  ┌─────────────────────────────────────────────────────────────┐  │
  │              FAULT STATES (from any state)                  │  │
  │  ┌───────────────┐  ┌──────────────┐  ┌──────────────────┐  │  │
  │  │ THERMAL_FAULT │  │ SENSOR_FAULT │  │ DPS_DISCONNECTED │  │  │
  │  └───────────────┘  └──────────────┘  └──────────────────┘  │  │
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
  - Load persisted metrics from NVS (runtime, best temperature, session count)
  - **Hot reset detection**: If DPS is already running at >2A, adopt the current to avoid thermal shock
  - Timeout after 30 seconds to fault state
- **Exit conditions**:
  - All hardware ready → SELF_TEST
  - Hot reset with high current → RAMP_UP or STEADY_STATE (skips self-test)
  - Timeout → SENSOR_FAULT or DPS_DISCONNECTED

#### SELF_TEST

- **Purpose**: Verify DPS communication and output control before operation
- **Duration**: ~3-5 seconds typically
- **Phases**:
  1. Configure both DPS to safe test voltage (1V) and current (0.1A) with output OFF
  2. Verify settings were applied correctly
  3. Enable output on DPS0, verify it turned on, disable it
  4. Enable output on DPS1, verify it turned on, disable it
  5. All tests passed → proceed to STARTUP
- **Exit conditions**:
  - All tests pass → STARTUP
  - Any test fails → DPS_DISCONNECTED
  - Timeout → DPS_DISCONNECTED

#### STARTUP

- **Purpose**: Soft-start with low current to prevent thermal shock
- **Duration**: 60 seconds (`STARTUP_HOLD_DURATION_MS`)
- **Actions**:
  - Set both channels to `STARTUP_CURRENT` (2.0A)
  - Enable outputs with fixed voltage (16V)
  - Run all safety checks including sensor sanity validation
- **Exit conditions**:
  - Duration elapsed → RAMP_UP
  - Safety fault → appropriate fault state
  - Manual dial touched → MANUAL_OVERRIDE

#### RAMP_UP

- **Purpose**: Gradually increase current to find optimal operating point
- **Interval**: 20 seconds between adjustments (`RAMP_ADJUSTMENT_INTERVAL_MS`)
- **Algorithm**:
  1. Increase current by adaptive step size (0.5A coarse / 0.25A medium / 0.1A fine) on both channels
  2. Wait 60 seconds for thermal system to stabilize
  3. Evaluate: Did temperature improve?
     - Yes → Record as new optimal, continue ramping
     - No (or warming detected) → Back off to previous current, enter STEADY_STATE
- **Derivative control**: Also monitors cooling _rate_ - if rate degrades significantly even though absolute temperature looks OK, it backs off early
- **Hot side limiting**: Step size halved when in warning zone (55-65°C), ramping stops at alarm zone (≥65°C)
- **Cross-sensor validation**: After 2-minute grace period, warns if cold plate is not colder than hot plate
- **Exit conditions**:
  - Reached `MAX_CURRENT_PER_CHANNEL` → STEADY_STATE
  - Hot side limiting → STEADY_STATE
  - Cooling stalled (rate ≈ 0) → STEADY_STATE
  - Past inflection point (warming) → STEADY_STATE

#### STEADY_STATE

- **Purpose**: Maintain optimal operation, periodically probe for improvement
- **Actions**:
  - Hold current at optimal level
  - Track minimum achieved cold plate temperature (session and all-time)
  - Every 15 minutes, if conditions favorable, try a small current bump
  - Persist all-time best temperature to NVS when new record achieved
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
  - **SYMMETRIC SHUTDOWN**: If one PSU fails, shut down BOTH immediately
  - No degraded mode - running one TEC creates dangerous thermal gradient
  - Wait for reconnection with both channels at 0A
- **Rationale**: The mechanical stress of differential expansion on the cooling assembly (and the glass chamber on top) is too high risk. Thermal gradients across the cold plate can damage seals, crack the vacuum chamber, or warp the aluminum plate.
- **Exit conditions**:
  - Both PSUs reconnected → STARTUP (safe restart)

---

## Control Algorithm

### Optimal Current Finding

The controller automatically finds the optimal TEC current using an iterative approach:

1. **Start low** (2A) to avoid thermal shock
2. **Ramp up** in 0.5A steps with 60-second minimum evaluation windows
3. **Wait for hot-side stabilization**: The 420mm AIO water loop has ~3-5 minute thermal equilibrium time. Don't evaluate cold side success until the hot side dT/dt approaches zero.
4. **Watch for inflection point**: When increasing current no longer improves cooling (or makes it worse), back off

TECs have an optimal operating current - too little current provides insufficient cooling, but too much current generates excess heat that overwhelms the heat rejection capacity. The controller finds this sweet spot automatically.

### Thermal Inertia Compensation

The water cooling loop has significant thermal mass. After a step-change in TEC power:

1. **Cold plate responds quickly** (direct thermal contact with TEC)
2. **Hot plate responds slowly** (heat must propagate through water loop)
3. **Evaluating too early gives misleading results** (cold plate may appear to improve before hot side equilibrates)

The controller waits for **hot side dT/dt to stabilize** (< 0.3°C/min) before evaluating whether a current change helped, with a maximum wait of 5 minutes to prevent infinite waiting.

| Parameter                                  | Value     | Description                     |
| ------------------------------------------ | --------- | ------------------------------- |
| `CURRENT_EVALUATION_DELAY_MS`              | 60000     | Minimum wait time (1 min)       |
| `HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN` | 0.3°C/min | Hot side considered "stable"    |
| `HOT_SIDE_STABILIZATION_MAX_WAIT_MS`       | 300000    | Maximum wait (5 min)            |
| `HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES`      | 60        | 60s window for rate calculation |

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

4. **Manual override** (`checkManualOverride()`):
   - Detect when user adjusts DPS dial
   - Requires **3 consecutive mismatches** to avoid false positives
   - Checks current, voltage, and output state mismatches
   - Ignores readings during grace period after ESP sends commands

### Emergency Shutdown

Two shutdown variants:

**Soft shutdown (controlled ramp-down)**:

```cpp
startEmergencyShutdown()  // Initiate non-blocking ramp-down
updateEmergencyShutdown() // Call in update loop until complete
```

- Ramps down at 2A/sec in 500ms steps
- Disables outputs when current reaches 0
- Used for non-critical faults (hot side warning, sensor issues)

**Hard shutdown (immediate cut)**:

```cpp
hardShutdown()  // Blocking, immediate output disable
```

- Immediate output disable on both channels
- Used for critical faults (hot side ≥70°C)
- Thermal shock acceptable vs. component damage

> ⚠️ **IMPORTANT**: Emergency shutdown relies on Modbus communication. If Modbus is down, shutdown commands won't reach the PSUs. For true fail-safe operation, external hardware interlocks (thermal cutoff switches, contactors) should be used in addition to software control.

---

## Manual Override Detection

The controller detects when a human physically adjusts the DPS dial:

### Detection Logic

1. **Grace period**: After sending a command, wait 3 seconds (`MANUAL_OVERRIDE_GRACE_MS`) for the DPS to process and for the ESP to read back the new value
2. **Pending writes**: If there are queued Modbus writes, don't check for mismatch
3. **Consecutive mismatches**: Require 3 consecutive read cycles (`MANUAL_OVERRIDE_MISMATCH_COUNT`) showing mismatch before declaring override
4. **Tolerance**: Allow ±0.15A and ±0.15V tolerance for measurement noise

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

The controller registers and updates 3 display lines via the Logger:

| Line ID  | Label  | Unit | Content                                 |
| -------- | ------ | ---- | --------------------------------------- |
| TC_STATE | State: | -    | Current state (INIT, START, RAMP, etc.) |
| TC_RATE  | dT/dt: | K/m  | Cooling rate (negative = cooling)       |
| TC_I     | I Set: | A    | Commanded current (same on both ch.)    |

---

## Configuration Reference

All timing and threshold values are defined in `config.h`:

### Timing Parameters

| Parameter                             | Default | Description                         |
| ------------------------------------- | ------- | ----------------------------------- |
| `STARTUP_HOLD_DURATION_MS`            | 60000   | Soft-start duration                 |
| `RAMP_ADJUSTMENT_INTERVAL_MS`         | 20000   | Time between current steps          |
| `CURRENT_EVALUATION_DELAY_MS`         | 60000   | Minimum wait before evaluating      |
| `HOT_SIDE_STABILIZATION_MAX_WAIT_MS`  | 300000  | Max wait for hot side stabilization |
| `HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES` | 60      | Window for hot side dT/dt (samples) |
| `STEADY_STATE_RECHECK_INTERVAL_MS`    | 900000  | Periodic optimization probe         |
| `SENSOR_RECOVERY_TIMEOUT_MS`          | 60000   | Sensor fault recovery window        |
| `INIT_TIMEOUT_MS`                     | 30000   | Hardware initialization timeout     |
| `MANUAL_OVERRIDE_GRACE_MS`            | 3000    | Settling window after command       |
| `EMERGENCY_SHUTDOWN_STEP_MS`          | 500     | Ramp-down step interval             |

### Temperature Thresholds

| Parameter                                  | Default   | Description                |
| ------------------------------------------ | --------- | -------------------------- |
| `HOT_SIDE_WARNING_C`                       | 55°C      | Reduce ramp aggressiveness |
| `HOT_SIDE_WARNING_EXIT_C`                  | 50°C      | Exit warning (hysteresis)  |
| `HOT_SIDE_ALARM_C`                         | 65°C      | Stop increasing current    |
| `HOT_SIDE_ALARM_EXIT_C`                    | 60°C      | Exit alarm (hysteresis)    |
| `HOT_SIDE_FAULT_C`                         | 70°C      | Emergency shutdown         |
| `HOT_SIDE_RATE_FAULT_C_PER_MIN`            | 5.0°C/min | Runaway detection          |
| `HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN` | 0.3°C/min | Hot side stabilization     |
| `COOLING_STALL_THRESHOLD_C`                | 0.5°C/min | Stall detection            |
| `OVERCURRENT_WARMING_THRESHOLD_C`          | 0.3°C     | Back-off threshold         |

### Current Parameters

| Parameter                            | Default | Description                         |
| ------------------------------------ | ------- | ----------------------------------- |
| `TEC_VOLTAGE_SETPOINT`               | 16.0V   | Fixed TEC voltage                   |
| `MAX_CURRENT_PER_CHANNEL`            | 12.0A   | Maximum TEC current                 |
| `MIN_CURRENT_PER_CHANNEL`            | 0.5A    | Minimum TEC current                 |
| `STARTUP_CURRENT`                    | 2.0A    | Initial soft-start current          |
| `DEGRADED_MODE_CURRENT`              | 5.0A    | Single-PSU limit                    |
| `COARSE_STEP_A`                      | 0.5A    | Coarse step size (far from optimum) |
| `EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC` | 2.0A/s  | Shutdown ramp rate                  |
| `Timing::SHUTDOWN_STEP_MS`           | 500ms   | Time between shutdown steps         |

---

## Usage Example

```cpp
#include "ThermalController.h"

// Hardware instances
Logger logger(display, Serial);
PT100Sensor coldPlate(PIN_MAX31865_CS);
DS18B20Sensor hotPlate(oneWire, DS18B20_1_ADDRESS);
DS18B20Sensor ambientSensors[2] = { ... };
DPS5015 psu0(logger, "DC12", Serial1);
DPS5015 psu1(logger, "DC34", Serial2);

// Create controller
ThermalController controller(
    logger,
    coldPlate,
    hotPlate,
    ambientSensors, 2,
    psu0,   // First PSU (reference)
    psu1    // Second PSU (reference)
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

| Symptom                    | Possible Cause          | Solution                                     |
| -------------------------- | ----------------------- | -------------------------------------------- |
| Stays in INITIALIZING      | Hardware not responding | Check wiring, sensor addresses               |
| Stays in SELF_TEST         | DPS not responding      | Check Modbus wiring, baud rate               |
| Frequent MANUAL_OVERRIDE   | Noisy Modbus, EMI       | Add ferrite cores, check grounding           |
| Never reaches STEADY_STATE | Hot side limiting       | Improve heat rejection (fans, flow)          |
| Thermal runaway            | Insufficient cooling    | Reduce max current, improve heatsink         |
| Channel imbalance logs     | TEC degradation         | Replace TECs or adjust mounting              |
| "Cold>=Hot" warnings       | Sensors swapped/faulty  | Check sensor wiring and placement            |
| "FAULT: impossible" error  | Sensor reading invalid  | Check sensor connections, replace if damaged |

### Debug Information

Monitor these for diagnostics:

- Serial output for state transitions and events
- Display shows state, cooling rate, commanded currents
- History buffer contains 5-minute trend data
- NVS metrics show all-time best temperature and total runtime
- **PSRAM log buffer** contains ~12,800 log entries with timestamps (see below)

---

## PSRAM Log Buffer

The Logger maintains a circular buffer in PSRAM (volatile memory with unlimited writes) that stores ~12,800 log entries with timestamps. This is invaluable for post-mortem diagnostics after an issue occurs.

### Characteristics

| Property        | Value                                      |
| --------------- | ------------------------------------------ |
| Location        | PSRAM (external DRAM)                      |
| Size            | 1MB (~12,800 entries × 80 chars)           |
| Write endurance | Unlimited (DRAM, not flash)                |
| Persistence     | **Volatile** - lost on power cycle / reset |
| Timestamps      | Milliseconds since boot                    |

### Accessing the Log Buffer

**Via Serial command** (future enhancement - not yet implemented):

```
// TODO: Add serial command handler
```

**Programmatically**:

```cpp
// Dump all logs to Serial
logger.dumpLogBuffer();

// Check how many entries are stored
size_t count = logger.getLogBufferCount();
```

**Output format**:

```
=== BEGIN LOG BUFFER DUMP ===
Total entries: 347
---
[1234] TC: init
[1289] DS18B20 initialized.
[1345] DPS5015 connected.
[2100] TC: -> TEST
[5234] TC: Self-test PASS
[5235] TC: -> START
...
---
=== END LOG BUFFER DUMP ===
```

### When to Use

- **After THERMAL_FAULT**: Dump logs to understand what led to the fault
- **After unexpected behavior**: See the sequence of events
- **Debugging sensor issues**: Track when warnings/errors occurred
- **Performance analysis**: Correlate state transitions with timestamps

### Limitations

- Buffer is **circular** - oldest entries overwritten when full
- Lost on any reset (power cycle, watchdog, brownout)
- All 500 entries dump at once - no filtering
- For persistent logging, use SPIFFS data logging (future enhancement)

### Implementation Notes

The buffer is allocated in PSRAM using `heap_caps_malloc(MALLOC_CAP_SPIRAM)`. If PSRAM is unavailable (allocation fails), logging continues normally to Serial only and `getLogBufferCount()` returns 0.

Each log entry stores:

```
[timestamp_ms] original_message
```

Where `timestamp_ms` is `millis()` at the time of logging.

---

## Cross-Sensor Validation

The controller validates that sensor readings make physical sense:

### Cross-Check (Cold vs Hot)

After a 2-minute grace period in RAMP_UP state, the controller warns if the cold plate is not at least 2°C colder than the hot plate. This could indicate:

- Sensors are swapped or misconfigured
- A sensor is reading incorrectly
- TECs are not cooling (failed or reversed polarity)
- Insufficient thermal paste or mounting pressure

This is a **warning, not a fault** - the condition might be transient during thermal equilibration.

### Sanity Limits

The controller faults immediately if sensor readings are physically impossible:

| Sensor     | Min Valid | Max Valid | Rationale                         |
| ---------- | --------- | --------- | --------------------------------- |
| Cold Plate | -60°C     | 80°C      | PT100 limit / shouldn't be hot    |
| Hot Plate  | -20°C     | 100°C     | Shouldn't freeze / way past fault |

These indicate sensor malfunction rather than thermal issues.

---

## NVS Metrics Persistence

The controller persists key metrics to flash (NVS) for long-term diagnostics:

### Stored Metrics

| Key        | Type  | Description                         |
| ---------- | ----- | ----------------------------------- |
| `min_t`    | float | All-time minimum cold plate temp    |
| `opt_i`    | float | Current that achieved best temp     |
| `runtime`  | ulong | Total accumulated runtime (seconds) |
| `sessions` | ulong | Number of boot cycles               |

### Write Strategy (Flash Wear Prevention)

NVS flash has limited write cycles (~100,000). The controller minimizes writes:

- **Runtime**: Saved every 1 minute (incremental accumulation)
- **Temperature/Current**: Saved every 5 minutes (only if changed)
- **Force save**: When new all-time minimum temperature achieved

At these intervals, the flash will last:

- Runtime writes: 100,000 ÷ (60 writes/hour) = 1,666 hours = 69 days continuous
- But NVS internally deduplicates unchanged values, so actual wear is much lower
- Realistic expectation: Years of operation

### Metrics Display on Startup

On boot, the controller logs:

```
TC: Best=-32.5C@8.5A
TC: Runtime=156h23m #47
```

This shows the best temperature ever achieved, the current that achieved it, total runtime, and session count.

---

## DPS Self-Test

On startup (after INITIALIZING, before STARTUP), the controller runs a self-test sequence to verify DPS communication and output control:

### Test Phases

1. **Configure**: Set both DPS to safe test values (1V, 0.1A, output OFF)
2. **Verify settings**: Confirm voltage setting was applied
3. **Test DPS0 output**: Enable output, verify it turned on, disable
4. **Test DPS1 output**: Enable output, verify it turned on, disable
5. **Complete**: All tests passed, proceed to STARTUP

### Failure Handling

If any phase fails or times out (3 seconds per DPS), the controller transitions to DPS_DISCONNECTED state.

### Hot Reset Bypass

Self-test is **skipped** on hot reset (when DPS is already running at >2A). This avoids disrupting an already-running system that might be recovering from a watchdog reset.

### Configuration

| Parameter                 | Value | Description                 |
| ------------------------- | ----- | --------------------------- |
| `DPS_SELFTEST_TIMEOUT_MS` | 3000  | Max time per phase          |
| `DPS_SELFTEST_SETTLE_MS`  | 500   | Wait after each command     |
| `DPS_SELFTEST_VOLTAGE`    | 1.0V  | Safe test voltage (no load) |
| `DPS_SELFTEST_CURRENT`    | 0.1A  | Safe test current (no load) |
