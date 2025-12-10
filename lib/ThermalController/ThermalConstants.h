/**
 * @file ThermalConstants.h
 * @brief All constants for ThermalController and related modules
 *
 * This file consolidates all thermal control configuration:
 * - Voltage and current limits
 * - Timing intervals
 * - Temperature thresholds
 * - Control parameters
 * - History buffer settings
 * - NVS persistence settings
 * - Sensor validation settings
 */

#ifndef THERMAL_CONSTANTS_H
#define THERMAL_CONSTANTS_H

#include <cstddef>

// =============================================================================
// Voltage and Current Limits
// =============================================================================

constexpr float TEC_VOLTAGE_SETPOINT = 16.0f; // Fixed voltage for cascade TECs
constexpr float MAX_CURRENT_PER_CHANNEL = 12.0f; // Maximum current per DPS
constexpr float MIN_CURRENT_PER_CHANNEL = 0.5f;  // Minimum operational current
constexpr float STARTUP_CURRENT = 2.0f; // Initial current during startup
constexpr float HOT_RESET_NEAR_MAX_A =
    MAX_CURRENT_PER_CHANNEL -
    0.5f; // Threshold for skipping ramp after hot reset
constexpr float DEGRADED_MODE_CURRENT =
    5.0f;                              // Max current when one DPS disconnects
constexpr float DPS_OCP_LIMIT = 12.5f; // Hardware Over Current Protection
                                       // Failsafe in case ESP commands >12A
constexpr float DPS_OVP_LIMIT =
    16.5f; // Hardware Over Voltage Protection
           // Slightly above setpoint to catch Modbus errors

// Hot reset detection threshold (uses STARTUP_CURRENT from above)
constexpr float HOT_RESET_CURRENT_THRESHOLD_A = STARTUP_CURRENT;

// =============================================================================
// Timing Intervals (milliseconds)
// =============================================================================

constexpr unsigned long STARTUP_HOLD_DURATION_MS = 60000; // 60s startup hold
constexpr unsigned long RAMP_ADJUSTMENT_INTERVAL_MS =
    20000; // 20s between ramp steps
constexpr unsigned long CURRENT_EVALUATION_DELAY_MS =
    60000; // 60s to evaluate current change
constexpr unsigned long STEADY_STATE_RECHECK_INTERVAL_MS =
    900000; // 15min recheck
constexpr unsigned long SENSOR_RECOVERY_TIMEOUT_MS =
    60000;                                       // 60s sensor recovery
constexpr unsigned long INIT_TIMEOUT_MS = 30000; // 30s init timeout

// =============================================================================
// Temperature Thresholds (Celsius)
// =============================================================================

constexpr float HOT_SIDE_WARNING_C = 55.0f; // Reduce ramp aggressiveness
constexpr float HOT_SIDE_WARNING_EXIT_C =
    50.0f;                                // Exit warning state (hysteresis)
constexpr float HOT_SIDE_ALARM_C = 65.0f; // Stop increasing current
constexpr float HOT_SIDE_ALARM_EXIT_C = 60.0f; // Exit alarm state (hysteresis)
constexpr float HOT_SIDE_FAULT_C = 70.0f;      // Emergency shutdown
constexpr float HOT_SIDE_RATE_FAULT_C_PER_MIN =
    5.0f; // Thermal runaway detection
constexpr float COOLING_STALL_THRESHOLD_C =
    0.5f; // Consider stalled if less than this
constexpr float OVERCURRENT_WARMING_THRESHOLD_C =
    0.3f; // Back off if cold plate warms this much
constexpr float COOLING_RATE_DEGRADATION_THRESHOLD =
    0.3f; // K/min - detect when cooling rate slows significantly

// =============================================================================
// Control Parameters - Adaptive Step Sizes
// =============================================================================

// Use coarse steps when far from optimum (fast initial cooling)
// Use fine steps when near optimum (precise positioning)
constexpr float COARSE_STEP_A =
    0.5f; // Far from optimum, cooling rate > 1 K/min
constexpr float MEDIUM_STEP_A = 0.25f; // Approaching optimum, rate 0.5-1 K/min
constexpr float FINE_STEP_A = 0.1f;    // Near optimum, rate < 0.5 K/min

// Thresholds for selecting step size based on cooling rate magnitude
constexpr float STEP_COARSE_RATE_THRESHOLD =
    1.0f; // K/min - above this use coarse
constexpr float STEP_MEDIUM_RATE_THRESHOLD =
    0.5f; // K/min - above this use medium

constexpr float MANUAL_OVERRIDE_VOLTAGE_TOLERANCE_V =
    0.15f; // Tolerance for override detection
constexpr float MANUAL_OVERRIDE_CURRENT_TOLERANCE_A =
    0.15f; // Tolerance for override detection

// =============================================================================
// Hot-Side Stabilization (thermal inertia compensation)
// =============================================================================

// The 420mm AIO water loop has ~3-5 minute thermal equilibrium time
// Wait for hot side dT/dt to stabilize before evaluating cold plate success
constexpr float HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN =
    0.3f; // Hot side dT/dt must be below this to be "stable"
constexpr unsigned long HOT_SIDE_STABILIZATION_MAX_WAIT_MS =
    300000; // 5 min max wait for stabilization (prevents infinite wait)
constexpr unsigned long HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES =
    60; // 60s window for hot side rate calculation
constexpr unsigned long MANUAL_OVERRIDE_GRACE_MS =
    3000; // Settling window: time for DPS to process command and for ESP
          // to read back new value before checking for override
constexpr int MANUAL_OVERRIDE_MISMATCH_COUNT =
    3; // Consecutive read cycles showing mismatch before declaring override
       // At 500ms update interval, this is 1.5 seconds of sustained mismatch
constexpr float EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC =
    2.0f; // Emergency shutdown ramp

// =============================================================================
// Channel Imbalance Detection
// =============================================================================

constexpr float CHANNEL_CURRENT_IMBALANCE_A =
    1.0f; // Log warning if |I1-I2| exceeds this when both commanded equal
constexpr float CHANNEL_POWER_IMBALANCE_W =
    5.0f; // Log warning if |P1-P2| exceeds this when both commanded equal

// =============================================================================
// History Buffer
// =============================================================================

constexpr size_t HISTORY_BUFFER_SIZE = 300;                // 5 minutes at 1Hz
constexpr unsigned long HISTORY_SAMPLE_INTERVAL_MS = 1000; // 1 second samples
constexpr size_t COOLING_RATE_WINDOW_SAMPLES =
    30; // 30s window for rate calculation

// =============================================================================
// Buffer Size Validation
// =============================================================================

// Validate that buffer size and sample interval give expected history duration
// Expected: HISTORY_BUFFER_SIZE * HISTORY_SAMPLE_INTERVAL_MS = 5 minutes (300s)
static_assert(HISTORY_BUFFER_SIZE * HISTORY_SAMPLE_INTERVAL_MS == 300000,
              "History buffer does not hold exactly 5 minutes of data");

// Validate that rate calculation windows fit in buffer
static_assert(COOLING_RATE_WINDOW_SAMPLES <= HISTORY_BUFFER_SIZE,
              "Cooling rate window exceeds buffer size");

static_assert(HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES <= HISTORY_BUFFER_SIZE,
              "Hot side rate window exceeds buffer size");

// =============================================================================
// Cross-Sensor Validation
// =============================================================================

// During normal cooling operation, cold plate must be colder than hot plate
// This check is only active after STARTUP (TECs need time to establish
// gradient)
constexpr float SENSOR_CROSS_CHECK_MARGIN_C =
    2.0f; // Cold plate must be at least this much colder than hot
constexpr unsigned long SENSOR_CROSS_CHECK_GRACE_MS =
    120000; // 2 minutes grace after entering RAMP_UP before checking
constexpr unsigned long SENSOR_CROSS_CHECK_LOG_INTERVAL_MS =
    30000; // Rate limit cross-check warnings

// Absolute sensor sanity limits
constexpr float COLD_PLATE_MIN_VALID_C = -60.0f; // PT100 can't read below this
constexpr float COLD_PLATE_MAX_VALID_C = 80.0f;  // Cold plate shouldn't be hot
constexpr float HOT_PLATE_MIN_VALID_C = -20.0f;  // Hot side shouldn't freeze
constexpr float HOT_PLATE_MAX_VALID_C = 100.0f;  // Above fault threshold

// PT100 plausibility check (physics-based validation)
// If cold plate reports very low temp but hot side is only moderately warm,
// the PT100 reading is implausible (sensor may be shorted or miscalibrated)
// Expected: At full power, deltaT across TEC should be proportional to current
constexpr float PLAUSIBILITY_MIN_DELTA_T_PER_AMP =
    2.0f; // Minimum expected K/A at steady state
constexpr float PLAUSIBILITY_MAX_COLD_IMPLAUSIBLE_C =
    -35.0f; // Below this, check hot side
constexpr float PLAUSIBILITY_HOT_THRESHOLD_FOR_CHECK_C =
    35.0f; // Hot side must be above this for extreme cold
constexpr unsigned long PLAUSIBILITY_CHECK_GRACE_MS =
    180000; // 3 min grace after startup

// =============================================================================
// NVS Metrics Persistence
// =============================================================================

// Persist key metrics to NVS for long-term diagnostics
// Write interval is long to avoid wearing out flash (NVS has limited writes)
constexpr unsigned long NVS_METRICS_SAVE_INTERVAL_MS =
    300000; // Save every 5 minutes
constexpr unsigned long NVS_RUNTIME_SAVE_INTERVAL_MS =
    60000; // Save runtime every 1 minute (incremental)

// =============================================================================
// Display Line IDs (internal)
// =============================================================================

namespace ThermalDisplay {
constexpr const char *LINE_STATE = "TC_STATE";
constexpr const char *LINE_RATE = "TC_RATE";
constexpr const char *LINE_CURRENT =
    "TC_I"; // Single line - both channels use same current
} // namespace ThermalDisplay

// =============================================================================
// Tolerance Constants
// =============================================================================

namespace Tolerance {
// Minimum current below which we consider output "off"
constexpr float MIN_CURRENT_THRESHOLD = 0.1f;

// NVS float comparison tolerance
constexpr float NVS_FLOAT_TOLERANCE = 0.01f;

// Temperature improvement threshold
constexpr float TEMP_IMPROVEMENT_THRESHOLD_C = 0.1f;
} // namespace Tolerance

// =============================================================================
// Timing Constants (implementation details)
// =============================================================================

namespace Timing {
// Time between emergency shutdown current steps
constexpr unsigned long SHUTDOWN_STEP_MS = 500;

// Imbalance logging rate limit
constexpr unsigned long IMBALANCE_LOG_INTERVAL_MS = 60000;

// DPS self-test timing
constexpr unsigned long SELFTEST_TIMEOUT_MS = 3000; // Max time per DPS test
constexpr unsigned long SELFTEST_SETTLE_MS = 500;   // Wait after each command

// Startup configuration timing
constexpr unsigned long STARTUP_CONFIG_WINDOW_MS =
    500; // Window for initial PSU config
constexpr unsigned long STARTUP_CONFIG_SEND_MS =
    50; // Send config in first 50ms
} // namespace Timing

#endif // THERMAL_CONSTANTS_H
