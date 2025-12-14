/**
 * @file config.h
 * @brief Tunable configuration parameters
 *
 * This file contains all tunable parameters that may need adjustment:
 * - Update intervals and timing
 * - Communication settings
 * - Display settings
 * - Thermal control thresholds and limits
 *
 * For pin definitions and hardware addresses, see pins.h
 * For implementation constants that rarely change, see ThermalConstants.h
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pins.h"
#include <cstddef>

// =============================================================================
// Control Loop Enable
// =============================================================================
// When false, the system runs as a simple sensor logger without PSU control
// Note: Must be a preprocessor macro (not constexpr) for #if directives
#define CONTROL_LOOP_ENABLED 1

// =============================================================================
// WiFi Time Sync Enable
// =============================================================================
// When false, skip WiFi/NTP time sync on boot (faster startup)
#define WIFI_TIME_SYNC_ENABLED 0

// =============================================================================
// Sensor Update Intervals
// =============================================================================
constexpr unsigned long PT100_UPDATE_INTERVAL_MS = 1000;
constexpr unsigned long DS18B20_UPDATE_INTERVAL_MS =
    1000; // 12-bit resolution needs 750ms conversion
constexpr unsigned long DS18B20_CONVERSION_TIME_MS =
    750; // 12-bit resolution conversion time
constexpr unsigned long DPS5015_UPDATE_INTERVAL_MS = 500;

// =============================================================================
// Modbus Communication Settings
// =============================================================================
// Default baud rate is 9600. If experiencing communication errors at high
// TEC power due to EMI from the DC lines, try:
// - Lowering baud rate to 4800 or 2400 (more robust but slower updates)
// - Adding ferrite cores to UART lines
// - Using shielded cables for Modbus connections
// - Increasing physical separation from high-current DC wiring
constexpr unsigned long MODBUS_BAUD_RATE = 9600;
constexpr unsigned long MODBUS_BYTE_TIMEOUT_MS = 10; // Per-byte timeout
constexpr int MODBUS_MAX_RETRIES = 3; // Consecutive failures before error state

// =============================================================================
// Display and Logging Settings
// =============================================================================
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;
constexpr unsigned long SPINNER_UPDATE_MS = 100;
constexpr int LOG_AREA_LINES = 2; // Lines reserved for live logs at bottom

// =============================================================================
// Watchdog Configuration
// =============================================================================
constexpr unsigned long WDT_TIMEOUT_SECONDS = 10; // Task WDT timeout

// =============================================================================
// WiFi Time Sync (boot-time, optional)
// =============================================================================
// If WIFI_SSID from env.h is present and reachable, the system will connect
// briefly on boot, sync wall time via NTP, then disconnect.
// If any step fails, wall time remains invalid and logging format is unchanged.
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000; // 10s connect timeout
constexpr unsigned long NTP_SYNC_TIMEOUT_MS = 8000;      // 8s NTP wait timeout

// NTP servers (UTC by default)
constexpr const char *NTP_SERVER_1 = "pool.ntp.org";
constexpr const char *NTP_SERVER_2 = "time.nist.gov";

// Timezone rule string (POSIX TZ). If empty, TIME_GMT_OFFSET_SEC and
// TIME_DAYLIGHT_OFFSET_SEC are used instead.
// Default is Central Europe (Poland): CET/CEST with DST rules.
constexpr const char *TIME_TZ_STRING = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// Timezone offsets for configTime (seconds). Keep 0 for UTC.
constexpr long TIME_GMT_OFFSET_SEC = 0;
constexpr int TIME_DAYLIGHT_OFFSET_SEC = 0;

// =============================================================================
// PT100 Sensor Error Detection
// =============================================================================
constexpr float PT100_ERROR_MIN_C = -100.0f; // Below this = sensor error
constexpr float PT100_ERROR_MAX_C = 500.0f;  // Above this = sensor error

// =============================================================================
// Thermal Controller - Voltage and Current Limits
// =============================================================================
constexpr float TEC_VOLTAGE_SETPOINT = 15.0f; // Fixed voltage for cascade TECs
constexpr float MAX_CURRENT_PER_CHANNEL = 12.0f; // Maximum current per DPS
constexpr float MIN_CURRENT_PER_CHANNEL = 0.5f;  // Minimum operational current
constexpr float STARTUP_CURRENT = 2.0f; // Initial current during startup
constexpr float DEGRADED_MODE_CURRENT =
    5.0f; // Max current when one DPS disconnects

// Hardware protection limits (failsafe in case of software/Modbus errors)
constexpr float DPS_OVP_LIMIT = 16.0f; // Over Voltage Protection (V)
constexpr float DPS_OCP_LIMIT = 13.0f; // Over Current Protection (A)

// =============================================================================
// Thermal Controller - Temperature Thresholds (Celsius)
// =============================================================================
constexpr float HOT_SIDE_WARNING_C = 55.0f;      // Reduce ramp aggressiveness
constexpr float HOT_SIDE_WARNING_EXIT_C = 50.0f; // Exit warning (hysteresis)
constexpr float HOT_SIDE_ALARM_C = 65.0f;        // Stop increasing current
constexpr float HOT_SIDE_ALARM_EXIT_C = 60.0f;   // Exit alarm (hysteresis)
constexpr float HOT_SIDE_FAULT_C = 70.0f;        // Emergency shutdown
constexpr float HOT_SIDE_PROBE_HEADROOM_C =
    5.0f; // Headroom below warning for probing

// Thermal runaway detection
constexpr float HOT_SIDE_RATE_FAULT_C_PER_MIN =
    5.0f; // Max acceptable rise rate

// Cooling performance thresholds
constexpr float COOLING_STALL_THRESHOLD_C = 0.5f; // Rate below this = stalled
constexpr float OVERCURRENT_WARMING_THRESHOLD_C = 0.3f; // Back off if warming
constexpr float COOLING_RATE_DEGRADATION_THRESHOLD = 0.3f; // K/min degradation
constexpr float MIN_CURRENT_FOR_STALL_CHECK_A =
    4.0f; // Don't check stall below this
constexpr float MIN_RAMP_CURRENT_BEFORE_EXIT_A =
    6.0f; // Min current before allowing ramp exit due to degradation
constexpr int CONSECUTIVE_DEGRADED_FOR_REVERT =
    2; // Require multiple degraded evals before reverting during ramp

// =============================================================================
// Thermal Controller - Timing Intervals (milliseconds)
// =============================================================================
constexpr unsigned long STARTUP_HOLD_DURATION_MS = 60000; // 60s startup
constexpr unsigned long RAMP_ADJUSTMENT_INTERVAL_MS =
    20000; // 20s between steps
constexpr unsigned long CURRENT_EVALUATION_DELAY_MS = 60000; // 60s evaluation
constexpr unsigned long STEADY_STATE_RECHECK_INTERVAL_MS =
    900000; // 15min recheck
constexpr unsigned long SENSOR_RECOVERY_TIMEOUT_MS =
    60000;                                       // 60s sensor recovery
constexpr unsigned long INIT_TIMEOUT_MS = 30000; // 30s init timeout

// Hot-side stabilization (thermal inertia compensation)
constexpr float HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN =
    0.3f; // Stable when below
constexpr unsigned long HOT_SIDE_STABILIZATION_MAX_WAIT_MS =
    300000; // 5 min max

// =============================================================================
// Thermal Controller - Adaptive Step Sizes
// =============================================================================
// Use coarse steps when far from optimum, fine steps when near optimum
constexpr float COARSE_STEP_A = 0.5f;  // Far from optimum (rate > 1 K/min)
constexpr float MEDIUM_STEP_A = 0.25f; // Approaching optimum (rate 0.5-1 K/min)
constexpr float FINE_STEP_A = 0.1f;    // Near optimum (rate < 0.5 K/min)

// Rate thresholds for step size selection
constexpr float STEP_COARSE_RATE_THRESHOLD = 1.0f; // K/min - above = coarse
constexpr float STEP_MEDIUM_RATE_THRESHOLD = 0.5f; // K/min - above = medium

// =============================================================================
// Thermal Controller - Manual Override Detection
// =============================================================================
constexpr float MANUAL_OVERRIDE_VOLTAGE_TOLERANCE_V = 0.15f;
constexpr float MANUAL_OVERRIDE_CURRENT_TOLERANCE_A = 0.15f;
constexpr unsigned long MANUAL_OVERRIDE_GRACE_MS = 3000; // Settling window
constexpr int MANUAL_OVERRIDE_MISMATCH_COUNT =
    3; // Consecutive mismatches needed

// =============================================================================
// Thermal Controller - Channel Imbalance Detection
// =============================================================================
constexpr float CHANNEL_CURRENT_IMBALANCE_A = 1.0f; // Log if |I1-I2| exceeds
constexpr float CHANNEL_POWER_IMBALANCE_W = 5.0f;   // Log if |P1-P2| exceeds

// =============================================================================
// Thermal Controller - Sensor Validation
// =============================================================================

// Cross-sensor validation (cold plate must be colder than hot plate)
constexpr float SENSOR_CROSS_CHECK_MARGIN_C = 2.0f;           // Minimum delta
constexpr unsigned long SENSOR_CROSS_CHECK_GRACE_MS = 120000; // 2 min grace
constexpr unsigned long SENSOR_CROSS_CHECK_LOG_INTERVAL_MS = 30000;

// Absolute sensor sanity limits
constexpr float COLD_PLATE_MIN_VALID_C = -60.0f; // PT100 can't read below this
constexpr float COLD_PLATE_MAX_VALID_C = 80.0f;  // Cold plate shouldn't be hot
constexpr float HOT_PLATE_MIN_VALID_C = -20.0f;  // Hot side shouldn't freeze
constexpr float HOT_PLATE_MAX_VALID_C = 100.0f;  // Above fault threshold

// PT100 plausibility check (physics-based validation)
constexpr float PLAUSIBILITY_MIN_DELTA_T_PER_AMP = 2.0f; // Min expected K/A
constexpr float PLAUSIBILITY_MAX_COLD_IMPLAUSIBLE_C = -35.0f; // Check if below
constexpr float PLAUSIBILITY_HOT_THRESHOLD_FOR_CHECK_C =
    35.0f;                                                    // Hot must exceed
constexpr unsigned long PLAUSIBILITY_CHECK_GRACE_MS = 180000; // 3 min grace

// =============================================================================
// NVS Metrics Persistence
// =============================================================================
// Write intervals are long to avoid wearing out flash (limited writes)
constexpr unsigned long NVS_METRICS_SAVE_INTERVAL_MS = 300000; // 5 minutes
constexpr unsigned long NVS_RUNTIME_SAVE_INTERVAL_MS = 60000;  // 1 minute

#endif // CONFIG_H
