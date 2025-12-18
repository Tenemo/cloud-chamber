/**
 * @file config.h
 * @brief Tunable configuration parameters
 *
 * Only namespaced configuration values live here. Use the namespaces directly
 * (e.g., `Limits::MAX_CURRENT_PER_CHANNEL`,
 * `Timing::STARTUP_HOLD_DURATION_MS`).
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "pins.h"
#include <cstddef>

// =============================================================================
// Feature Flags (0=disabled, 1=enabled)
// =============================================================================
// Can be overridden from build flags, e.g. `-D FEATURE_WIFI_TIME_SYNC=1`.
#ifndef FEATURE_WIFI_TIME_SYNC
#define FEATURE_WIFI_TIME_SYNC 0
#endif

// =============================================================================
// Sensor Update Intervals
// =============================================================================
namespace Intervals {
constexpr unsigned long PT100_UPDATE_INTERVAL_MS = 1000;
constexpr unsigned long DS18B20_UPDATE_INTERVAL_MS =
    1000; // 12-bit resolution needs 750ms conversion
constexpr unsigned long DS18B20_CONVERSION_TIME_MS =
    750; // 12-bit resolution conversion time
constexpr unsigned long DPS5015_UPDATE_INTERVAL_MS = 500;
} // namespace Intervals

// =============================================================================
// Modbus Communication Settings
// =============================================================================
namespace Modbus {
// Default baud rate is 9600. If experiencing communication errors at high
// TEC power due to EMI from the DC lines, try:
// - Lowering baud rate to 4800 or 2400 (more robust but slower updates)
// - Adding ferrite cores to UART lines
// - Using shielded cables for Modbus connections
// - Increasing physical separation from high-current DC wiring
constexpr unsigned long BAUD_RATE = 9600;
constexpr int MAX_RETRIES = 3; // Consecutive failures before error state
} // namespace Modbus

// =============================================================================
// Display and Logging Settings
// =============================================================================
namespace Display {
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;
constexpr unsigned long SPINNER_UPDATE_MS = 100;
constexpr int LOG_AREA_LINES = 2; // Lines reserved for live logs
} // namespace Display

namespace Labels {
constexpr const char *COLD_PLATE = "Cold plate";
constexpr const char *HOT_PLATE = "Hot plate";
constexpr const char *DELTA_T = "dT";
constexpr const char *RATE = "dT/dt";
constexpr const char *CURRENT = "I set";
} // namespace Labels

namespace Units {
constexpr const char *TEMP = "C";
constexpr const char *RATE = "K/m";
constexpr const char *CURRENT = "A";
constexpr const char *POWER = "W";
} // namespace Units

// =============================================================================
// Watchdog Configuration
// =============================================================================
namespace Watchdog {
constexpr unsigned long TIMEOUT_SECONDS = 10; // Task WDT timeout
} // namespace Watchdog

// =============================================================================
// WiFi Time Sync (boot-time, optional)
// =============================================================================
namespace WiFiTimeSync {
// If WIFI_SSID from env.h is present and reachable, the system will connect
// briefly on boot, sync wall time via NTP, then disconnect.
// If any step fails, wall time remains invalid and logging format is unchanged.
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000; // 10s connect timeout
constexpr unsigned long NTP_SYNC_TIMEOUT_MS = 8000;      // 8s NTP wait

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
} // namespace WiFiTimeSync

// =============================================================================
// PT100 Sensor Error Detection
// =============================================================================
namespace PT100 {
constexpr float ERROR_MIN_C = -100.0f; // Below this = sensor error
constexpr float ERROR_MAX_C = 500.0f;  // Above this = sensor error
} // namespace PT100

// =============================================================================
// Limits (hardware and safety limits)
// =============================================================================
namespace Limits {
// Fixed voltage for cascade TECs
constexpr float TEC_VOLTAGE_SETPOINT =
    16.1f; // TEMPORARY, LOWER TO 15V AFTER TESTING

// Current limits
constexpr float MAX_CURRENT_PER_CHANNEL =
    14.1f; // TEMPORARY, LOWER TO 13A AFTER TESTING
constexpr float MIN_CURRENT_PER_CHANNEL = 0.5f;
constexpr float STARTUP_CURRENT = 2.0f;
constexpr float DEGRADED_MODE_CURRENT =
    5.0f; // Max current when one DPS disconnects

// Hardware protection limits (failsafe in case of software/Modbus errors)
constexpr float DPS_OVP_LIMIT = 17.0f; // Over Voltage Protection (V) //
                                       // TEMPORARY, LOWER TO 16 V AFTER TESTING
constexpr float DPS_OCP_LIMIT = 15.0f; // Over Current Protection (A) //
                                       // TEMPORARY, LOWER TO 14A AFTER TESTING

// Temperature thresholds (Celsius)
constexpr float HOT_SIDE_WARNING_C = 55.0f;
constexpr float HOT_SIDE_WARNING_EXIT_C = 50.0f;
constexpr float HOT_SIDE_ALARM_C = 65.0f;
constexpr float HOT_SIDE_ALARM_EXIT_C = 60.0f;
constexpr float HOT_SIDE_FAULT_C = 70.0f;
constexpr float HOT_SIDE_RATE_FAULT_C_PER_MIN = 10.0f;
constexpr float HOT_SIDE_PROBE_HEADROOM_C = 5.0f;

struct ThermalLimit {
    float enter;
    float exit;
};

constexpr ThermalLimit HOT_WARNING_LIMIT{HOT_SIDE_WARNING_C,
                                         HOT_SIDE_WARNING_EXIT_C};
constexpr ThermalLimit HOT_ALARM_LIMIT{HOT_SIDE_ALARM_C, HOT_SIDE_ALARM_EXIT_C};

// Absolute sensor sanity limits
constexpr float COLD_PLATE_MIN_VALID_C = -60.0f;
constexpr float COLD_PLATE_MAX_VALID_C = 80.0f;
constexpr float HOT_PLATE_MIN_VALID_C = -20.0f;
constexpr float HOT_PLATE_MAX_VALID_C = 100.0f;
} // namespace Limits

// =============================================================================
// Timing (milliseconds)
// =============================================================================
namespace Timing {
constexpr unsigned long STARTUP_HOLD_DURATION_MS = 60000;
constexpr unsigned long RAMP_ADJUSTMENT_INTERVAL_MS = 20000;
constexpr unsigned long CURRENT_EVALUATION_DELAY_MS = 60000;
constexpr unsigned long STEADY_STATE_RECHECK_INTERVAL_MS =
    1UL * 60UL * 1000UL; // 1 min
constexpr unsigned long SENSOR_RECOVERY_TIMEOUT_MS = 60000;
constexpr unsigned long INIT_TIMEOUT_MS = 30000;

// Hot-side stabilization / thermal inertia compensation
constexpr unsigned long THERMAL_STABILIZATION_MS = 90000;
constexpr unsigned long HOT_SIDE_STABILIZATION_MAX_WAIT_MS = 300000;
constexpr unsigned long HOT_RESET_STABILIZATION_MS = THERMAL_STABILIZATION_MS;

// Manual override detection
constexpr unsigned long MANUAL_OVERRIDE_GRACE_MS = 3000;

// Cross-check / plausibility grace
constexpr unsigned long SENSOR_CROSS_CHECK_GRACE_MS = 120000;
constexpr unsigned long SENSOR_CROSS_CHECK_LOG_INTERVAL_MS = 30000;
constexpr unsigned long PLAUSIBILITY_CHECK_GRACE_MS = THERMAL_STABILIZATION_MS;
} // namespace Timing

// =============================================================================
// Tuning (algorithm thresholds and steps)
// =============================================================================
namespace Tuning {
// Cooling performance thresholds
constexpr float COOLING_STALL_THRESHOLD_C = 0.02f;
constexpr float OVERCURRENT_WARMING_THRESHOLD_C = 0.3f;
constexpr float COOLING_RATE_DEGRADATION_THRESHOLD = 0.1f;
constexpr float MIN_CURRENT_FOR_STALL_CHECK_A = 4.0f;
constexpr float MIN_RAMP_CURRENT_BEFORE_EXIT_A = 6.0f;
constexpr int CONSECUTIVE_DEGRADED_FOR_REVERT = 2;

// Hot-side "stable" threshold
constexpr float HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN = 0.3f;

// Adaptive step sizes (two-step approach: coarse for approach, fine for scan)
constexpr float COARSE_STEP_A = 1.0f;
constexpr float FINE_STEP_A = 0.1f;
constexpr float RAMP_COARSE_BELOW_A = 9.0f;
constexpr float STEP_COARSE_RATE_THRESHOLD = 1.0f; // K/min

// Manual override tolerances
constexpr float MANUAL_OVERRIDE_VOLTAGE_TOLERANCE_V = 0.15f;
constexpr float MANUAL_OVERRIDE_CURRENT_TOLERANCE_A = 0.15f;
constexpr int MANUAL_OVERRIDE_MISMATCH_COUNT = 3;

// Channel imbalance detection
constexpr float CHANNEL_CURRENT_IMBALANCE_A = 1.0f;
constexpr float CHANNEL_POWER_IMBALANCE_W = 5.0f;

// Sensor validation
constexpr float SENSOR_CROSS_CHECK_MARGIN_C = 2.0f;

// PT100 plausibility check (physics-based validation)
constexpr float PLAUSIBILITY_MIN_DELTA_T_PER_AMP = 2.0f;
constexpr float PLAUSIBILITY_MAX_COLD_IMPLAUSIBLE_C = -35.0f;
constexpr float PLAUSIBILITY_HOT_THRESHOLD_FOR_CHECK_C = 35.0f;
} // namespace Tuning

#endif // CONFIG_H
