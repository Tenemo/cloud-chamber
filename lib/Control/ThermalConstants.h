/**
 * @file ThermalConstants.h
 * @brief Implementation constants for ThermalController
 *
 * This file contains constants that are implementation details and
 * typically don't need modification during testing or tuning:
 * - Derived values (calculated from config.h values)
 * - Buffer sizes and validation
 * - Internal timing constants
 * - Display line IDs
 * - Tolerance values
 *
 * For tunable parameters (thresholds, timing, limits), see config.h
 */

#ifndef THERMAL_CONSTANTS_H
#define THERMAL_CONSTANTS_H

#include "config.h"
#include <cstddef>

// =============================================================================
// Derived Constants (calculated from config.h values)
// =============================================================================

// Threshold for skipping ramp after hot reset (near max current)
constexpr float HOT_RESET_NEAR_MAX_A = MAX_CURRENT_PER_CHANNEL - 0.5f;

// Hot reset detection threshold
constexpr float HOT_RESET_CURRENT_THRESHOLD_A = STARTUP_CURRENT;

// =============================================================================
// History Buffer Configuration
// =============================================================================

constexpr size_t HISTORY_BUFFER_SIZE = 300;                // 5 minutes at 1Hz
constexpr unsigned long HISTORY_SAMPLE_INTERVAL_MS = 1000; // 1 second samples
constexpr size_t COOLING_RATE_WINDOW_SAMPLES = 30; // 30s window for cold plate
constexpr size_t HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES =
    60; // 60s window for hot side

// Buffer size validation
static_assert(HISTORY_BUFFER_SIZE * HISTORY_SAMPLE_INTERVAL_MS == 300000,
              "History buffer does not hold exactly 5 minutes of data");
static_assert(COOLING_RATE_WINDOW_SAMPLES <= HISTORY_BUFFER_SIZE,
              "Cooling rate window exceeds buffer size");
static_assert(HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES <= HISTORY_BUFFER_SIZE,
              "Hot side rate window exceeds buffer size");

// =============================================================================
// Display Line IDs
// =============================================================================

namespace ThermalDisplay {
constexpr const char *LINE_STATE = "TC_STATE";
constexpr const char *LINE_RATE = "TC_RATE";
constexpr const char *LINE_CURRENT = "TC_I";
} // namespace ThermalDisplay

// =============================================================================
// Tolerance Constants
// =============================================================================

namespace Tolerance {
constexpr float MIN_CURRENT_THRESHOLD = 0.1f;        // Output considered "off"
constexpr float NVS_FLOAT_TOLERANCE = 0.01f;         // NVS comparison tolerance
constexpr float TEMP_IMPROVEMENT_THRESHOLD_C = 0.1f; // Min improvement to count
} // namespace Tolerance

// =============================================================================
// Internal Timing Constants
// =============================================================================

namespace Timing {
// Emergency shutdown
constexpr unsigned long SHUTDOWN_STEP_MS = 500;
constexpr float EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC = 2.0f;

// Rate limiting
constexpr unsigned long IMBALANCE_LOG_INTERVAL_MS = 60000;

// DPS self-test
constexpr unsigned long SELFTEST_TIMEOUT_MS = 3000;
constexpr unsigned long SELFTEST_SETTLE_MS = 500;

// Startup configuration
constexpr unsigned long STARTUP_CONFIG_WINDOW_MS = 500;
constexpr unsigned long STARTUP_CONFIG_SEND_MS = 50;
} // namespace Timing

// =============================================================================
// Utility Functions
// =============================================================================

namespace Clamp {
/**
 * @brief Clamp current to valid operational range
 */
inline float current(float val) {
    if (val < MIN_CURRENT_PER_CHANNEL)
        return MIN_CURRENT_PER_CHANNEL;
    if (val > MAX_CURRENT_PER_CHANNEL)
        return MAX_CURRENT_PER_CHANNEL;
    return val;
}
} // namespace Clamp

#endif // THERMAL_CONSTANTS_H
