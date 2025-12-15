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
// History Buffer Configuration
// =============================================================================

constexpr size_t HISTORY_BUFFER_SIZE = 1800;               // 30 minutes at 1Hz
constexpr unsigned long HISTORY_SAMPLE_INTERVAL_MS = 1000; // 1 second samples
constexpr size_t COOLING_RATE_WINDOW_SAMPLES = 60; // 60s window for cold plate
constexpr size_t HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES =
    60; // 60s window for hot side

// Buffer size validation
static_assert(HISTORY_BUFFER_SIZE * HISTORY_SAMPLE_INTERVAL_MS == 1800000,
              "History buffer does not hold exactly 30 minutes of data");
static_assert(COOLING_RATE_WINDOW_SAMPLES <= HISTORY_BUFFER_SIZE,
              "Cooling rate window exceeds buffer size");
static_assert(HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES <= HISTORY_BUFFER_SIZE,
              "Hot side rate window exceeds buffer size");

// =============================================================================
// Derived Constants (calculated from config.h values)
// =============================================================================

// Threshold for skipping ramp after hot reset (near max current)
constexpr float HOT_RESET_NEAR_MAX_A = Limits::MAX_CURRENT_PER_CHANNEL - 0.5f;

// Hot reset detection threshold (skip self-test if DPS running above this)
// Low threshold catches any meaningful TEC operation for seamless recovery
constexpr float HOT_RESET_CURRENT_THRESHOLD_A = 0.5f;

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
} // namespace Timing

// =============================================================================
// Utility Functions
// =============================================================================

namespace Clamp {
/**
 * @brief Clamp current to valid operational range
 */
inline float current(float val) {
    if (val < Limits::MIN_CURRENT_PER_CHANNEL)
        return Limits::MIN_CURRENT_PER_CHANNEL;
    if (val > Limits::MAX_CURRENT_PER_CHANNEL)
        return Limits::MAX_CURRENT_PER_CHANNEL;
    return val;
}
} // namespace Clamp

#endif // THERMAL_CONSTANTS_H
