/**
 * @file ThermalConstants.h
 * @brief Internal constants for ThermalController
 *
 * These are stable constants that typically don't need modification during
 * testing or tuning. They define physical constraints and implementation
 * details that should rarely change.
 *
 * For tunable parameters (thresholds, timing, etc.), see config.h
 */

#ifndef THERMAL_CONSTANTS_H
#define THERMAL_CONSTANTS_H

#include "config.h"
#include <cstddef>

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
// Display Line IDs (internal)
// =============================================================================

namespace ThermalDisplay {
constexpr const char *LINE_STATE = "TC_STATE";
constexpr const char *LINE_RATE = "TC_RATE";
constexpr const char *LINE_I1_SET = "TC_I1";
constexpr const char *LINE_I2_SET = "TC_I2";
} // namespace ThermalDisplay

// =============================================================================
// Self-Test Phase Constants
// =============================================================================

namespace SelfTest {
constexpr int PHASE_START = 0;
constexpr int PHASE_CHECK_SETTINGS = 1;
constexpr int PHASE_ENABLE_PSU0 = 2;
constexpr int PHASE_VERIFY_PSU0 = 3;
constexpr int PHASE_ENABLE_PSU1 = 4;
constexpr int PHASE_VERIFY_PSU1 = 5;
constexpr int PHASE_COMPLETE = 6;
} // namespace SelfTest

// =============================================================================
// Tolerance Constants (rarely need tuning)
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
} // namespace Timing

#endif // THERMAL_CONSTANTS_H
