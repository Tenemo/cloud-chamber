/**
 * @file SafetyMonitor.h
 * @brief Centralized safety monitoring for thermal control system
 *
 * Consolidates all safety checks with consistent error handling:
 * - Thermal limits (hot side temperature)
 * - Sensor health (connection and error states)
 * - Sensor sanity (impossible value detection)
 * - PT100 plausibility (physics-based validation)
 * - Cross-sensor validation (cold must be colder than hot)
 * - DPS connection monitoring
 * - Manual override detection
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "DS18B20.h"
#include "DualPowerSupply.h"
#include "Logger.h"
#include "PT100.h"
#include "ThermalConstants.h"
#include "ThermalMetrics.h"
#include "ThermalTypes.h"

/**
 * @brief Result of a safety check
 *
 * Provides consistent error handling across all safety checks.
 * Ordered by severity (higher value = more severe) for priority comparison.
 */
enum class SafetyStatus {
    OK,               // All checks passed
    WARNING,          // Non-critical issue (logged but not blocking)
    SENSOR_FAULT,     // Sensor failure detected
    DPS_DISCONNECTED, // Lost communication with PSU
    THERMAL_FAULT     // Critical temperature exceeded (most severe)
};

/**
 * @brief Result of comprehensive safety check
 */
struct SafetyResult {
    SafetyStatus status;
    const char *reason; // Points to internal buffer, valid until next check
};

/**
 * @brief Centralized safety monitoring
 *
 * All safety checks return SafetyStatus for consistent handling.
 * The caller can then take appropriate action based on the result.
 */
class SafetyMonitor {
  public:
    SafetyMonitor(Logger &logger, PT100Sensor &coldPlate,
                  DS18B20Sensor &hotPlate, DualPowerSupply &dps);

    /**
     * @brief Run all safety checks in priority order
     *
     * Consolidates sensor health, sanity, thermal limits, DPS connection,
     * manual override, PT100 plausibility, and cross-sensor validation.
     * Grace periods are applied internally based on state context.
     *
     * Order: sensor health → sensor sanity → thermal limits → DPS → override
     *        → PT100 plausibility → cross-sensor validation
     *
     * @param current_state Current thermal controller state
     * @param ramp_start_time Time when RAMP_UP began (0 if not in RAMP_UP)
     * @param avg_current Average current for plausibility check
     * @return SafetyResult with status and reason (if fault)
     */
    SafetyResult checkAll(ThermalState current_state,
                          unsigned long ramp_start_time, float avg_current);

    /**
     * @brief Get the reason string for the last fault
     * @return Human-readable description of the fault
     */
    const char *getLastFaultReason() const { return _last_fault_reason; }

    /**
     * @brief Set thermal metrics reference for rate-based checks
     * @param metrics Pointer to unified thermal metrics (includes history)
     */
    void setMetrics(const ThermalMetrics *metrics) { _metrics = metrics; }

    /**
     * @brief Check if hot side is in warning zone (with hysteresis)
     */
    bool isHotSideWarning() const { return _hot_side_in_warning; }

    /**
     * @brief Check if hot side is in alarm zone (with hysteresis)
     */
    bool isHotSideAlarm() const { return _hot_side_in_alarm; }

  private:
    Logger &_logger;
    PT100Sensor &_cold_plate;
    DS18B20Sensor &_hot_plate;
    DualPowerSupply &_dps;
    const ThermalMetrics *_metrics;

    // Fault tracking
    char _last_fault_reason[48];

    // Hot-side hysteresis
    bool _hot_side_in_warning;
    bool _hot_side_in_alarm;

    // Cross-check warning state
    bool _cross_check_warning_active;
    unsigned long _last_cross_check_log_time;

    /**
     * @brief Set fault reason and return status
     */
    SafetyStatus setFault(SafetyStatus status, const char *reason);

    /**
     * @brief Update hot-side hysteresis state
     */
    void updateHysteresis();

    /**
     * @brief Check thermal limits
     * @return SafetyStatus::OK or SafetyStatus::THERMAL_FAULT
     */
    SafetyStatus checkThermalLimits();

    /**
     * @brief Check sensor health
     * @return SafetyStatus::OK or SafetyStatus::SENSOR_FAULT
     */
    SafetyStatus checkSensorHealth();

    /**
     * @brief Check sensor sanity (impossible values)
     * @return SafetyStatus::OK or SafetyStatus::SENSOR_FAULT
     */
    SafetyStatus checkSensorSanity();

    /**
     * @brief Check PT100 plausibility (physics-based)
     * @param avg_current Average current across channels
     * @return SafetyStatus::OK, WARNING, or THERMAL_FAULT
     */
    SafetyStatus checkPT100Plausibility(float avg_current);

    /**
     * @brief Log cross-sensor validation warnings
     * @param skip_check Whether to skip this check
     */
    void logCrossSensorWarnings(bool skip_check);

    /**
     * @brief Check DPS connection status
     * @return SafetyStatus::OK or SafetyStatus::DPS_DISCONNECTED
     */
    SafetyStatus checkDpsConnection();
};

#endif // SAFETY_MONITOR_H
