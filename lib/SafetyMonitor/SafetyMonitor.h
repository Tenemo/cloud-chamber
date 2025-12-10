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
#include "ThermalMetrics.h"
#include "config.h"
#include <Arduino.h>

/**
 * @brief Result of a safety check
 *
 * Provides consistent error handling across all safety checks.
 */
enum class SafetyStatus {
    OK,               // All checks passed
    THERMAL_FAULT,    // Critical temperature exceeded
    SENSOR_FAULT,     // Sensor failure detected
    DPS_DISCONNECTED, // Lost communication with PSU
    MANUAL_OVERRIDE,  // User has taken control
    WARNING           // Non-critical issue (logged but not blocking)
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
     * and manual override checks. Does NOT include PT100 plausibility or
     * cross-sensor validation (which have grace period logic).
     *
     * Order: sensor health → sensor sanity → thermal limits → DPS → override
     *
     * @return SafetyResult with status and reason (if fault)
     */
    SafetyResult checkAll();

    /**
     * @brief Get the reason string for the last fault
     * @return Human-readable description of the fault
     */
    const char *getLastFaultReason() const { return _last_fault_reason; }

    /**
     * @brief Check thermal limits only
     * @return SafetyStatus::OK or SafetyStatus::THERMAL_FAULT
     */
    SafetyStatus checkThermalLimits();

    /**
     * @brief Check sensor health only
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
     * @param skip_check Whether to skip this check (e.g., during startup)
     * @return SafetyStatus::OK, WARNING, or THERMAL_FAULT
     */
    SafetyStatus checkPT100Plausibility(float avg_current, bool skip_check);

    /**
     * @brief Check cross-sensor validation
     * @param skip_check Whether to skip this check (e.g., during grace period)
     * @return SafetyStatus::OK or SafetyStatus::WARNING
     */
    SafetyStatus checkCrossSensorValidation(bool skip_check);

    /**
     * @brief Check DPS connection status
     * @return SafetyStatus::OK or SafetyStatus::DPS_DISCONNECTED
     */
    SafetyStatus checkDpsConnection();

    /**
     * @brief Set thermal metrics reference for rate-based checks
     * @param metrics Pointer to unified thermal metrics (includes history)
     */
    void setMetrics(const ThermalMetrics *metrics) { _metrics = metrics; }

    /**
     * @brief Update hot-side hysteresis state
     * Call once per update cycle to track warning/alarm states.
     */
    void updateHysteresis();

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

    // DPS connection state tracking
    bool _dps_was_connected;

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
};

#endif // SAFETY_MONITOR_H
