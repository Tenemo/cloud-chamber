/**
 * @file SafetyMonitor.cpp
 * @brief Implementation of centralized safety monitoring
 */

#include "SafetyMonitor.h"
#include <cmath>
#include <cstring>

SafetyMonitor::SafetyMonitor(Logger &logger, PT100Sensor &coldPlate,
                             DS18B20Sensor &hotPlate, DualPowerSupply &dps)
    : _logger(logger), _cold_plate(coldPlate), _hot_plate(hotPlate), _dps(dps),
      _metrics(nullptr), _hot_side_in_warning(false), _hot_side_in_alarm(false),
      _cross_check_warning_active(false), _last_cross_check_log_time(0) {
    _last_fault_reason[0] = '\0';
}

SafetyResult SafetyMonitor::checkAll(ThermalState current_state,
                                     unsigned long ramp_start_time,
                                     float avg_current) {
    // Update hysteresis state before running checks
    updateHysteresis();

    // Run all checks and track the most severe fault
    // Priority: THERMAL_FAULT > DPS_DISCONNECTED > SENSOR_FAULT
    SafetyStatus worst_status = SafetyStatus::OK;
    const char *worst_reason = nullptr;

    // Helper to update worst status (higher enum value = more severe)
    auto updateWorst = [&](SafetyStatus status, const char *reason) {
        if (static_cast<int>(status) > static_cast<int>(worst_status)) {
            worst_status = status;
            worst_reason = reason;
        }
    };

    // Check sensor health
    SafetyStatus status = checkSensorHealth();
    if (status != SafetyStatus::OK)
        updateWorst(status, _last_fault_reason);

    // Check sensor sanity
    status = checkSensorSanity();
    if (status != SafetyStatus::OK)
        updateWorst(status, _last_fault_reason);

    // Check thermal limits
    status = checkThermalLimits();
    if (status != SafetyStatus::OK)
        updateWorst(status, _last_fault_reason);

    // Check DPS connection
    status = checkDpsConnection();
    if (status != SafetyStatus::OK)
        updateWorst(status, _last_fault_reason);

    // Return the most severe fault found
    if (worst_status != SafetyStatus::OK) {
        // Restore worst reason to _last_fault_reason for caller
        if (worst_reason) {
            strncpy(_last_fault_reason, worst_reason,
                    sizeof(_last_fault_reason) - 1);
            _last_fault_reason[sizeof(_last_fault_reason) - 1] = '\0';
        }
        return {worst_status, _last_fault_reason};
    }

    // PT100 plausibility check with grace period
    // Skip during STARTUP and early RAMP_UP (first 3 minutes)
    unsigned long now = millis();
    bool in_early_ramp =
        (current_state == ThermalState::RAMP_UP &&
         (now - ramp_start_time) < PLAUSIBILITY_CHECK_GRACE_MS);
    bool skip_plausibility =
        (current_state == ThermalState::STARTUP || in_early_ramp);

    if (!skip_plausibility) {
        status = checkPT100Plausibility(avg_current);
        if (status == SafetyStatus::THERMAL_FAULT) {
            return {status, _last_fault_reason};
        }
    }

    // Cross-sensor validation with grace period (warning-only, no state change)
    // Skip during SELF_TEST, STARTUP, and early RAMP_UP when cold=hot is
    // expected
    bool in_cross_check_grace =
        (current_state == ThermalState::SELF_TEST ||
         current_state == ThermalState::STARTUP ||
         (current_state == ThermalState::RAMP_UP &&
          (now - ramp_start_time) < SENSOR_CROSS_CHECK_GRACE_MS));
    logCrossSensorWarnings(in_cross_check_grace);

    return {SafetyStatus::OK, nullptr};
}

SafetyStatus SafetyMonitor::setFault(SafetyStatus status, const char *reason) {
    strncpy(_last_fault_reason, reason, sizeof(_last_fault_reason) - 1);
    _last_fault_reason[sizeof(_last_fault_reason) - 1] = '\0';
    return status;
}

// Helper function to update a hysteresis state
static inline void updateHysteresisState(bool &state, float value,
                                         const ThermalLimit &limit) {
    if (state) {
        if (value < limit.exit)
            state = false;
    } else {
        if (value >= limit.enter)
            state = true;
    }
}

void SafetyMonitor::updateHysteresis() {
    float hot_temp = _hot_plate.getTemperature();

    // Update alarm and warning states with hysteresis
    updateHysteresisState(_hot_side_in_alarm, hot_temp, HOT_ALARM_LIMIT);
    updateHysteresisState(_hot_side_in_warning, hot_temp, HOT_WARNING_LIMIT);
}

SafetyStatus SafetyMonitor::checkThermalLimits() {
    // Note: Sensor connectivity is checked by checkSensorHealth() which runs
    // first This function assumes hot_plate is connected and focuses on
    // temperature limits
    float hot_temp = _hot_plate.getTemperature();

    // Critical hot side fault
    if (hot_temp >= HOT_SIDE_FAULT_C) {
        char buf[32];
        snprintf(buf, sizeof(buf), "HOT>%.0fC", hot_temp);
        return setFault(SafetyStatus::THERMAL_FAULT, buf);
    }

    // Hot side rate check (thermal runaway detection)
    // Note: _metrics must be set via setMetrics() for runaway detection to work
    if (_metrics != nullptr && _metrics->hasMinimumHistory(60)) {
        float hot_rate = _metrics->getHotPlateRate();
        if (hot_rate > HOT_SIDE_RATE_FAULT_C_PER_MIN) {
            return setFault(SafetyStatus::THERMAL_FAULT, "RUNAWAY");
        }
    } else if (_metrics == nullptr) {
        // Log warning once if metrics not configured (runaway detection
        // disabled)
        static bool warned_once = false;
        if (!warned_once) {
            _logger.log("WARN: Metrics null - runaway detection disabled");
            warned_once = true;
        }
    }

    return SafetyStatus::OK;
}

SafetyStatus SafetyMonitor::checkSensorHealth() {
    bool cold_ok = !_cold_plate.isInError();
    bool hot_ok = _hot_plate.isConnected();

    if (!cold_ok) {
        return setFault(SafetyStatus::SENSOR_FAULT, "PT100 error");
    }
    if (!hot_ok) {
        return setFault(SafetyStatus::SENSOR_FAULT, "Hot plate disconnected");
    }

    return SafetyStatus::OK;
}

SafetyStatus SafetyMonitor::checkSensorSanity() {
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Check cold plate sanity
    if (cold_temp < COLD_PLATE_MIN_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Cold=%.1fC impossible", cold_temp);
        return setFault(SafetyStatus::SENSOR_FAULT, buf);
    }
    if (cold_temp > COLD_PLATE_MAX_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Cold=%.1fC too hot", cold_temp);
        return setFault(SafetyStatus::SENSOR_FAULT, buf);
    }

    // Check hot plate sanity
    if (hot_temp < HOT_PLATE_MIN_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Hot=%.1fC impossible", hot_temp);
        return setFault(SafetyStatus::SENSOR_FAULT, buf);
    }
    if (hot_temp > HOT_PLATE_MAX_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Hot=%.1fC extreme", hot_temp);
        return setFault(SafetyStatus::SENSOR_FAULT, buf);
    }

    return SafetyStatus::OK;
}

SafetyStatus SafetyMonitor::checkPT100Plausibility(float avg_current) {
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Only check if cold plate reports very low temperature
    if (cold_temp > PLAUSIBILITY_MAX_COLD_IMPLAUSIBLE_C) {
        return SafetyStatus::OK;
    }

    // Very low current means delta expectation is meaningless (startup/idle)
    if (avg_current < MIN_CURRENT_FOR_STALL_CHECK_A) {
        return SafetyStatus::OK;
    }

    // If cold plate is extremely cold, hot side MUST be significantly warm
    if (hot_temp < PLAUSIBILITY_HOT_THRESHOLD_FOR_CHECK_C) {
        _logger.logf("PT100 implausible! C=%.1f H=%.1f", cold_temp, hot_temp);

        // Check if current draw supports the cold temperature claim
        // If we're drawing significant current, we expect a temperature delta
        float expected_min_delta =
            avg_current * PLAUSIBILITY_MIN_DELTA_T_PER_AMP;
        float actual_delta = hot_temp - cold_temp;

        // Fault if delta is implausibly SMALL - PT100 claims extreme cold but
        // hot side shows no corresponding heat buildup (sensor failure)
        if (actual_delta < expected_min_delta) {
            return setFault(SafetyStatus::THERMAL_FAULT, "PT100 failed low");
        }

        return SafetyStatus::WARNING;
    }

    return SafetyStatus::OK;
}

void SafetyMonitor::logCrossSensorWarnings(bool skip_check) {
    if (skip_check)
        return;

    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();
    unsigned long now = millis();

    // Cold plate should be significantly colder than hot plate
    if (cold_temp >= hot_temp - SENSOR_CROSS_CHECK_MARGIN_C) {
        if (!_cross_check_warning_active) {
            _cross_check_warning_active = true;
            _logger.logf("WARN: Cold>=Hot! C=%.1f H=%.1f", cold_temp, hot_temp);
            _last_cross_check_log_time = now;
        } else if (now - _last_cross_check_log_time >=
                   SENSOR_CROSS_CHECK_LOG_INTERVAL_MS) {
            _logger.logf("WARN: Cold>=Hot C=%.1f H=%.1f", cold_temp, hot_temp);
            _last_cross_check_log_time = now;
        }
    } else {
        if (_cross_check_warning_active) {
            _cross_check_warning_active = false;
            _logger.log("Cross-check OK: Cold<Hot");
        }
    }
}

SafetyStatus SafetyMonitor::checkDpsConnection() {
    // Check for asymmetric failure (one connected, one not)
    // This is the critical failure mode we need to catch
    if (_dps.isAsymmetricFailure()) {
        return setFault(SafetyStatus::DPS_DISCONNECTED, "DPS asymmetric");
    }

    // Both disconnected is also a problem
    if (!_dps.isEitherConnected()) {
        return setFault(SafetyStatus::DPS_DISCONNECTED, "DPS disconnected");
    }

    return SafetyStatus::OK;
}
