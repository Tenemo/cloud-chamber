/**
 * @file SafetyMonitor.cpp
 * @brief Implementation of centralized safety monitoring
 */

#include "SafetyMonitor.h"
#include <cmath>
#include <cstring>

SafetyMonitor::SafetyMonitor(Logger &logger, PT100Sensor &coldPlate,
                             DS18B20Sensor &hotPlate, DPS5015 *psus)
    : _logger(logger), _cold_plate(coldPlate), _hot_plate(hotPlate),
      _psus(psus), _history(nullptr), _dps_was_connected{false, false},
      _hot_side_in_warning(false), _hot_side_in_alarm(false),
      _cross_check_warning_active(false), _last_cross_check_log_time(0) {
    _last_fault_reason[0] = '\0';
}

SafetyStatus SafetyMonitor::setFault(SafetyStatus status, const char *reason) {
    strncpy(_last_fault_reason, reason, sizeof(_last_fault_reason) - 1);
    _last_fault_reason[sizeof(_last_fault_reason) - 1] = '\0';
    return status;
}

SafetyStatus SafetyMonitor::runAllChecks() {
    // Run checks in priority order (most critical first)
    SafetyStatus status;

    status = checkThermalLimits();
    if (status != SafetyStatus::OK)
        return status;

    status = checkSensorHealth();
    if (status != SafetyStatus::OK)
        return status;

    status = checkSensorSanity();
    if (status != SafetyStatus::OK)
        return status;

    status = checkDpsConnection();
    if (status != SafetyStatus::OK)
        return status;

    status = checkManualOverride();
    if (status != SafetyStatus::OK)
        return status;

    return SafetyStatus::OK;
}

void SafetyMonitor::updateHysteresis() {
    float hot_temp = _hot_plate.getTemperature();

    // Update alarm state with hysteresis
    if (_hot_side_in_alarm) {
        if (hot_temp < HOT_SIDE_ALARM_EXIT_C)
            _hot_side_in_alarm = false;
    } else {
        if (hot_temp >= HOT_SIDE_ALARM_C)
            _hot_side_in_alarm = true;
    }

    // Update warning state with hysteresis
    if (_hot_side_in_warning) {
        if (hot_temp < HOT_SIDE_WARNING_EXIT_C)
            _hot_side_in_warning = false;
    } else {
        if (hot_temp >= HOT_SIDE_WARNING_C)
            _hot_side_in_warning = true;
    }
}

SafetyStatus SafetyMonitor::checkThermalLimits() {
    // Hot-side sensor fault is an immediate safety concern
    if (!_hot_plate.isConnected()) {
        return setFault(SafetyStatus::THERMAL_FAULT, "HOT SENSOR LOST");
    }

    float hot_temp = _hot_plate.getTemperature();

    // Critical hot side fault
    if (hot_temp >= HOT_SIDE_FAULT_C) {
        char buf[32];
        snprintf(buf, sizeof(buf), "HOT>%.0fC", hot_temp);
        return setFault(SafetyStatus::THERMAL_FAULT, buf);
    }

    // Hot side rate check (thermal runaway detection)
    if (_history != nullptr && _history->hasMinimumHistory(60)) {
        float hot_rate = _history->getHotPlateRate();
        if (hot_rate > HOT_SIDE_RATE_FAULT_C_PER_MIN) {
            return setFault(SafetyStatus::THERMAL_FAULT, "RUNAWAY");
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

SafetyStatus SafetyMonitor::checkPT100Plausibility(float avg_current,
                                                   bool skip_check) {
    if (skip_check) {
        return SafetyStatus::OK;
    }

    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Only check if cold plate reports very low temperature
    if (cold_temp > PLAUSIBILITY_MAX_COLD_IMPLAUSIBLE_C) {
        return SafetyStatus::OK;
    }

    // If cold plate is extremely cold, hot side MUST be significantly warm
    if (hot_temp < PLAUSIBILITY_HOT_THRESHOLD_FOR_CHECK_C) {
        char buf[64];
        snprintf(buf, sizeof(buf), "PT100 implausible! C=%.1f H=%.1f",
                 cold_temp, hot_temp);
        _logger.log(buf);

        // Check if current draw supports the cold temperature claim
        float expected_min_delta =
            avg_current * PLAUSIBILITY_MIN_DELTA_T_PER_AMP;
        float actual_delta = hot_temp - cold_temp;

        if (actual_delta > expected_min_delta * 3) {
            return setFault(SafetyStatus::THERMAL_FAULT, "PT100 failed low");
        }

        return SafetyStatus::WARNING;
    }

    return SafetyStatus::OK;
}

SafetyStatus SafetyMonitor::checkCrossSensorValidation(bool skip_check) {
    if (skip_check) {
        return SafetyStatus::OK;
    }

    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();
    unsigned long now = millis();

    // Cold plate should be significantly colder than hot plate
    if (cold_temp >= hot_temp - SENSOR_CROSS_CHECK_MARGIN_C) {
        if (!_cross_check_warning_active) {
            _cross_check_warning_active = true;
            char buf[48];
            snprintf(buf, sizeof(buf), "WARN: Cold>=Hot! C=%.1f H=%.1f",
                     cold_temp, hot_temp);
            _logger.log(buf);
            _last_cross_check_log_time = now;
        } else if (now - _last_cross_check_log_time >=
                   SENSOR_CROSS_CHECK_LOG_INTERVAL_MS) {
            char buf[48];
            snprintf(buf, sizeof(buf), "WARN: Cold>=Hot C=%.1f H=%.1f",
                     cold_temp, hot_temp);
            _logger.log(buf);
            _last_cross_check_log_time = now;
        }

        return SafetyStatus::WARNING;
    } else {
        if (_cross_check_warning_active) {
            _cross_check_warning_active = false;
            _logger.log("Cross-check OK: Cold<Hot");
        }
    }

    return SafetyStatus::OK;
}

SafetyStatus SafetyMonitor::checkDpsConnection() {
    bool psu0_ok = _psus[0].isConnected();
    bool psu1_ok = _psus[1].isConnected();

    // Track connection state
    bool was0 = _dps_was_connected[0];
    bool was1 = _dps_was_connected[1];
    _dps_was_connected[0] = psu0_ok;
    _dps_was_connected[1] = psu1_ok;

    // Detect disconnection
    if ((was0 && !psu0_ok) || (was1 && !psu1_ok)) {
        return setFault(SafetyStatus::DPS_DISCONNECTED, "DPS disconnected");
    }

    return SafetyStatus::OK;
}

SafetyStatus SafetyMonitor::checkManualOverride() {
    for (size_t i = 0; i < 2; i++) {
        if (!_psus[i].isConnected())
            continue;

        // Skip check if there are pending writes or we're in grace period
        if (_psus[i].hasPendingWrites() || _psus[i].isInGracePeriod())
            continue;

        // Check for sustained current mismatch
        if (_psus[i].getConsecutiveMismatches() >=
            MANUAL_OVERRIDE_MISMATCH_COUNT) {
            return setFault(SafetyStatus::MANUAL_OVERRIDE,
                            "Manual override (I)");
        }

        // Voltage mismatch (immediate)
        if (_psus[i].hasVoltageMismatch()) {
            char buf[48];
            snprintf(buf, sizeof(buf), "Manual override (V) %.1f!=%.1f",
                     _psus[i].getSetVoltage(), _psus[i].getCommandedVoltage());
            return setFault(SafetyStatus::MANUAL_OVERRIDE, buf);
        }

        // Output state mismatch (immediate)
        if (_psus[i].hasOutputMismatch()) {
            return setFault(SafetyStatus::MANUAL_OVERRIDE, "Output toggled");
        }
    }

    return SafetyStatus::OK;
}
