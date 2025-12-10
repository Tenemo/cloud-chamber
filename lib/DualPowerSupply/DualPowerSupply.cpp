/**
 * @file DualPowerSupply.cpp
 * @brief Implementation of symmetric dual PSU control
 */

#include "DualPowerSupply.h"
#include "CrashLog.h"
#include <cmath>

// =============================================================================
// Constructor and Initialization
// =============================================================================

DualPowerSupply::DualPowerSupply(Logger &logger, DPS5015 &psu0, DPS5015 &psu1)
    : _logger(logger), _psu0(psu0), _psu1(psu1), _target_current(0.0f),
      _target_voltage(0.0f), _target_output(false),
      _shutdown_in_progress(false), _shutdown_current(0.0f),
      _last_shutdown_step_time(0), _consecutive_mismatches(0) {}

void DualPowerSupply::begin() {
    // Nothing special needed - PSUs are initialized separately
}

void DualPowerSupply::update() {
    // Currently no periodic updates needed
    // PSU update() is called separately in main loop
}

// =============================================================================
// Symmetric Control
// =============================================================================

bool DualPowerSupply::setSymmetricCurrent(float current) {
    // Clamp to valid range
    current =
        fmax(MIN_CURRENT_PER_CHANNEL, fmin(current, MAX_CURRENT_PER_CHANNEL));

    _target_current = current;

    bool success = true;
    if (_psu0.isConnected()) {
        success &= _psu0.setCurrent(current);
    }
    if (_psu1.isConnected()) {
        success &= _psu1.setCurrent(current);
    }

    return success;
}

bool DualPowerSupply::setSymmetricVoltage(float voltage) {
    _target_voltage = voltage;

    bool success = true;
    if (_psu0.isConnected()) {
        success &= _psu0.setVoltage(voltage);
    }
    if (_psu1.isConnected()) {
        success &= _psu1.setVoltage(voltage);
    }

    return success;
}

bool DualPowerSupply::enableOutput() {
    _target_output = true;

    bool success = true;
    if (_psu0.isConnected()) {
        success &= _psu0.setOutput(true);
    }
    if (_psu1.isConnected()) {
        success &= _psu1.setOutput(true);
    }

    return success;
}

bool DualPowerSupply::disableOutput() {
    _target_output = false;

    bool success = true;
    success &= _psu0.disableOutput();
    success &= _psu1.disableOutput();

    return success;
}

void DualPowerSupply::configure(float voltage, float current, bool outputOn) {
    _target_voltage = voltage;
    _target_current = current;
    _target_output = outputOn;

    _psu0.configure(voltage, current, outputOn);
    _psu1.configure(voltage, current, outputOn);
}

// =============================================================================
// Emergency Shutdown
// =============================================================================

void DualPowerSupply::startEmergencyShutdown() {
    if (_shutdown_in_progress)
        return;

    _logger.log("DPS: Emergency shutdown");
    CrashLog::logCritical("SHUTDOWN", "Emergency ramp-down");

    _shutdown_in_progress = true;
    _shutdown_current =
        fmax(_psu0.getOutputCurrent(), _psu1.getOutputCurrent());
    if (_shutdown_current < _target_current) {
        _shutdown_current = _target_current;
    }
    _last_shutdown_step_time = millis();
}

bool DualPowerSupply::updateEmergencyShutdown() {
    if (!_shutdown_in_progress)
        return false;

    unsigned long now = millis();
    if (now - _last_shutdown_step_time < SHUTDOWN_STEP_MS)
        return true;

    _last_shutdown_step_time = now;

    float step = EMERGENCY_RAMP_RATE_A_PER_SEC * (SHUTDOWN_STEP_MS / 1000.0f);
    _shutdown_current -= step;

    if (_shutdown_current <= 0.1f) {
        // Final shutdown
        _psu0.disableOutput();
        _psu1.disableOutput();
        _target_current = 0;
        _target_output = false;
        _shutdown_in_progress = false;
        _logger.log("DPS: Shutdown complete");
        CrashLog::logCritical("SHUTDOWN", "Complete");
        return false;
    } else {
        // Continue ramp-down with immediate writes
        _psu0.setCurrentImmediate(_shutdown_current);
        _psu1.setCurrentImmediate(_shutdown_current);
        return true;
    }
}

bool DualPowerSupply::hardShutdown() {
    _logger.log("DPS: HARD CUT!");
    CrashLog::logCritical("SHUTDOWN", "Hard cut");

    _shutdown_in_progress = false;
    _target_current = 0;
    _target_output = false;

    bool success = true;
    success &= _psu0.disableOutput();
    success &= _psu1.disableOutput();

    if (!success) {
        _logger.log("CRIT: PSU shutdown FAIL!");
    }

    return success;
}

// =============================================================================
// Status Queries
// =============================================================================

bool DualPowerSupply::areBothConnected() const {
    return _psu0.isConnected() && _psu1.isConnected();
}

bool DualPowerSupply::isEitherConnected() const {
    return _psu0.isConnected() || _psu1.isConnected();
}

bool DualPowerSupply::isAsymmetricFailure() const {
    return _psu0.isConnected() != _psu1.isConnected();
}

OverrideStatus DualPowerSupply::checkManualOverride() const {
    // Skip check if not connected or in grace period
    if (!areBothConnected())
        return OverrideStatus::NONE;

    if (_psu0.isInGracePeriod() || _psu1.isInGracePeriod())
        return OverrideStatus::NONE;

    if (_psu0.hasPendingWrites() || _psu1.hasPendingWrites())
        return OverrideStatus::NONE;

    // Check for current mismatch on either channel
    bool mismatch0 = _psu0.hasCurrentMismatch();
    bool mismatch1 = _psu1.hasCurrentMismatch();

    if (mismatch0 || mismatch1) {
        _consecutive_mismatches++;

        if (_consecutive_mismatches >= OVERRIDE_CONFIRM_COUNT) {
            return OverrideStatus::DETECTED;
        }
        return OverrideStatus::PENDING;
    }

    // No mismatch - reset counter
    _consecutive_mismatches = 0;
    return OverrideStatus::NONE;
}

bool DualPowerSupply::isOutputOn() const {
    return _psu0.isOutputOn() || _psu1.isOutputOn();
}

float DualPowerSupply::getAverageOutputCurrent() const {
    float sum = 0.0f;
    int count = 0;

    if (_psu0.isConnected()) {
        sum += _psu0.getOutputCurrent();
        count++;
    }
    if (_psu1.isConnected()) {
        sum += _psu1.getOutputCurrent();
        count++;
    }

    return count > 0 ? (sum / count) : 0.0f;
}

float DualPowerSupply::getTotalPower() const {
    float power = 0.0f;
    if (_psu0.isConnected())
        power += _psu0.getOutputPower();
    if (_psu1.isConnected())
        power += _psu1.getOutputPower();
    return power;
}

float DualPowerSupply::getOutputCurrent(size_t channel) const {
    if (channel == 0)
        return _psu0.getOutputCurrent();
    if (channel == 1)
        return _psu1.getOutputCurrent();
    return 0.0f;
}

float DualPowerSupply::getSetCurrent(size_t channel) const {
    if (channel == 0)
        return _psu0.getSetCurrent();
    if (channel == 1)
        return _psu1.getSetCurrent();
    return 0.0f;
}

bool DualPowerSupply::isConnected(size_t channel) const {
    if (channel == 0)
        return _psu0.isConnected();
    if (channel == 1)
        return _psu1.isConnected();
    return false;
}

bool DualPowerSupply::isOutputOn(size_t channel) const {
    if (channel == 0)
        return _psu0.isOutputOn();
    if (channel == 1)
        return _psu1.isOutputOn();
    return false;
}

bool DualPowerSupply::validateBeforeWrite(float expected_current) const {
    if (!areBothConnected())
        return true; // Can't validate if not connected

    bool valid0 = _psu0.validateStateBeforeWrite(expected_current);
    bool valid1 = _psu1.validateStateBeforeWrite(expected_current);

    return valid0 && valid1;
}

// =============================================================================
// Channel Imbalance Detection
// =============================================================================

float DualPowerSupply::getCurrentImbalance() const {
    if (!areBothConnected())
        return 0.0f;

    // Only check if we commanded same current to both
    if (fabs(_psu0.getCommandedCurrent() - _psu1.getCommandedCurrent()) > 0.1f)
        return 0.0f;

    return fabs(_psu0.getOutputCurrent() - _psu1.getOutputCurrent());
}

float DualPowerSupply::getPowerImbalance() const {
    if (!areBothConnected())
        return 0.0f;

    // Only check if we commanded same current to both
    if (fabs(_psu0.getCommandedCurrent() - _psu1.getCommandedCurrent()) > 0.1f)
        return 0.0f;

    return fabs(_psu0.getOutputPower() - _psu1.getOutputPower());
}
