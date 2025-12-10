/**
 * @file DualPowerSupply.cpp
 * @brief Implementation of symmetric dual PSU control
 */

#include "DualPowerSupply.h"
#include "CrashLog.h"
#include "ThermalConstants.h"
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
    if (now - _last_shutdown_step_time < Timing::SHUTDOWN_STEP_MS)
        return true;

    _last_shutdown_step_time = now;

    float step = Timing::EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC *
                 (Timing::SHUTDOWN_STEP_MS / 1000.0f);
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

    // Check for any mismatch (current, voltage, or output state)
    bool current_mismatch =
        _psu0.hasCurrentMismatch() || _psu1.hasCurrentMismatch();
    bool voltage_mismatch =
        _psu0.hasVoltageMismatch() || _psu1.hasVoltageMismatch();
    bool output_mismatch =
        _psu0.hasOutputMismatch() || _psu1.hasOutputMismatch();

    if (current_mismatch || voltage_mismatch || output_mismatch) {
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

bool DualPowerSupply::areBothSettled() const {
    // Both PSUs must be connected and have processed our last commands
    if (!areBothConnected())
        return false;

    return _psu0.isSettled() && _psu1.isSettled();
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

void DualPowerSupply::checkAndLogImbalance(float current_threshold,
                                           float power_threshold,
                                           unsigned long interval_ms) {
    if (!areBothConnected())
        return;

    unsigned long now = millis();
    if (now - _last_imbalance_log_time < interval_ms)
        return;

    float current_diff = getCurrentImbalance();
    float power_diff = getPowerImbalance();

    if (current_diff > current_threshold || power_diff > power_threshold) {
        _logger.logf("IMBAL: dI=%.1fA dP=%.0fW", current_diff, power_diff);
        _last_imbalance_log_time = now;
    }
}

// =============================================================================
// Hot Reset Detection
// =============================================================================

float DualPowerSupply::detectHotReset(float min_threshold) {
    if (!isEitherConnected() || !isOutputOn())
        return 0.0f;

    float actual_current = getAverageOutputCurrent();

    // Wait for Modbus read to complete (current not yet available)
    if (actual_current < Tolerance::MIN_CURRENT_THRESHOLD)
        return 0.0f;

    // Below threshold - not considered a hot reset
    if (actual_current <= min_threshold)
        return 0.0f;

    // Hot reset detected - return the current to adopt
    _logger.log("DPS: Hot reset detected");
    return fmin(actual_current, MAX_CURRENT_PER_CHANNEL);
}

// =============================================================================
// Self-Test
// =============================================================================

// Self-test phase constants (local to this file)
namespace {
constexpr int ST_PHASE_START = 0;
constexpr int ST_PHASE_CHECK_SETTINGS = 1;
constexpr int ST_PHASE_ENABLE_PSU0 = 2;
constexpr int ST_PHASE_VERIFY_PSU0 = 3;
constexpr int ST_PHASE_ENABLE_PSU1 = 4;
constexpr int ST_PHASE_VERIFY_PSU1 = 5;
constexpr int ST_PHASE_COMPLETE = 6;

constexpr unsigned long ST_SETTLE_MS = 500;
constexpr unsigned long ST_TIMEOUT_MS = 3000;
constexpr float ST_VOLTAGE = 5.0f;
constexpr float ST_CURRENT = 0.5f;
constexpr float ST_VOLTAGE_TOLERANCE = 0.5f;
} // namespace

void DualPowerSupply::resetSelfTest() {
    _selftest_phase = ST_PHASE_START;
    _selftest_phase_start = 0;
    _selftest_passed[0] = false;
    _selftest_passed[1] = false;
}

SelfTestResult DualPowerSupply::runSelfTest() {
    unsigned long now = millis();
    unsigned long phase_elapsed = now - _selftest_phase_start;

    switch (_selftest_phase) {
    case ST_PHASE_START:
        _logger.log("DPS: Self-test start");
        _psu0.setVoltage(ST_VOLTAGE);
        _psu0.setCurrent(ST_CURRENT);
        _psu0.setOutput(false);
        _psu1.setVoltage(ST_VOLTAGE);
        _psu1.setCurrent(ST_CURRENT);
        _psu1.setOutput(false);
        _selftest_phase = ST_PHASE_CHECK_SETTINGS;
        _selftest_phase_start = now;
        break;

    case ST_PHASE_CHECK_SETTINGS:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (!areBothConnected()) {
            _logger.log("DPS: Self-test FAIL: offline");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS offline");
            return SelfTestResult::FAILED;
        }

        if (fabs(_psu0.getSetVoltage() - ST_VOLTAGE) > ST_VOLTAGE_TOLERANCE ||
            fabs(_psu1.getSetVoltage() - ST_VOLTAGE) > ST_VOLTAGE_TOLERANCE) {
            _logger.log("DPS: Self-test FAIL: V mismatch");
            CrashLog::logCritical("SELFTEST_FAIL", "Voltage mismatch");
            return SelfTestResult::FAILED;
        }

        _selftest_phase = ST_PHASE_ENABLE_PSU0;
        _selftest_phase_start = now;
        break;

    case ST_PHASE_ENABLE_PSU0:
        _psu0.setOutput(true);
        _selftest_phase = ST_PHASE_VERIFY_PSU0;
        _selftest_phase_start = now;
        break;

    case ST_PHASE_VERIFY_PSU0:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (_psu0.isOutputOn()) {
            _selftest_passed[0] = true;
            _psu0.setOutput(false);
            _selftest_phase = ST_PHASE_ENABLE_PSU1;
            _selftest_phase_start = now;
        } else if (phase_elapsed > ST_TIMEOUT_MS) {
            _logger.log("DPS: Self-test FAIL: DPS0 output");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS0 output");
            return SelfTestResult::FAILED;
        }
        break;

    case ST_PHASE_ENABLE_PSU1:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;
        _psu1.setOutput(true);
        _selftest_phase = ST_PHASE_VERIFY_PSU1;
        _selftest_phase_start = now;
        break;

    case ST_PHASE_VERIFY_PSU1:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (_psu1.isOutputOn()) {
            _selftest_passed[1] = true;
            _psu1.setOutput(false);
            _selftest_phase = ST_PHASE_COMPLETE;
            _selftest_phase_start = now;
        } else if (phase_elapsed > ST_TIMEOUT_MS) {
            _logger.log("DPS: Self-test FAIL: DPS1 output");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS1 output");
            return SelfTestResult::FAILED;
        }
        break;

    case ST_PHASE_COMPLETE:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (_selftest_passed[0] && _selftest_passed[1]) {
            _logger.log("DPS: Self-test PASS");
            return SelfTestResult::PASSED;
        } else {
            _logger.log("DPS: Self-test incomplete");
            CrashLog::logCritical("SELFTEST_FAIL", "Incomplete");
            return SelfTestResult::FAILED;
        }

    default:
        // Shouldn't happen, but treat as timeout
        _logger.log("DPS: Self-test invalid state");
        return SelfTestResult::FAILED;
    }

    return SelfTestResult::IN_PROGRESS;
}
