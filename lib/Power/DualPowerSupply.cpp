/**
 * @file DualPowerSupply.cpp
 * @brief Implementation of symmetric dual PSU control
 */

#include "DualPowerSupply.h"
#include <cmath>

DualPowerSupply::DualPowerSupply(Logger &logger)
    : _logger(logger), _psu_storage{DPS5015(logger, "DC12", Serial1),
                                    DPS5015(logger, "DC34", Serial2)},
      _psus{&_psu_storage[0], &_psu_storage[1]}, _target_current(0.0f),
      _target_output(false), _shutdown_in_progress(false),
      _shutdown_current(0.0f), _last_shutdown_step_time(0),
    _consecutive_mismatches() {}

void DualPowerSupply::begin() {
    _psus[0]->begin(PIN_DPS5015_1_RX, PIN_DPS5015_1_TX);
    _psus[1]->begin(PIN_DPS5015_2_RX, PIN_DPS5015_2_TX);
}

void DualPowerSupply::update() {
    for (auto *psu : _psus) {
        psu->update();
    }

    // Track latest measured current for override classification
    float sum_current = 0.0f;
    int count = 0;
    for (auto *psu : _psus) {
        if (psu->isConnected()) {
            sum_current += psu->getOutputCurrent();
            count++;
        }
    }
    if (count > 0) {
        _last_measure_ms = millis();
        _last_measure_mA = static_cast<uint16_t>(lround((sum_current / count) * 1000.0f));
    }
}

bool DualPowerSupply::setSymmetricCurrent(float current) {
    // Clamp to valid range
    current = Clamp::current(current);
    _target_current = current;
    _last_setpoint_change_time = millis(); // Record for imbalance suppression
    _last_command_ms = millis();

    bool success = true;
    for (auto *psu : _psus) {
        if (psu->isConnected()) {
            success &= psu->setCurrent(current);
        }
    }
    return success;
}

bool DualPowerSupply::enableOutput() {
    _target_output = true;
    _last_command_ms = millis();

    bool success = true;
    for (auto *psu : _psus) {
        if (psu->isConnected()) {
            success &= psu->setOutput(true);
        }
    }
    return success;
}

bool DualPowerSupply::disableOutput() {
    _target_output = false;
    _last_command_ms = millis();

    bool success = true;
    for (auto *psu : _psus) {
        success &= psu->disableOutput();
    }
    return success;
}

void DualPowerSupply::configure(float voltage, float current, bool outputOn) {
    _target_current = current;
    _target_output = outputOn;
    _last_command_ms = millis();

    for (auto *psu : _psus) {
        psu->configure(voltage, current, outputOn);
    }

    // Reset override counter - we just intentionally changed settings
    // so any temporary mismatch should not trigger override detection
    resetOverrideCounter();
}

void DualPowerSupply::startEmergencyShutdown() {
    if (_shutdown_in_progress)
        return;

    _logger.log("DPS: Emergency shutdown");

    // Immediately set target to 0 to prevent any further increases
    _target_current = 0.0f;
    _target_output = false;

    // Clear any pending override detection (we're intentionally shutting down)
    resetOverrideCounter();

    _shutdown_in_progress = true;
    _shutdown_current =
        fmax(_psus[0]->getOutputCurrent(), _psus[1]->getOutputCurrent());
    // Use the actual current for ramp-down (don't use old _target_current)
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
        for (auto *psu : _psus) {
            psu->disableOutput();
        }
        _target_current = 0;
        _target_output = false;
        _shutdown_in_progress = false;
        _logger.log("DPS: Shutdown complete");
        return false;
    } else {
        // Continue ramp-down with immediate writes
        for (auto *psu : _psus) {
            psu->setCurrentImmediate(_shutdown_current);
        }
        return true;
    }
}

bool DualPowerSupply::hardShutdown() {
    _logger.log("DPS: HARD CUT!");

    _shutdown_in_progress = false;
    _target_current = 0;
    _target_output = false;

    bool success = true;
    for (auto *psu : _psus) {
        success &= psu->disableOutput();
    }

    if (!success) {
        _logger.log("CRIT: PSU shutdown FAIL!");
    }

    return success;
}

bool DualPowerSupply::areBothConnected() const {
    return _psus[0]->isConnected() && _psus[1]->isConnected();
}

bool DualPowerSupply::isEitherConnected() const {
    return _psus[0]->isConnected() || _psus[1]->isConnected();
}

bool DualPowerSupply::isAsymmetricFailure() const {
    return _psus[0]->isConnected() != _psus[1]->isConnected();
}

OverrideInfo DualPowerSupply::checkOverrideDetail() {
    OverrideInfo info;

    if (!areBothConnected())
        return info;

    // Skip during settle or pending writes/busy comms
    bool any_grace = false;
    bool any_pending = false;
    bool any_busy = false;
    for (auto *psu : _psus) {
        any_grace |= psu->isInGracePeriod();
        any_pending |= psu->hasPendingWrites();
        any_busy |= psu->isBusy();
    }
    if (any_grace || any_pending || any_busy)
        return info;

    const unsigned long now = millis();
    const float actualA = getAverageOutputCurrent();
    const float cmdA = _target_current;

    const bool current_mismatch =
        fabs(actualA - cmdA) > Tuning::MANUAL_OVERRIDE_CURRENT_TOLERANCE_A;
    const bool output_mismatch = _target_output != isOutputOn();
    const bool mismatch = current_mismatch || output_mismatch;

    if (!mismatch) {
        _consecutive_mismatches.reset();
        return info;
    }

    // Give time to settle after last command
    if (now - _last_command_ms < Timing::MANUAL_OVERRIDE_GRACE_MS) {
        return info; // treat as pending
    }

    _consecutive_mismatches.inc();
    if (!_consecutive_mismatches.atLeast(OVERRIDE_CONFIRM_COUNT)) {
        return info; // not confirmed yet
    }

    // Human override heuristic: current changed without a command in flight
    const bool measured_recently = (now - _last_measure_ms) < 2000;
    const bool actual_changed =
        measured_recently &&
        (abs((int)(actualA * 1000) - (int)_last_measure_mA) >
         (int)lround(Tuning::MANUAL_OVERRIDE_CURRENT_TOLERANCE_A * 1000.0f));

    if (!any_busy && actual_changed &&
        (now - _last_command_ms) > Timing::MANUAL_OVERRIDE_GRACE_MS) {
        info.cause = OverrideCause::HUMAN_OVERRIDE;
        snprintf(info.reason, sizeof(info.reason),
                 "PSU changed externally: cmd=%.2fA act=%.2fA", cmdA, actualA);
        return info;
    }

    info.cause = OverrideCause::CONTROL_MISMATCH;
    snprintf(info.reason, sizeof(info.reason),
             "Command mismatch persists: cmd=%.2fA act=%.2fA", cmdA, actualA);
    return info;
}

bool DualPowerSupply::isOutputOn() const {
    for (auto *psu : _psus) {
        if (psu->isOutputOn())
            return true;
    }
    return false;
}

bool DualPowerSupply::areBothSettled() const {
    // Both PSUs must be connected and have processed our last commands
    if (!areBothConnected())
        return false;

    for (auto *psu : _psus) {
        if (!psu->isSettled())
            return false;
    }
    return true;
}

float DualPowerSupply::getAverageOutputCurrent() const {
    float sum = 0.0f;
    int count = 0;

    for (auto *psu : _psus) {
        if (psu->isConnected()) {
            sum += psu->getOutputCurrent();
            count++;
        }
    }

    return count > 0 ? (sum / count) : 0.0f;
}

float DualPowerSupply::getTotalPower() const {
    float power = 0.0f;
    for (auto *psu : _psus) {
        if (psu->isConnected()) {
            power += psu->getOutputPower();
        }
    }
    return power;
}

float DualPowerSupply::getOutputCurrent(size_t channel) const {
    return (channel < 2) ? _psus[channel]->getOutputCurrent() : 0.0f;
}

float DualPowerSupply::getOutputVoltage(size_t channel) const {
    return (channel < 2) ? _psus[channel]->getOutputVoltage() : 0.0f;
}

float DualPowerSupply::getOutputPower(size_t channel) const {
    return (channel < 2) ? _psus[channel]->getOutputPower() : 0.0f;
}

bool DualPowerSupply::isConnected(size_t channel) const {
    return (channel < 2) ? _psus[channel]->isConnected() : false;
}

bool DualPowerSupply::isOutputOn(size_t channel) const {
    return (channel < 2) ? _psus[channel]->isOutputOn() : false;
}

float DualPowerSupply::getCurrentImbalance() const {
    if (!areBothConnected())
        return 0.0f;

    // Only check if we commanded same current to both
    if (fabs(_psus[0]->getCommandedCurrent() -
             _psus[1]->getCommandedCurrent()) > 0.1f)
        return 0.0f;

    return fabs(_psus[0]->getOutputCurrent() - _psus[1]->getOutputCurrent());
}

float DualPowerSupply::getPowerImbalance() const {
    if (!areBothConnected())
        return 0.0f;

    // Only check if we commanded same current to both
    if (fabs(_psus[0]->getCommandedCurrent() -
             _psus[1]->getCommandedCurrent()) > 0.1f)
        return 0.0f;

    return fabs(_psus[0]->getOutputPower() - _psus[1]->getOutputPower());
}

void DualPowerSupply::checkAndLogImbalance(float current_threshold,
                                           float power_threshold,
                                           unsigned long interval_ms) {
    if (!areBothConnected())
        return;

    // Skip check if either PSU is in grace period (just received a command)
    for (auto *psu : _psus) {
        if (psu->isInGracePeriod())
            return;
    }

    unsigned long now = millis();

    // Skip if we recently changed the setpoint - PSUs may ramp at different
    // rates
    if (now - _last_setpoint_change_time < IMBALANCE_SETTLE_MS)
        return;

    if (now - _last_imbalance_log_time < interval_ms)
        return;

    float current_diff = getCurrentImbalance();
    float power_diff = getPowerImbalance();

    if (current_diff > current_threshold || power_diff > power_threshold) {
        _logger.logf("IMBAL: dI=%.2fA dP=%.0fW", current_diff, power_diff);
        _last_imbalance_log_time = now;
    }
}

bool DualPowerSupply::hasAnyMismatch() const {
    // If not connected, we can't determine mismatch, assume safe (false)
    // or let safety monitor handle disconnection.
    if (!areBothConnected())
        return false;

    // Delegate to individual PSUs
    // Note: This ignores the grace period! It is a raw check.
    // If we are in a grace period, we shouldn't be attempting an optimization
    // step anyway.
    for (auto *psu : _psus) {
        if (psu->hasAnyMismatch())
            return true;
    }
    return false;
}

float DualPowerSupply::detectHotReset(float min_threshold) {
    // We might be called when only one is connected.
    // We want to know if *any* connected PSU is actually DELIVERING power.

    float current_sum = 0.0f;
    float power_sum = 0.0f;
    int active_count = 0;

    for (auto *psu : _psus) {
        if (!psu->isConnected())
            continue;
        if (!psu->isOutputOn())
            continue;

        float i = psu->getOutputCurrent();
        float p = psu->getOutputPower();

        // Treat as active only if both current and power are meaningful
        if (i > min_threshold && p > 1.0f) {
            current_sum += i;
            power_sum += p;
            active_count++;
        }
    }

    if (active_count == 0)
        return 0.0f;

    float avg_current = current_sum / active_count;

    // Hot reset detected - round down to nearest 0.1A for clean values
    float rounded_current = floorf(avg_current * 10.0f) / 10.0f;
    rounded_current = fmin(rounded_current, Limits::MAX_CURRENT_PER_CHANNEL);

    return rounded_current;
}

void DualPowerSupply::resetSelfTest() {
    _selftest_phase = static_cast<int>(SelfTestPhase::START);
    _selftest_phase_start = 0;
    _selftest_passed[0] = false;
    _selftest_passed[1] = false;
}

SelfTestResult DualPowerSupply::runSelfTest() {
    unsigned long now = millis();
    unsigned long phase_elapsed = now - _selftest_phase_start;
    auto phase = static_cast<SelfTestPhase>(_selftest_phase);

    switch (phase) {
    case SelfTestPhase::START:
        _logger.log("DPS: Self-test start");
        for (auto *psu : _psus) {
            psu->setVoltage(ST_VOLTAGE);
            psu->setCurrent(ST_CURRENT);
            psu->setOutput(false);
        }
        _selftest_phase = static_cast<int>(SelfTestPhase::WAIT_WRITES);
        _selftest_phase_start = now;
        break;

    case SelfTestPhase::WAIT_WRITES:
        // Wait for all pending writes to complete before checking
        for (auto *psu : _psus) {
            if (psu->hasPendingWrites())
                return SelfTestResult::IN_PROGRESS;
        }
        // Writes done - start settle timer
        _selftest_phase = static_cast<int>(SelfTestPhase::CHECK_SETTINGS);
        _selftest_phase_start = now;
        break;

    case SelfTestPhase::CHECK_SETTINGS:
        // Wait for settle time (allows Modbus read to update values)
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (!areBothConnected()) {
            _logger.log("CRIT: Self-test FAIL: offline");
            return SelfTestResult::FAILED;
        }

        for (auto *psu : _psus) {
            if (fabs(psu->getSetVoltage() - ST_VOLTAGE) >
                ST_VOLTAGE_TOLERANCE) {
                _logger.log("CRIT: Self-test FAIL: V mismatch");
                return SelfTestResult::FAILED;
            }
        }

        _selftest_phase = static_cast<int>(SelfTestPhase::ENABLE_PSU0);
        _selftest_phase_start = now;
        break;

    case SelfTestPhase::ENABLE_PSU0:
        _psus[0]->setOutput(true);
        _selftest_phase = static_cast<int>(SelfTestPhase::VERIFY_PSU0);
        _selftest_phase_start = now;
        break;

    case SelfTestPhase::VERIFY_PSU0:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (_psus[0]->isOutputOn()) {
            _selftest_passed[0] = true;
            _psus[0]->setOutput(false);
            _selftest_phase = static_cast<int>(SelfTestPhase::ENABLE_PSU1);
            _selftest_phase_start = now;
        } else if (phase_elapsed > ST_TIMEOUT_MS) {
            _logger.log("CRIT: Self-test FAIL: DPS0 output");
            return SelfTestResult::FAILED;
        }
        break;

    case SelfTestPhase::ENABLE_PSU1:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;
        _psus[1]->setOutput(true);
        _selftest_phase = static_cast<int>(SelfTestPhase::VERIFY_PSU1);
        _selftest_phase_start = now;
        break;

    case SelfTestPhase::VERIFY_PSU1:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (_psus[1]->isOutputOn()) {
            _selftest_passed[1] = true;
            _psus[1]->setOutput(false);
            _selftest_phase = static_cast<int>(SelfTestPhase::COMPLETE);
            _selftest_phase_start = now;
        } else if (phase_elapsed > ST_TIMEOUT_MS) {
            _logger.log("CRIT: Self-test FAIL: DPS1 output");
            return SelfTestResult::FAILED;
        }
        break;

    case SelfTestPhase::COMPLETE:
        if (phase_elapsed < ST_SETTLE_MS)
            return SelfTestResult::IN_PROGRESS;

        if (_selftest_passed[0] && _selftest_passed[1]) {
            _logger.log("DPS: Self-test PASS");
            return SelfTestResult::PASSED;
        } else {
            _logger.log("CRIT: Self-test incomplete");
            return SelfTestResult::FAILED;
        }

    default:
        // Shouldn't happen, but treat as timeout
        _logger.log("DPS: Self-test invalid state");
        return SelfTestResult::FAILED;
    }

    return SelfTestResult::IN_PROGRESS;
}
