/**
 * @file ThermalController.cpp
 * @brief Implementation of the thermal control system
 *
 * Refactored for clarity:
 * - Safety checks run once at top of update(), not in each handler
 * - All PSU control goes through DualPowerSupply wrapper
 * - State handlers contain only state-specific logic
 */

#include "ThermalController.h"
#include <cmath>

// =============================================================================
// Constructor and Initialization
// =============================================================================

ThermalController::ThermalController(Logger &logger,
                                     TemperatureSensors &sensors)
    : _logger(logger), _sensors(sensors), _dps(logger), _metrics(logger),
      _optimizer(logger), _safety(logger, sensors.getColdPlateSensor(),
                                  sensors.getHotPlateSensor(), _dps),
      _state(ThermalState::INITIALIZING),
      _state_before_fault(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_sample_time(0), _last_adjustment_time(0),
      _steady_state_start_time(0), _ramp_start_time(0), _sensor_fault_time(0),
      _startup_configured(false) {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_sample_time = millis();

    _dps.begin();
    _metrics.begin();
    _safety.setMetrics(&_metrics);

    registerDisplayLines();
    _logger.log("Controller initialized.");
}

void ThermalController::registerDisplayLines() {
    _metrics.registerDisplayLines();
}

unsigned long ThermalController::getSteadyStateUptime() const {
    if (_state == ThermalState::STEADY_STATE) {
        return millis() - _steady_state_start_time;
    }
    return 0;
}

float ThermalController::getCoolingRate() const {
    return _metrics.getColdPlateRate();
}

void ThermalController::update() {
    unsigned long now = millis();

    // Update PSU communication (Modbus polling)
    _dps.update();

    if (now - _last_sample_time >= HISTORY_SAMPLE_INTERVAL_MS) {
        _metrics.recordSample(_sensors, _dps);
        _last_sample_time = now;
    }

    // Update delegate modules
    _metrics.update();

    // Continue emergency shutdown if in progress
    if (_dps.isShutdownInProgress()) {
        _dps.updateEmergencyShutdown();
    }

    // Run safety checks ONCE at top of loop (not in each handler)
    // Skip for non-operational states (hardware not ready or already faulted)
    if (isOperationalState() && !runSafetyChecks()) {
        updateDisplay();
        return; // Safety check triggered state transition
    }

    // Run state-specific handler
    switch (_state) {
    case ThermalState::INITIALIZING:
        handleInitializing();
        break;
    case ThermalState::SELF_TEST:
        handleSelfTest();
        break;
    case ThermalState::STARTUP:
        handleStartup();
        break;
    case ThermalState::RAMP_UP:
        handleRampUp();
        break;
    case ThermalState::STEADY_STATE:
        handleSteadyState();
        break;
    case ThermalState::MANUAL_OVERRIDE:
        handleManualOverride();
        break;
    case ThermalState::THERMAL_FAULT:
        handleThermalFault();
        break;
    case ThermalState::SENSOR_FAULT:
        handleSensorFault();
        break;
    case ThermalState::DPS_DISCONNECTED:
        handleDpsDisconnected();
        break;
    }

    updateDisplay();
}

bool ThermalController::runSafetyChecks() {
    // All safety checks including grace period logic are now in SafetyMonitor
    // Use actual average current (not target) for physics-based plausibility
    // check
    SafetyResult result = _safety.checkAll(_state, _ramp_start_time,
                                           _dps.getAverageOutputCurrent());

    switch (result.status) {
    case SafetyStatus::SENSOR_FAULT:
        transitionTo(ThermalState::SENSOR_FAULT);
        return false;

    case SafetyStatus::THERMAL_FAULT:
        enterThermalFault(result.reason);
        return false;

    case SafetyStatus::DPS_DISCONNECTED:
        transitionTo(ThermalState::DPS_DISCONNECTED);
        return false;

    case SafetyStatus::MANUAL_OVERRIDE:
        _logger.log("TC: Manual override detected");
        transitionTo(ThermalState::MANUAL_OVERRIDE);
        return false;

    case SafetyStatus::OK:
    case SafetyStatus::WARNING:
        break;
    }

    return true;
}

bool ThermalController::isOperationalState() const {
    // States where safety checks should NOT run:
    // - INITIALIZING: Hardware not ready yet, sensors may not be valid
    // - SELF_TEST: Running DPS verification, not operational
    // - THERMAL_FAULT: Already in fault state, shutdown in progress
    // - DPS_DISCONNECTED: No PSU communication, can't check or control
    // - MANUAL_OVERRIDE: Already detected, don't keep re-checking/logging
    return _state != ThermalState::INITIALIZING &&
           _state != ThermalState::SELF_TEST &&
           _state != ThermalState::THERMAL_FAULT &&
           _state != ThermalState::DPS_DISCONNECTED &&
           _state != ThermalState::MANUAL_OVERRIDE;
}

bool ThermalController::canControlPower() const {
    // Centralized check for "am I allowed to send PSU commands?"
    // This policy is checked before any current/voltage adjustment.

    // Never control in fault or override states
    if (_state == ThermalState::MANUAL_OVERRIDE ||
        _state == ThermalState::THERMAL_FAULT ||
        _state == ThermalState::DPS_DISCONNECTED) {
        return false;
    }

    // Don't fight the emergency shutdown ramp
    if (_dps.isShutdownInProgress()) {
        return false;
    }

    return true;
}

void ThermalController::transitionTo(ThermalState newState) {
    if (newState == _state)
        return;

    _state_before_fault = _state;
    _state = newState;
    _state_entry_time = millis();

    _logger.logf(false, "TC: -> %s", stateToString(_state));

    // State entry actions
    switch (newState) {
    case ThermalState::STARTUP:
        // Configure PSUs immediately on entering STARTUP
        _startup_configured = false;
        _dps.configure(TEC_VOLTAGE_SETPOINT, STARTUP_CURRENT, true);
        _startup_configured = true;
        _logger.logf(false, "TC: Config %.0fV %.1fA", TEC_VOLTAGE_SETPOINT,
                     STARTUP_CURRENT);
        break;
    case ThermalState::STEADY_STATE:
        _steady_state_start_time = millis();
        break;
    case ThermalState::RAMP_UP:
        _ramp_start_time = millis();
        break;
    case ThermalState::SELF_TEST:
        _dps.resetSelfTest();
        break;
    case ThermalState::SENSOR_FAULT:
        _sensor_fault_time = millis();
        break;
    case ThermalState::MANUAL_OVERRIDE:
        // Human has taken control - clear optimizer state to avoid stale data
        // affecting any future automatic control (after power cycle)
        _optimizer.reset();
        break;
    default:
        break;
    }
}

void ThermalController::enterThermalFault(const char *reason) {
    _logger.logf(false, "TC FAULT: %s", reason);
    CrashLog::logCritical("THERMAL_FAULT", reason);

    float hot_temp = _sensors.getHotPlateTemperature();

    if (hot_temp >= HOT_SIDE_FAULT_C) {
        _dps.hardShutdown();
    } else {
        _dps.startEmergencyShutdown();
    }

    _state = ThermalState::THERMAL_FAULT;
    _state_entry_time = millis();
}

void ThermalController::handleInitializing() {
    unsigned long elapsed = millis() - _state_entry_time;

    bool cold_plate_ok = !_sensors.isColdPlateError();
    bool hot_plate_ok = _sensors.isHotPlateConnected();
    bool both_psu_ok = _dps.areBothConnected();

    // Check for hot reset (DPS already running)
    float adopted = _dps.detectHotReset(HOT_RESET_CURRENT_THRESHOLD_A);
    if (adopted > 0.0f) {
        CrashLog::logCritical("HOT_RESET", "DPS was running on boot");
        _dps.configure(TEC_VOLTAGE_SETPOINT, adopted, true);

        // Seed the optimizer with current state as starting best point
        float current_temp = _sensors.getColdPlateTemperature();
        _optimizer.seedWithCurrent(adopted, current_temp);

        if (adopted >= HOT_RESET_NEAR_MAX_A) {
            transitionTo(ThermalState::STEADY_STATE);
        } else {
            transitionTo(ThermalState::RAMP_UP);
        }
        return;
    }

    // All hardware ready?
    if (cold_plate_ok && hot_plate_ok && both_psu_ok) {
        transitionTo(ThermalState::SELF_TEST);
        return;
    }

    // Timeout
    if (elapsed > INIT_TIMEOUT_MS) {
        if (!cold_plate_ok || !hot_plate_ok) {
            CrashLog::logCritical("INIT_FAIL", "Sensor timeout");
            transitionTo(ThermalState::SENSOR_FAULT);
        } else {
            CrashLog::logCritical("INIT_FAIL", "DPS timeout");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
    }
}

void ThermalController::handleSelfTest() {
    SelfTestResult result = _dps.runSelfTest();

    switch (result) {
    case SelfTestResult::PASSED:
        transitionTo(ThermalState::STARTUP);
        break;

    case SelfTestResult::FAILED:
        transitionTo(ThermalState::DPS_DISCONNECTED);
        break;

    case SelfTestResult::IN_PROGRESS:
        // Still running, check for overall timeout
        if (millis() - _state_entry_time > Timing::SELFTEST_TIMEOUT_MS * 3) {
            _logger.log("TC: Self-test timeout");
            CrashLog::logCritical("SELFTEST_FAIL", "Timeout");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;
    }
}

void ThermalController::handleStartup() {
    unsigned long elapsed = millis() - _state_entry_time;

    // Configuration is done in transitionTo() when entering STARTUP
    // Just wait for the startup hold duration
    if (elapsed >= STARTUP_HOLD_DURATION_MS) {
        transitionTo(ThermalState::RAMP_UP);
    }
}

void ThermalController::applyOptimizationDecision(
    const ThermalOptimizationDecision &decision, unsigned long snapshot_now) {
    // Log if optimizer has a message
    if (decision.log_message) {
        _logger.log(decision.log_message, true);
    }

    // Execute current change if requested
    if (decision.change_current) {
        _dps.setSymmetricCurrent(decision.new_current);
        _dps.resetOverrideCounter();
        _last_adjustment_time = snapshot_now;
    }
}

void ThermalController::handleRampUp() {
    ThermalSnapshot snapshot = _metrics.buildSnapshot(_sensors, _dps, _safety);
    float current = snapshot.current_setpoint;

    // Check termination conditions via optimizer
    bool has_enough_history =
        _metrics.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2);

    if (_optimizer.shouldExitRamp(snapshot, has_enough_history)) {
        _optimizer.updateBest(current, snapshot.cold_temp);

        if (_safety.isHotSideAlarm() ||
            (fabs(snapshot.cooling_rate) < COOLING_STALL_THRESHOLD_C &&
             has_enough_history)) {
            _optimizer.setProbeDirection(-1); // Start by trying to decrease
            _logger.logf(true, "TC: Limit/stall at %.1fA, fine-tuning",
                         current);
        }

        transitionTo(ThermalState::STEADY_STATE);
        return;
    }

    // Let optimizer decide what to do
    bool psus_ready = canControlPower() && _dps.areBothSettled();
    ThermalOptimizationDecision decision = _optimizer.update(
        snapshot, ThermalControlPhase::RAMP_UP, _last_adjustment_time,
        RAMP_ADJUSTMENT_INTERVAL_MS, psus_ready);

    applyOptimizationDecision(decision, snapshot.now);

    // Handle revert on degraded evaluation
    if (decision.evaluation_complete && decision.should_transition) {
        transitionTo(ThermalState::STEADY_STATE);
    }
}

void ThermalController::handleSteadyState() {
    ThermalSnapshot snapshot = _metrics.buildSnapshot(_sensors, _dps, _safety);

    // Record new minimum for NVS persistence
    _metrics.recordNewMinimum(snapshot.cold_temp, snapshot.current_setpoint);

    // Reset converged state at recheck interval
    if (_optimizer.isConverged()) {
        if (_metrics.isTimeForAdjustment(_last_adjustment_time,
                                         STEADY_STATE_RECHECK_INTERVAL_MS)) {
            _optimizer.clearConverged();
            _logger.log("TC: Recheck, probing again", true);
        }
    }

    // Let optimizer decide what to do
    bool psus_ready = canControlPower() && _dps.areBothSettled();
    ThermalOptimizationDecision decision = _optimizer.update(
        snapshot, ThermalControlPhase::STEADY_STATE, _last_adjustment_time,
        STEADY_STATE_RECHECK_INTERVAL_MS, psus_ready);

    applyOptimizationDecision(decision, snapshot.now);
}

void ThermalController::handleManualOverride() {}

void ThermalController::handleThermalFault() {}

void ThermalController::handleSensorFault() {
    unsigned long elapsed = millis() - _sensor_fault_time;

    // Reduce to safe current
    if (_dps.getTargetCurrent() > DEGRADED_MODE_CURRENT / 2) {
        _dps.setSymmetricCurrent(DEGRADED_MODE_CURRENT / 2);
    }

    // Check recovery
    bool cold_ok = !_sensors.isColdPlateError();
    bool hot_ok = _sensors.isHotPlateConnected();

    if (cold_ok && hot_ok) {
        _logger.log("TC: Sensors recovered");
        transitionTo(_state_before_fault);
        return;
    }

    if (elapsed > SENSOR_RECOVERY_TIMEOUT_MS) {
        CrashLog::logCritical("SENSOR_FAULT", "Recovery timeout");
        enterThermalFault("Sensor timeout");
    }
}

void ThermalController::handleDpsDisconnected() {
    if (_dps.areBothConnected()) {
        _logger.log("TC: DPS reconnected");
        transitionTo(ThermalState::STARTUP);
        return;
    }

    // Symmetric shutdown: if one fails, shut down both
    if (_dps.isAsymmetricFailure()) {
        _logger.log("TC: Symmetric shutdown");
        CrashLog::logCritical("DPS_FAIL", "Symmetric shutdown");
        _dps.disableOutput();
    }
}

void ThermalController::updateDisplay() {
    _metrics.updateDisplay(stateToString(_state), _dps.getTargetCurrent());
}
