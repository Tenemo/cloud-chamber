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
                                     TemperatureSensors &sensors,
                                     DualPowerSupply &dps)
    : _logger(logger), _sensors(sensors), _dps(dps), _metrics(logger),
      _optimizer(logger), _safety(logger, sensors.getColdPlateSensor(),
                                  sensors.getHotPlateSensor(), _dps),
      _state(ThermalState::INITIALIZING),
      _state_before_fault(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_sample_time(0), _last_adjustment_time(0),
      _steady_state_start_time(0), _ramp_start_time(0), _sensor_fault_time(0) {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_sample_time = millis();

    _dps.begin();
    _metrics.begin();
    _safety.setMetrics(&_metrics);

    registerDisplayLines();
    _logger.log("ThermalCtrl: init");
}

void ThermalController::registerDisplayLines() {
    using namespace ThermalDisplay;
    _logger.registerTextLine(LINE_STATE, "State:", "INIT");
    _logger.registerLine(LINE_RATE, "dT/dt:", "K/m", 0.0f);
    _logger.registerLine(LINE_CURRENT, "I Set:", "A", 0.0f);
}

// =============================================================================
// State String Conversion
// =============================================================================

const char *ThermalController::getStateString() const {
    switch (_state) {
    case ThermalState::INITIALIZING:
        return "INIT";
    case ThermalState::SELF_TEST:
        return "TEST";
    case ThermalState::STARTUP:
        return "START";
    case ThermalState::RAMP_UP:
        return "RAMP";
    case ThermalState::STEADY_STATE:
        return "STEADY";
    case ThermalState::MANUAL_OVERRIDE:
        return "MANUAL";
    case ThermalState::THERMAL_FAULT:
        return "FAULT";
    case ThermalState::SENSOR_FAULT:
        return "SENS_F";
    case ThermalState::DPS_DISCONNECTED:
        return "DSCNCT";
    default:
        return "???";
    }
}

// =============================================================================
// Public Accessors
// =============================================================================

unsigned long ThermalController::getSteadyStateUptime() const {
    if (_state == ThermalState::STEADY_STATE) {
        return millis() - _steady_state_start_time;
    }
    return 0;
}

float ThermalController::getCoolingRate() const {
    return _metrics.getColdPlateRate();
}

// =============================================================================
// Main Update Loop
// =============================================================================

void ThermalController::update() {
    unsigned long now = millis();

    // Update PSU communication (Modbus polling)
    _dps.update();

    // Record history sample at regular intervals
    if (now - _last_sample_time >= HISTORY_SAMPLE_INTERVAL_MS) {
        recordSample();
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

// =============================================================================
// Safety Checks (run once per update, not in each handler)
// =============================================================================

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

// =============================================================================
// Control Permission Check
// =============================================================================

bool ThermalController::isOperationalState() const {
    // States where safety checks should NOT run:
    // - INITIALIZING: Hardware not ready yet, sensors may not be valid
    // - SELF_TEST: Running DPS verification, not operational
    // - THERMAL_FAULT: Already in fault state, shutdown in progress
    // - DPS_DISCONNECTED: No PSU communication, can't check or control
    return _state != ThermalState::INITIALIZING &&
           _state != ThermalState::SELF_TEST &&
           _state != ThermalState::THERMAL_FAULT &&
           _state != ThermalState::DPS_DISCONNECTED;
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

// =============================================================================
// State Transitions
// =============================================================================

void ThermalController::transitionTo(ThermalState newState) {
    if (newState == _state)
        return;

    _state_before_fault = _state;
    _state = newState;
    _state_entry_time = millis();

    _logger.logf(false, "TC: -> %s", getStateString());

    // State entry actions
    switch (newState) {
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

// =============================================================================
// State Handlers (simplified - no repeated safety checks)
// =============================================================================

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

    // Configure PSUs at start of window
    if (elapsed < Timing::STARTUP_CONFIG_WINDOW_MS) {
        if (elapsed < Timing::STARTUP_CONFIG_SEND_MS) {
            _dps.configure(TEC_VOLTAGE_SETPOINT, STARTUP_CURRENT, true);
            _logger.logf("TC: Start %.1fA", STARTUP_CURRENT);
        }
        return;
    }

    // Hold for startup duration
    if (elapsed >= STARTUP_HOLD_DURATION_MS) {
        transitionTo(ThermalState::RAMP_UP);
    }
}

void ThermalController::handleRampUp() {
    ThermalSnapshot snapshot = buildSnapshot();
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

    // Log if optimizer has a message
    if (decision.log_message) {
        _logger.log(decision.log_message, true);
    }

    // Execute current change if requested
    if (decision.change_current) {
        _dps.setSymmetricCurrent(decision.new_current);
        _dps.resetOverrideCounter();
        _last_adjustment_time = snapshot.now;
    }

    // Handle revert on degraded evaluation
    if (decision.evaluation_complete && decision.should_transition) {
        transitionTo(ThermalState::STEADY_STATE);
    }
}

void ThermalController::handleSteadyState() {
    ThermalSnapshot snapshot = buildSnapshot();

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

    // Log if optimizer has a message
    if (decision.log_message) {
        _logger.log(decision.log_message, true);
    }

    // Execute current change if requested
    if (decision.change_current) {
        _dps.setSymmetricCurrent(decision.new_current);
        _dps.resetOverrideCounter();
        _last_adjustment_time = snapshot.now;
    }
}

void ThermalController::handleManualOverride() {
    // ==========================================================================
    // MANUAL OVERRIDE POLICY: Monitor-only, no automatic control
    // ==========================================================================
    // This state is entered when a human physically adjusts the DPS dial,
    // detected via consecutive mismatches between commanded and actual values.
    //
    // Behavior:
    // - NO commands are sent to the PSUs - we respect human intervention
    // - Safety checks STILL RUN via runSafetyChecks() at top of update()
    // - Thermal faults will still trigger emergency shutdown
    // - This state is PERMANENT until power cycle (no auto-recovery)
    //
    // Rationale: If someone is manually adjusting the system, they have a
    // reason. Automatically fighting their changes could be dangerous.
    // We stay hands-off but keep watching for critical safety issues.
    // ==========================================================================
}

void ThermalController::handleThermalFault() {
    // Intentionally empty - emergency shutdown is handled by DualPowerSupply.
    // This state is latched (requires power cycle to exit) to ensure
    // the system stays off after a thermal event until human inspection.
}

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

// =============================================================================
// Optimizer Support
// =============================================================================

ThermalSnapshot ThermalController::buildSnapshot() const {
    return ThermalSnapshot{
        _sensors.getColdPlateTemperature(),
        _sensors.getHotPlateTemperature(),
        _metrics.getColdPlateRate(),
        _metrics.getHotPlateRate(),
        _dps.getTargetCurrent(),
        _safety.isHotSideWarning(),
        _safety.isHotSideAlarm(),
        _metrics.isHotSideStable(HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN),
        millis()};
}

// =============================================================================
// History and Analysis
// =============================================================================

void ThermalController::recordSample() {
    ThermalSample sample;
    sample.cold_plate_temp = _sensors.getColdPlateTemperature();
    sample.hot_plate_temp = _sensors.getHotPlateTemperature();
    sample.set_current = _dps.getTargetCurrent();
    sample.voltage[0] = _dps.getOutputVoltage(0);
    sample.voltage[1] = _dps.getOutputVoltage(1);
    sample.current[0] = _dps.getOutputCurrent(0);
    sample.current[1] = _dps.getOutputCurrent(1);
    sample.power[0] = _dps.getOutputPower(0);
    sample.power[1] = _dps.getOutputPower(1);
    sample.timestamp = millis();

    _metrics.recordSample(sample);

    // Check for channel imbalance (rate-limited logging)
    _dps.checkAndLogImbalance(CHANNEL_CURRENT_IMBALANCE_A,
                              CHANNEL_POWER_IMBALANCE_W,
                              Timing::IMBALANCE_LOG_INTERVAL_MS);
}

// =============================================================================
// Display
// =============================================================================

void ThermalController::updateDisplay() {
    using namespace ThermalDisplay;
    _logger.updateLineText(LINE_STATE, getStateString());
    _logger.updateLine(LINE_RATE, _metrics.getColdPlateRate());
    _logger.updateLine(LINE_CURRENT, _dps.getTargetCurrent());
}
