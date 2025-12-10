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

ThermalController::ThermalController(Logger &logger, PT100Sensor &coldPlate,
                                     DS18B20Sensor &hotPlate, DPS5015 &psu0,
                                     DPS5015 &psu1)
    : _logger(logger), _cold_plate(coldPlate), _hot_plate(hotPlate),
      _dps(logger, psu0, psu1), _metrics(logger),
      _safety(logger, coldPlate, hotPlate, _dps),
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
    // Skip for certain states:
    // - INITIALIZING: Hardware not ready yet, sensors may not be valid
    // - SELF_TEST: Running DPS verification, not operational
    // - THERMAL_FAULT: Already in fault state, shutdown in progress
    // - DPS_DISCONNECTED: No PSU communication, can't check or control
    bool skip_safety = (_state == ThermalState::INITIALIZING ||
                        _state == ThermalState::SELF_TEST ||
                        _state == ThermalState::THERMAL_FAULT ||
                        _state == ThermalState::DPS_DISCONNECTED);

    if (!skip_safety && !runSafetyChecks()) {
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
    SafetyResult result =
        _safety.checkAll(_state, _ramp_start_time, _dps.getTargetCurrent());

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
        _metrics.optimizer().reset();
        break;
    default:
        break;
    }
}

void ThermalController::enterThermalFault(const char *reason) {
    _logger.logf(false, "TC FAULT: %s", reason);
    CrashLog::logCritical("THERMAL_FAULT", reason);

    float hot_temp = _hot_plate.getTemperature();

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

    bool cold_plate_ok = !_cold_plate.isInError();
    bool hot_plate_ok = _hot_plate.isConnected();
    bool both_psu_ok = _dps.areBothConnected();

    // Check for hot reset (DPS already running)
    float adopted = _dps.detectHotReset(HOT_RESET_CURRENT_THRESHOLD_A);
    if (adopted > 0.0f) {
        CrashLog::logCritical("HOT_RESET", "DPS was running on boot");
        _dps.configure(TEC_VOLTAGE_SETPOINT, adopted, true);

        // Seed the optimizer with current state as starting best point
        float current_temp = _cold_plate.getTemperature();
        _metrics.optimizer().updateBest(adopted, current_temp);

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
    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();
    float current = _dps.getTargetCurrent();
    OptimizerState &opt = _metrics.optimizer();

    // Evaluate pending current change (with transition on bounce)
    if (tryCompleteEvaluation(true)) {
        if (_state != ThermalState::RAMP_UP)
            return; // Transitioned to STEADY_STATE
    }

    // Check termination conditions
    if (shouldExitRamp(current)) {
        opt.optimal_current = current;
        opt.temp_at_optimal = cold_temp;

        if (_safety.isHotSideAlarm() ||
            (fabs(_metrics.getColdPlateRate()) < COOLING_STALL_THRESHOLD_C &&
             _metrics.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2))) {
            opt.probe_direction = -1; // Start by trying to decrease
            _logger.logf(true, "TC: Limit/stall at %.1fA, fine-tuning",
                         current);
        }

        transitionTo(ThermalState::STEADY_STATE);
        return;
    }

    // Time for adjustment?
    if (!_metrics.isTimeForAdjustment(_last_adjustment_time,
                                      RAMP_ADJUSTMENT_INTERVAL_MS))
        return;

    // Before changing current, verify PSUs are settled and we're allowed
    if (!canControlPower())
        return;

    if (!_dps.areBothSettled()) {
        // PSUs haven't processed last command yet - wait
        return;
    }

    // Choose adaptive step size based on cooling rate (delegated to
    // ThermalMetrics)
    float step = _metrics.recommendStepSize(_safety.isHotSideWarning());
    opt.current_step = step;

    // Record baseline before step (for local revert if needed)
    opt.baseline_current = current;
    opt.baseline_temp = cold_temp;
    opt.baseline_rate = _metrics.getColdPlateRate();

    // Increase current (ramp is always upward)
    float new_current = fmin(current + step, MAX_CURRENT_PER_CHANNEL);
    _dps.setSymmetricCurrent(new_current);
    _dps.resetOverrideCounter(); // We just caused a mismatch intentionally
    _last_adjustment_time = now;
    opt.awaiting_evaluation = true;
    opt.eval_start_time = now;

    _logger.logf(true, "TC: Ramp %.1fA (+%.2fA)", new_current, step);
}

bool ThermalController::shouldExitRamp(float current) const {
    // At maximum current
    if (current >= MAX_CURRENT_PER_CHANNEL)
        return true;

    // Hot side limiting
    if (_safety.isHotSideAlarm())
        return true;

    // Cooling stalled (only if we have enough history)
    if (_metrics.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2)) {
        if (fabs(_metrics.getColdPlateRate()) < COOLING_STALL_THRESHOLD_C)
            return true;
    }

    return false;
}

void ThermalController::handleSteadyState() {
    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();
    float current = _dps.getTargetCurrent();
    OptimizerState &opt = _metrics.optimizer();

    // Record new minimum for NVS persistence
    _metrics.recordNewMinimum(cold_temp, current);

    // Evaluate pending change
    if (tryCompleteEvaluation(false)) {
        return; // Evaluation processed, wait for next cycle
    }

    // Periodic optimization probe (bidirectional to find true optimum)
    bool can_probe = _metrics.isTimeForAdjustment(
        _last_adjustment_time, STEADY_STATE_RECHECK_INTERVAL_MS);

    if (can_probe) {
        // Before changing current, verify PSUs are settled and we're allowed
        if (!canControlPower())
            return;

        if (!_dps.areBothSettled()) {
            // PSUs haven't processed last command yet - wait
            return;
        }

        // Reset converged state at each recheck interval - conditions may have
        // changed
        if (opt.converged) {
            opt.converged = false;
            opt.consecutive_bounces = 0;
            _logger.log("TC: Recheck, probing again", true);
        }

        // Choose adaptive step size (delegated to ThermalMetrics)
        float step = _metrics.recommendStepSize(_safety.isHotSideWarning());
        opt.current_step = step;

        // Record baseline before step
        opt.baseline_current = current;
        opt.baseline_temp = cold_temp;
        opt.baseline_rate = _metrics.getColdPlateRate();

        // Calculate target current based on probe direction
        float new_current;
        bool can_step = false;

        if (opt.probe_direction > 0) {
            // Try increasing (if we have room and thermal headroom)
            if (current < MAX_CURRENT_PER_CHANNEL &&
                hot_temp < HOT_SIDE_WARNING_C - HOT_SIDE_PROBE_HEADROOM_C) {
                new_current = fmin(current + step, MAX_CURRENT_PER_CHANNEL);
                can_step = true;
            }
        } else {
            // Try decreasing (if we have room)
            if (current > STARTUP_CURRENT + step) {
                new_current = current - step;
                can_step = true;
            }
        }

        if (can_step) {
            _dps.setSymmetricCurrent(new_current);
            _dps.resetOverrideCounter(); // We just caused a mismatch
                                         // intentionally
            _last_adjustment_time = now;
            opt.awaiting_evaluation = true;
            opt.eval_start_time = now;

            const char *dir_str = (opt.probe_direction > 0) ? "up" : "down";
            _logger.logf(true, "TC: Probe %s %.1fA (%+.2fA)", dir_str,
                         new_current, new_current - current);

            // Flip direction for next probe (alternate by default)
            opt.probe_direction *= -1;
        }
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
    bool cold_ok = !_cold_plate.isInError();
    bool hot_ok = _hot_plate.isConnected();

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
// Shared Evaluation Logic
// =============================================================================

bool ThermalController::tryCompleteEvaluation(bool may_transition) {
    float cold_temp = _cold_plate.getTemperature();
    float current = _dps.getTargetCurrent();

    // Delegate evaluation to ThermalMetrics
    EvaluationAction action =
        _metrics.processEvaluation(cold_temp, current, may_transition, _logger);

    if (action.result == EvaluationResult::WAITING)
        return false;

    // Handle revert if needed
    if (action.result == EvaluationResult::DEGRADED) {
        _dps.setSymmetricCurrent(action.revert_current);
        _dps.resetOverrideCounter(); // Intentional change - reset counter

        // Transition to steady state if bounced during ramp
        if (action.should_transition && _state == ThermalState::RAMP_UP) {
            transitionTo(ThermalState::STEADY_STATE);
        }
    }

    return true;
}

// =============================================================================
// History and Analysis
// =============================================================================

void ThermalController::recordSample() {
    ThermalSample sample;
    sample.cold_plate_temp = _cold_plate.getTemperature();
    sample.hot_plate_temp = _hot_plate.getTemperature();
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
