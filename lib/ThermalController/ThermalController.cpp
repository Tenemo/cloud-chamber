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
                                     DS18B20Sensor &hotPlate,
                                     DS18B20Sensor *ambientSensors,
                                     size_t numAmbient, DPS5015 &psu0,
                                     DPS5015 &psu1)
    : _logger(logger), _cold_plate(coldPlate), _hot_plate(hotPlate),
      _ambient_sensors(ambientSensors), _num_ambient(numAmbient),
      _dps(logger, psu0, psu1), _metrics(logger),
      _safety(logger, coldPlate, hotPlate, _dps),
      _state(ThermalState::INITIALIZING),
      _previous_state(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_sample_time(0), _optimizer(), _min_cold_temp_achieved(100.0f),
      _last_adjustment_time(0), _steady_state_start_time(0),
      _ramp_start_time(0), _sensor_fault_time(0), _last_imbalance_log_time(0),
      _selftest_phase(0), _selftest_phase_start(0),
      _selftest_passed{false, false} {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_sample_time = millis();

    _dps.begin();
    _metrics.begin();
    _safety.setHistory(&_history);

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
    return _history.getColdPlateRate();
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
    _dps.update();
    _metrics.update();
    _safety.updateHysteresis();

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
    // Check sensor health FIRST (other checks assume sensors are valid)
    SafetyStatus status = _safety.checkSensorHealth();
    if (status == SafetyStatus::SENSOR_FAULT) {
        transitionTo(ThermalState::SENSOR_FAULT);
        return false;
    }

    // Check sensor sanity (impossible values)
    status = _safety.checkSensorSanity();
    if (status == SafetyStatus::SENSOR_FAULT) {
        transitionTo(ThermalState::SENSOR_FAULT);
        return false;
    }

    // Check thermal limits (assumes sensors are valid from checks above)
    status = _safety.checkThermalLimits();
    if (status == SafetyStatus::THERMAL_FAULT) {
        enterThermalFault(_safety.getLastFaultReason());
        return false;
    }

    // Check DPS connection
    status = _safety.checkDpsConnection();
    if (status == SafetyStatus::DPS_DISCONNECTED) {
        transitionTo(ThermalState::DPS_DISCONNECTED);
        return false;
    }

    // Check manual override (via DualPowerSupply)
    OverrideStatus override = _dps.checkManualOverride();
    if (override == OverrideStatus::DETECTED) {
        _logger.log("TC: Manual override detected");
        transitionTo(ThermalState::MANUAL_OVERRIDE);
        return false;
    }

    // State-specific optional checks
    bool in_early_ramp =
        (_state == ThermalState::RAMP_UP &&
         (millis() - _ramp_start_time) < PLAUSIBILITY_CHECK_GRACE_MS);

    if (!in_early_ramp && _state != ThermalState::STARTUP) {
        float avg_current = _dps.getTargetCurrent();
        status = _safety.checkPT100Plausibility(avg_current, false);
        if (status == SafetyStatus::THERMAL_FAULT) {
            enterThermalFault(_safety.getLastFaultReason());
            return false;
        }
    }

    // Cross-sensor validation (cold plate must be colder than hot plate)
    bool in_cross_check_grace =
        (_state == ThermalState::RAMP_UP &&
         (millis() - _ramp_start_time) < SENSOR_CROSS_CHECK_GRACE_MS);
    _safety.checkCrossSensorValidation(in_cross_check_grace);

    return true;
}

// =============================================================================
// State Transitions
// =============================================================================

void ThermalController::transitionTo(ThermalState newState) {
    if (newState == _state)
        return;

    _previous_state = _state;
    _state = newState;
    _state_entry_time = millis();

    _logger.logf("TC: -> %s", getStateString());

    // State entry actions
    switch (newState) {
    case ThermalState::STEADY_STATE:
        _steady_state_start_time = millis();
        _min_cold_temp_achieved = _cold_plate.getTemperature();
        break;
    case ThermalState::RAMP_UP:
        _ramp_start_time = millis();
        // Initialize global best tracking for this ramp
        _optimizer.best_temp_during_ramp = _cold_plate.getTemperature();
        _optimizer.current_at_best_temp = _dps.getTargetCurrent();
        break;
    case ThermalState::SELF_TEST:
        _selftest_phase = SelfTest::PHASE_START;
        _selftest_phase_start = millis();
        _selftest_passed[0] = false;
        _selftest_passed[1] = false;
        break;
    case ThermalState::SENSOR_FAULT:
        _sensor_fault_time = millis();
        break;
    default:
        break;
    }
}

void ThermalController::enterThermalFault(const char *reason) {
    _logger.logf("TC FAULT: %s", reason);
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
    if (_dps.isEitherConnected() && _dps.isOutputOn()) {
        float actual_current = _dps.getAverageOutputCurrent();

        // Wait for Modbus read to complete
        if (actual_current < Tolerance::MIN_CURRENT_THRESHOLD)
            return;

        if (actual_current > HOT_RESET_CURRENT_THRESHOLD_A) {
            _logger.log("TC: Hot reset detected");
            CrashLog::logCritical("HOT_RESET", "DPS was running on boot");

            float adopted = fmin(actual_current, MAX_CURRENT_PER_CHANNEL);
            _dps.configure(TEC_VOLTAGE_SETPOINT, adopted, true);
            _optimizer.optimal_current = adopted;
            _optimizer.temp_at_optimal = _cold_plate.getTemperature();

            if (adopted >= HOT_RESET_NEAR_MAX_A) {
                transitionTo(ThermalState::STEADY_STATE);
            } else {
                transitionTo(ThermalState::RAMP_UP);
            }
            return;
        }
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
    unsigned long now = millis();
    unsigned long phase_elapsed = now - _selftest_phase_start;

    // Access raw PSUs for self-test
    DPS5015 &psu0 = _dps.getPsu(0);
    DPS5015 &psu1 = _dps.getPsu(1);

    switch (_selftest_phase) {
    case SelfTest::PHASE_START:
        _logger.log("TC: Self-test start");
        psu0.setVoltage(DPS_SELFTEST_VOLTAGE);
        psu0.setCurrent(DPS_SELFTEST_CURRENT);
        psu0.setOutput(false);
        psu1.setVoltage(DPS_SELFTEST_VOLTAGE);
        psu1.setCurrent(DPS_SELFTEST_CURRENT);
        psu1.setOutput(false);
        _selftest_phase = SelfTest::PHASE_CHECK_SETTINGS;
        _selftest_phase_start = now;
        break;

    case SelfTest::PHASE_CHECK_SETTINGS:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (!_dps.areBothConnected()) {
            _logger.log("TC: Self-test FAIL: DPS offline");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS offline");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }

        if (fabs(psu0.getSetVoltage() - DPS_SELFTEST_VOLTAGE) >
                DPS_SELFTEST_VOLTAGE_TOLERANCE ||
            fabs(psu1.getSetVoltage() - DPS_SELFTEST_VOLTAGE) >
                DPS_SELFTEST_VOLTAGE_TOLERANCE) {
            _logger.log("TC: Self-test FAIL: V mismatch");
            CrashLog::logCritical("SELFTEST_FAIL", "Voltage mismatch");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }

        _selftest_phase = SelfTest::PHASE_ENABLE_PSU0;
        _selftest_phase_start = now;
        break;

    case SelfTest::PHASE_ENABLE_PSU0:
        psu0.setOutput(true);
        _selftest_phase = SelfTest::PHASE_VERIFY_PSU0;
        _selftest_phase_start = now;
        break;

    case SelfTest::PHASE_VERIFY_PSU0:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (psu0.isOutputOn()) {
            _selftest_passed[0] = true;
            psu0.setOutput(false);
            _selftest_phase = SelfTest::PHASE_ENABLE_PSU1;
            _selftest_phase_start = now;
        } else if (phase_elapsed > DPS_SELFTEST_TIMEOUT_MS) {
            _logger.log("TC: Self-test FAIL: DPS0 output");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS0 output");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;

    case SelfTest::PHASE_ENABLE_PSU1:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;
        psu1.setOutput(true);
        _selftest_phase = SelfTest::PHASE_VERIFY_PSU1;
        _selftest_phase_start = now;
        break;

    case SelfTest::PHASE_VERIFY_PSU1:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (psu1.isOutputOn()) {
            _selftest_passed[1] = true;
            psu1.setOutput(false);
            _selftest_phase = SelfTest::PHASE_COMPLETE;
            _selftest_phase_start = now;
        } else if (phase_elapsed > DPS_SELFTEST_TIMEOUT_MS) {
            _logger.log("TC: Self-test FAIL: DPS1 output");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS1 output");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;

    case SelfTest::PHASE_COMPLETE:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (_selftest_passed[0] && _selftest_passed[1]) {
            _logger.log("TC: Self-test PASS");
            transitionTo(ThermalState::STARTUP);
        } else {
            _logger.log("TC: Self-test incomplete");
            CrashLog::logCritical("SELFTEST_FAIL", "Incomplete");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;

    default:
        if (millis() - _state_entry_time > DPS_SELFTEST_TIMEOUT_MS * 3) {
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
    if (elapsed < STARTUP_CONFIG_WINDOW_MS) {
        if (elapsed < STARTUP_CONFIG_SEND_MS) {
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

    // Track minimum temperature (session record)
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }

    // Track global best during this ramp (informational, not for hard revert)
    if (cold_temp < _optimizer.best_temp_during_ramp) {
        _optimizer.best_temp_during_ramp = cold_temp;
        _optimizer.current_at_best_temp = current;
    }

    // Evaluate pending current change
    if (_optimizer.awaiting_evaluation) {
        EvaluationResult result = evaluateCurrentChange();

        if (result == EvaluationResult::WAITING)
            return;

        if (result == EvaluationResult::IMPROVED) {
            // Accept new current as session best
            _optimizer.optimal_current = current;
            _optimizer.temp_at_optimal = cold_temp;
            _optimizer.consecutive_bounces =
                0; // Reset bounce counter on success
            _logger.logf(true, "TC: Opt %.1fA=%.1fC",
                         _optimizer.optimal_current,
                         _optimizer.temp_at_optimal);
        } else if (result == EvaluationResult::DEGRADED) {
            // Revert to baseline (local), not global best
            float revert_current = _optimizer.baseline_current;
            if (revert_current < MIN_CURRENT_PER_CHANNEL) {
                revert_current = current - _optimizer.current_step;
            }
            revert_current = fmax(revert_current, MIN_CURRENT_PER_CHANNEL);

            _dps.setSymmetricCurrent(revert_current);
            _optimizer.optimal_current = revert_current;
            _optimizer.temp_at_optimal = _optimizer.baseline_temp;

            // Bounce: flip direction and potentially shrink step
            _optimizer.probe_direction = -1; // Now try decreasing
            _optimizer.consecutive_bounces++;

            _logger.logf(true, "TC: Degraded, back to %.1fA (bounce #%d)",
                         revert_current, _optimizer.consecutive_bounces);
            transitionTo(ThermalState::STEADY_STATE);
            return;
        }
        _optimizer.awaiting_evaluation = false;
    }

    // Check termination conditions
    bool at_max = (current >= MAX_CURRENT_PER_CHANNEL);
    bool hot_limited = _safety.isHotSideAlarm();
    bool cooling_stalled = false;

    if (_history.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2)) {
        cooling_stalled =
            (fabs(_history.getColdPlateRate()) < COOLING_STALL_THRESHOLD_C);
    }

    // Hot limited or cooling stalled - switch to steady state for fine tuning
    if (hot_limited || cooling_stalled) {
        // Use current position as starting point, let steady-state probe refine
        _optimizer.optimal_current = current;
        _optimizer.temp_at_optimal = cold_temp;
        _optimizer.probe_direction = -1; // Start by trying to decrease

        _logger.logf(true, "TC: Limit/stall at %.1fA, fine-tuning", current);
        transitionTo(ThermalState::STEADY_STATE);
        return;
    }

    // Reached max through successful ramps - this is fine
    if (at_max) {
        _optimizer.optimal_current = current;
        _optimizer.temp_at_optimal = cold_temp;
        transitionTo(ThermalState::STEADY_STATE);
        return;
    }

    // Time for adjustment?
    if (now - _last_adjustment_time < RAMP_ADJUSTMENT_INTERVAL_MS)
        return;

    // Choose adaptive step size based on cooling rate
    float step = chooseStepSize();
    _optimizer.current_step = step;

    // Record baseline before step (for local revert if needed)
    _optimizer.baseline_current = current;
    _optimizer.baseline_temp = cold_temp;
    _optimizer.baseline_rate = _history.getColdPlateRate();
    _optimizer.temp_before_step = cold_temp;
    _optimizer.rate_before_step = _optimizer.baseline_rate;

    // Increase current (ramp is always upward)
    float new_current = fmin(current + step, MAX_CURRENT_PER_CHANNEL);
    _dps.setSymmetricCurrent(new_current);
    _last_adjustment_time = now;
    _optimizer.awaiting_evaluation = true;
    _optimizer.eval_start_time = now;

    _logger.logf(true, "TC: Ramp %.1fA (+%.2fA)", new_current, step);
}

void ThermalController::handleSteadyState() {
    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();
    float current = _dps.getTargetCurrent();

    // Track temperatures
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }
    _metrics.recordNewMinimum(cold_temp, current);

    // Evaluate pending change
    if (_optimizer.awaiting_evaluation) {
        EvaluationResult result = evaluateCurrentChange();
        if (result == EvaluationResult::WAITING)
            return;

        if (result == EvaluationResult::IMPROVED) {
            // Accept new current as session best
            _optimizer.optimal_current = current;
            _optimizer.temp_at_optimal = cold_temp;
            _optimizer.consecutive_bounces = 0; // Reset on success
            _optimizer.converged = false;       // Can probe again
            _logger.logf(true, "TC: New optimal %.1fA=%.1fC", current,
                         cold_temp);
        } else {
            // Bad probe: revert to baseline (local), not optimal_current
            float revert_current = _optimizer.baseline_current;
            if (revert_current < MIN_CURRENT_PER_CHANNEL) {
                revert_current = _optimizer.optimal_current;
            }
            _dps.setSymmetricCurrent(revert_current);

            // Bounce: flip direction and track
            _optimizer.probe_direction *= -1;
            _optimizer.consecutive_bounces++;

            // Convergence: if we've bounced twice (tried both directions), stop
            if (_optimizer.consecutive_bounces >= 2) {
                _optimizer.converged = true;
                _logger.logf(true, "TC: Converged at %.1fA (%.1fC)",
                             _optimizer.optimal_current,
                             _optimizer.temp_at_optimal);
            } else {
                _logger.logf(true, "TC: Probe bad, revert %.1fA (bounce #%d)",
                             revert_current, _optimizer.consecutive_bounces);
            }
        }
        _optimizer.awaiting_evaluation = false;
        return;
    }

    // Periodic optimization probe (bidirectional to find true optimum)
    bool can_probe =
        (now - _last_adjustment_time >= STEADY_STATE_RECHECK_INTERVAL_MS);

    if (can_probe) {
        // Reset converged state at each recheck interval - conditions may have
        // changed
        if (_optimizer.converged) {
            _optimizer.converged = false;
            _optimizer.consecutive_bounces = 0;
            _logger.log("TC: Recheck, probing again", true);
        }

        // Choose adaptive step size (smaller when near optimum / bouncing)
        float step = chooseStepSize();
        _optimizer.current_step = step;

        // Record baseline before step
        _optimizer.baseline_current = current;
        _optimizer.baseline_temp = cold_temp;
        _optimizer.baseline_rate = _history.getColdPlateRate();
        _optimizer.temp_before_step = cold_temp;
        _optimizer.rate_before_step = _optimizer.baseline_rate;

        // Calculate target current based on probe direction
        float new_current;
        bool can_step = false;

        if (_optimizer.probe_direction > 0) {
            // Try increasing (if we have room and thermal headroom)
            if (current < MAX_CURRENT_PER_CHANNEL &&
                hot_temp < HOT_SIDE_WARNING_C - 5.0f) {
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
            _last_adjustment_time = now;
            _optimizer.awaiting_evaluation = true;
            _optimizer.eval_start_time = now;

            const char *dir_str =
                (_optimizer.probe_direction > 0) ? "up" : "down";
            _logger.logf(true, "TC: Probe %s %.1fA (%+.2fA)", dir_str,
                         new_current, new_current - current);

            // Flip direction for next probe (alternate by default)
            _optimizer.probe_direction *= -1;
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
        transitionTo(_previous_state);
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
// Evaluation Logic
// =============================================================================

EvaluationResult ThermalController::evaluateCurrentChange() {
    unsigned long now = millis();

    // Minimum delay
    if (now - _optimizer.eval_start_time < CURRENT_EVALUATION_DELAY_MS)
        return EvaluationResult::WAITING;

    // Wait for hot-side stabilization
    float hot_rate = fabs(_history.getHotPlateRate());
    bool stable = (hot_rate < HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN);
    bool timeout =
        (now - _optimizer.eval_start_time > HOT_SIDE_STABILIZATION_MAX_WAIT_MS);

    if (!stable && !timeout)
        return EvaluationResult::WAITING;

    float cold_temp = _cold_plate.getTemperature();
    float rate = _history.getColdPlateRate();
    float rate_delta = rate - _optimizer.rate_before_step;

    bool improved = (cold_temp < _optimizer.temp_before_step -
                                     Tolerance::TEMP_IMPROVEMENT_THRESHOLD_C);
    bool degraded = (rate_delta > COOLING_RATE_DEGRADATION_THRESHOLD);
    bool worsened = (cold_temp > _optimizer.temp_before_step +
                                     OVERCURRENT_WARMING_THRESHOLD_C);

    if (improved && !degraded)
        return EvaluationResult::IMPROVED;
    if (worsened || degraded)
        return EvaluationResult::DEGRADED;
    return EvaluationResult::UNCHANGED;
}

float ThermalController::chooseStepSize() const {
    // Choose adaptive step size based on cooling rate magnitude and hot-side
    // Use larger steps when far from optimum (fast cooling) for speed
    // Use smaller steps when near optimum (slow cooling) for precision

    float rate_magnitude = fabs(_history.getColdPlateRate());

    // Hot-side in warning zone: cap at medium step regardless of rate
    if (_safety.isHotSideWarning()) {
        // If we've bounced multiple times, use fine step
        if (_optimizer.consecutive_bounces >= 2) {
            return FINE_STEP_A;
        }
        return MEDIUM_STEP_A;
    }

    // If we've bounced multiple times, use finer steps
    if (_optimizer.consecutive_bounces >= 2) {
        return FINE_STEP_A;
    }

    // Select based on cooling rate magnitude
    if (rate_magnitude > STEP_COARSE_RATE_THRESHOLD) {
        // Fast cooling - far from optimum, use coarse steps
        return COARSE_STEP_A;
    } else if (rate_magnitude > STEP_MEDIUM_RATE_THRESHOLD) {
        // Moderate cooling - approaching optimum
        return MEDIUM_STEP_A;
    } else {
        // Slow cooling - near optimum, use fine steps
        return FINE_STEP_A;
    }
}

// =============================================================================
// History and Analysis
// =============================================================================

void ThermalController::recordSample() {
    ThermalSample sample;
    sample.timestamp_ms = millis();
    sample.cold_plate_temp = _cold_plate.getTemperature();
    sample.hot_plate_temp = _hot_plate.getTemperature();
    sample.ambient_temp = getAmbientTemperature();
    sample.current_setpoint = _dps.getTargetCurrent();
    sample.actual_current_avg = _dps.getAverageOutputCurrent();
    sample.total_power = _dps.getTotalPower();

    _history.recordSample(sample);
    checkChannelImbalance();
}

float ThermalController::getAmbientTemperature() const {
    if (_num_ambient == 0 || _ambient_sensors == nullptr)
        return 25.0f;

    for (size_t i = 0; i < _num_ambient; i++) {
        if (_ambient_sensors[i].isConnected()) {
            return _ambient_sensors[i].getTemperature();
        }
    }
    return 25.0f;
}

void ThermalController::checkChannelImbalance() {
    if (!_dps.areBothConnected())
        return;

    unsigned long now = millis();
    if (now - _last_imbalance_log_time < Timing::IMBALANCE_LOG_INTERVAL_MS)
        return;

    float current_diff = _dps.getCurrentImbalance();
    float power_diff = _dps.getPowerImbalance();

    if (current_diff > CHANNEL_CURRENT_IMBALANCE_A ||
        power_diff > CHANNEL_POWER_IMBALANCE_W) {
        _logger.logf("IMBAL: dI=%.1fA dP=%.0fW", current_diff, power_diff);
        _last_imbalance_log_time = now;
    }
}

// =============================================================================
// Display
// =============================================================================

void ThermalController::updateDisplay() {
    using namespace ThermalDisplay;
    _logger.updateLineText(LINE_STATE, getStateString());
    _logger.updateLine(LINE_RATE, _history.getColdPlateRate());
    _logger.updateLine(LINE_CURRENT, _dps.getTargetCurrent());
}
