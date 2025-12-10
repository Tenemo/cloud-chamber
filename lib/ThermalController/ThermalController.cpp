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
      _previous_state(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_sample_time(0), _optimizer(), _min_cold_temp_achieved(100.0f),
      _last_adjustment_time(0), _steady_state_start_time(0),
      _ramp_start_time(0), _sensor_fault_time(0) {}

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
    // Core safety checks (sensor health, sanity, thermal limits, DPS, override)
    SafetyResult result = _safety.checkAll();

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
        break; // Continue to optional checks
    }

    // State-specific optional checks (have grace periods)
    bool in_early_ramp =
        (_state == ThermalState::RAMP_UP &&
         (millis() - _ramp_start_time) < PLAUSIBILITY_CHECK_GRACE_MS);

    if (!in_early_ramp && _state != ThermalState::STARTUP) {
        float avg_current = _dps.getTargetCurrent();
        SafetyStatus status =
            _safety.checkPT100Plausibility(avg_current, false);
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
    float adopted = _dps.detectHotReset(HOT_RESET_CURRENT_THRESHOLD_A);
    if (adopted > 0.0f) {
        CrashLog::logCritical("HOT_RESET", "DPS was running on boot");
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

    // Track minimum temperature (session record)
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }

    // Track global best during this ramp (informational, not for hard revert)
    if (cold_temp < _optimizer.best_temp_during_ramp) {
        _optimizer.best_temp_during_ramp = cold_temp;
        _optimizer.current_at_best_temp = current;
    }

    // Evaluate pending current change (with transition on bounce)
    if (processEvaluationIfPending(true)) {
        if (_state != ThermalState::RAMP_UP)
            return; // Transitioned to STEADY_STATE
    }

    // Check termination conditions
    bool at_max = (current >= MAX_CURRENT_PER_CHANNEL);
    bool hot_limited = _safety.isHotSideAlarm();
    bool cooling_stalled = false;

    if (_metrics.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2)) {
        cooling_stalled =
            (fabs(_metrics.getColdPlateRate()) < COOLING_STALL_THRESHOLD_C);
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

    // Before changing current, verify PSUs are settled and we're allowed
    if (!canControlPower())
        return;

    if (!_dps.areBothSettled()) {
        // PSUs haven't processed last command yet - wait
        return;
    }

    // Choose adaptive step size based on cooling rate
    float step = chooseStepSize();
    _optimizer.current_step = step;

    // Record baseline before step (for local revert if needed)
    _optimizer.baseline_current = current;
    _optimizer.baseline_temp = cold_temp;
    _optimizer.baseline_rate = _metrics.getColdPlateRate();

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
    if (processEvaluationIfPending(false)) {
        return; // Evaluation processed, wait for next cycle
    }

    // Periodic optimization probe (bidirectional to find true optimum)
    bool can_probe =
        (now - _last_adjustment_time >= STEADY_STATE_RECHECK_INTERVAL_MS);

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
        _optimizer.baseline_rate = _metrics.getColdPlateRate();

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
// Shared Evaluation Logic
// =============================================================================

bool ThermalController::processEvaluationIfPending(bool allow_transition) {
    if (!_optimizer.awaiting_evaluation)
        return false;

    EvaluationResult result = evaluateCurrentChange();
    if (result == EvaluationResult::WAITING)
        return false;

    float cold_temp = _cold_plate.getTemperature();
    float current = _dps.getTargetCurrent();

    if (result == EvaluationResult::IMPROVED) {
        // Accept new current as session best
        _optimizer.optimal_current = current;
        _optimizer.temp_at_optimal = cold_temp;
        _optimizer.consecutive_bounces = 0;
        _optimizer.converged = false;
        _logger.logf(true, "TC: Opt %.1fA=%.1fC", current, cold_temp);
    } else if (result == EvaluationResult::DEGRADED) {
        // Revert to baseline (local), not global best
        float revert_current = _optimizer.baseline_current;
        if (revert_current < MIN_CURRENT_PER_CHANNEL) {
            revert_current = _optimizer.optimal_current;
        }
        revert_current = fmax(revert_current, MIN_CURRENT_PER_CHANNEL);

        _dps.setSymmetricCurrent(revert_current);
        _optimizer.optimal_current = revert_current;
        _optimizer.temp_at_optimal = _optimizer.baseline_temp;

        // Bounce: flip direction and track
        _optimizer.probe_direction *= -1;
        _optimizer.consecutive_bounces++;

        // Check for convergence (tried both directions)
        if (_optimizer.consecutive_bounces >= 2) {
            _optimizer.converged = true;
            _logger.logf(true, "TC: Converged at %.1fA (%.1fC)",
                         _optimizer.optimal_current,
                         _optimizer.temp_at_optimal);
        } else {
            _logger.logf(true, "TC: Revert %.1fA (bounce #%d)", revert_current,
                         _optimizer.consecutive_bounces);
        }

        // Transition to steady state if bounced during ramp and allowed
        if (allow_transition && _state == ThermalState::RAMP_UP) {
            transitionTo(ThermalState::STEADY_STATE);
        }
    }
    // UNCHANGED: no action needed, optimizer state unchanged

    _optimizer.awaiting_evaluation = false;
    return true;
}

// =============================================================================
// Evaluation Logic
// =============================================================================

EvaluationResult ThermalController::evaluateCurrentChange() {
    unsigned long now = millis();

    // Minimum delay
    if (now - _optimizer.eval_start_time < CURRENT_EVALUATION_DELAY_MS)
        return EvaluationResult::WAITING;

    // Wait for hot-side stabilization (use ThermalMetrics helper)
    bool stable =
        _metrics.isHotSideStable(HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN);
    bool timeout =
        (now - _optimizer.eval_start_time > HOT_SIDE_STABILIZATION_MAX_WAIT_MS);

    if (!stable && !timeout)
        return EvaluationResult::WAITING;

    float cold_temp = _cold_plate.getTemperature();
    float rate = _metrics.getColdPlateRate();
    float rate_delta = rate - _optimizer.baseline_rate;

    bool improved = (cold_temp < _optimizer.baseline_temp -
                                     Tolerance::TEMP_IMPROVEMENT_THRESHOLD_C);
    bool degraded = (rate_delta > COOLING_RATE_DEGRADATION_THRESHOLD);
    bool worsened = (cold_temp > _optimizer.baseline_temp +
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
    //
    // Note: If insufficient history, getColdPlateRate() returns
    // RATE_INSUFFICIENT_HISTORY (-999). Taking fabs() gives a very large value,
    // which correctly biases us towards COARSE_STEP_A during early ramp when
    // we haven't collected enough samples yet. This is intentional: we want
    // fast initial ramp before we have rate data.

    float rate_magnitude = fabs(_metrics.getColdPlateRate());

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
    sample.cold_plate_temp = _cold_plate.getTemperature();
    sample.hot_plate_temp = _hot_plate.getTemperature();
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
