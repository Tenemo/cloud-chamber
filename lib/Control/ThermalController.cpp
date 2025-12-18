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
      _benchmark(logger),
      _state(ThermalState::INITIALIZING),
      _state_before_fault(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_sample_time(0), _last_adjustment_time(0),
      _steady_state_start_time(0), _ramp_start_time(0), _sensor_fault_time(0),
      _hot_reset_active(false), _hot_reset_current(0.0f) {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_sample_time = millis();

    _dps.begin();
    _metrics.begin();
    _safety.setMetrics(&_metrics);

    registerDisplayLines();
    _logger.log("Controller initialized.");
}

void ThermalController::beginBenchmark(const float *currents_a, size_t count,
                                       unsigned long hold_ms,
                                       float warmup_temp_c) {
    _benchmark.begin(currents_a, count, hold_ms, warmup_temp_c);
    begin();
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
        const bool in_manual = (_state == ThermalState::MANUAL_OVERRIDE);
        _metrics.recordSample(_sensors, _dps,
                              /*enable_imbalance_checks=*/!in_manual,
                              /*use_reported_set_current=*/in_manual);
        _last_sample_time = now;
    }

    // Update delegate modules
    _metrics.update();

    // Continue emergency shutdown if in progress
    if (_dps.isShutdownInProgress()) {
        _dps.updateEmergencyShutdown();
    }

    // Always enforce hot-side hard limit, even if DPS comms are down
    float hot_temp = _sensors.getHotPlateTemperature();
    if (_state != ThermalState::THERMAL_FAULT &&
        hot_temp >= Limits::HOT_SIDE_FAULT_C) {
        enterThermalFault("Hot side >= fault threshold");
        updateDisplay();
        return;
    }

    // Run safety checks ONCE at top of loop (not in each handler)
    // Skip for non-operational states (hardware not ready or already faulted)
    if (isOperationalState()) {
        // Centralized manual override detection (outside SafetyMonitor).
        // Only check while we're actively controlling; once in override or
        // disconnected states we want the state handler + safety checks to run.
        const bool should_check_override =
            (_state == ThermalState::STARTUP || _state == ThermalState::RAMP_UP ||
             _state == ThermalState::STEADY_STATE);
        if (should_check_override) {
            OverrideInfo override = _dps.checkOverrideDetail();
            if (override.cause == OverrideCause::HUMAN_OVERRIDE) {
                transitionTo(ThermalState::MANUAL_OVERRIDE, override.reason);
                updateDisplay();
                return;
            }
            if (override.cause == OverrideCause::CONTROL_MISMATCH) {
                transitionTo(ThermalState::DPS_DISCONNECTED, override.reason);
                updateDisplay();
                return;
            }
        }

        runSafetyChecks();
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

void ThermalController::runSafetyChecks() {
    // All safety checks including grace period logic are now in SafetyMonitor
    // Use actual average current (not target) for physics-based plausibility
    // check
    SafetyResult result = _safety.checkAll(_state, _ramp_start_time,
                                           _dps.getAverageOutputCurrent());

    switch (result.status) {
    case SafetyStatus::SENSOR_FAULT:
        transitionTo(ThermalState::SENSOR_FAULT);
        break;

    case SafetyStatus::THERMAL_FAULT:
        enterThermalFault(result.reason);
        break;

    case SafetyStatus::DPS_DISCONNECTED:
        transitionTo(ThermalState::DPS_DISCONNECTED);
        break;

    case SafetyStatus::OK:
    case SafetyStatus::WARNING:
        break;
    }
}

bool ThermalController::isOperationalState() const {
    // States where safety checks should NOT run:
    // - INITIALIZING: Hardware not ready yet, sensors may not be valid
    // - SELF_TEST: Running DPS verification, not operational
    // - THERMAL_FAULT: Already in fault state, shutdown in progress
    return _state != ThermalState::INITIALIZING &&
           _state != ThermalState::SELF_TEST &&
           _state != ThermalState::THERMAL_FAULT;
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

void ThermalController::transitionTo(ThermalState newState,
                                     const char *reason) {
    if (newState == _state)
        return;

    _state_before_fault = _state;
    _state = newState;
    _state_entry_time = millis();
    if (newState == ThermalState::DPS_DISCONNECTED) {
        _dps_disconnect_log_time = 0; // reset rate-limit timer
        _dps_restore_in_progress = false;
        _dps_restore_start_time = 0;
        _dps_restore_current = 0.0f;
        _dps_restore_output = true;
        _dps_restore_state = ThermalState::STARTUP;
    }

    if (reason) {
        _logger.logf(false, "TC: -> %s (%s)", stateToString(_state), reason);
    } else {
        _logger.logf(false, "TC: -> %s", stateToString(_state));
    }

    // State entry actions
    switch (newState) {
    case ThermalState::STARTUP:
    {
        // Normal boot path - clear hot reset state since we're starting fresh
        _hot_reset_active = false;
        _hot_reset_current = 0.0f;

        // Configure PSUs immediately on entering STARTUP
        const float startup_current = Limits::STARTUP_CURRENT;
        _dps.configure(Limits::TEC_VOLTAGE_SETPOINT, startup_current, true);
        _logger.logf(false, "TC: Config %.0fV %.2fA",
                     Limits::TEC_VOLTAGE_SETPOINT, startup_current);
        break;
    }
    case ThermalState::STEADY_STATE:
        // Reached stable operation - clear hot reset recovery state
        _hot_reset_active = false;
        _hot_reset_current = 0.0f;
        _steady_state_start_time = millis();
        _optimizer.clearPendingEvaluation(); // avoid carrying ramp evaluation

        // If no adjustment was ever made (e.g., hot reset recovery that
        // immediately exited ramp), allow immediate first probe since we
        // already waited during stabilization
        if (_last_adjustment_time == 0) {
            _last_adjustment_time =
                millis() - Timing::STEADY_STATE_RECHECK_INTERVAL_MS;
        }
        break;
    case ThermalState::RAMP_UP:
        _ramp_start_time = millis();
        _consecutive_stall_detects.reset();
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
    _logger.logf(true, "CRIT: THERMAL_FAULT %s", reason);

    float hot_temp = _sensors.getHotPlateTemperature();

    if (hot_temp >= Limits::HOT_SIDE_FAULT_C) {
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

    // 1. Detect Hot Reset (Run only once)
    // We check if we haven't already activated hot reset logic
    if (!_hot_reset_active) {
        // Check if ANY connected PSU is running
        float adopted = _dps.detectHotReset(HOT_RESET_CURRENT_THRESHOLD_A);

        if (adopted > 0.0f) {
            _logger.logf(true, "TC: Hot reset detected, adopting %.2fA",
                         adopted);

            _hot_reset_active = true;
            _hot_reset_current = adopted;

            // Configure expected state, but DO NOT transition yet.
            // This queues writes for currently connected PSUs and sets
            // pending config for the ones yet to connect.
            _dps.configure(Limits::TEC_VOLTAGE_SETPOINT, adopted, true);

            // Seed optimizer immediately so we don't start from 0
            float current_temp = _sensors.getColdPlateTemperature();
            _optimizer.seedWithCurrent(adopted, current_temp);
        }
    }

    // 2. Wait for ALL hardware to be ready
    if (cold_plate_ok && hot_plate_ok && both_psu_ok) {

        // If we are in hot reset mode, we skip self-test but we MUST ensure
        // the PSUs are settled (commands processed) before entering RAMP_UP
        if (_hot_reset_active) {
            if (!_dps.areBothSettled()) {
                // Wait for grace periods and pending writes to clear
                return;
            }

            if (_hot_reset_current >= HOT_RESET_NEAR_MAX_A) {
                transitionTo(ThermalState::STEADY_STATE, "Hot reset near max");
            } else {
                transitionTo(ThermalState::RAMP_UP, "Hot reset recovery");
            }
        } else {
            // Normal boot -> Go to Self Test
            transitionTo(ThermalState::SELF_TEST);
        }
        return;
    }

    // 3. Timeout Logic
    if (elapsed > Timing::INIT_TIMEOUT_MS) {
        if (!cold_plate_ok || !hot_plate_ok) {
            _logger.log("CRIT: INIT_FAIL Sensor timeout", true);
            transitionTo(ThermalState::SENSOR_FAULT);
        } else {
            // Note: In hot reset, if one PSU is dead, we might want to try
            // running on one? For now, stick to safe symmetric policy.
            _logger.log("CRIT: INIT_FAIL DPS timeout", true);
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
        if (millis() - _state_entry_time >
            InternalTiming::SELFTEST_TIMEOUT_MS * 3) {
            _logger.log("TC: Self-test timeout");
            _logger.log("CRIT: SELFTEST_FAIL Timeout", true);
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;
    }
}

void ThermalController::handleStartup() {
    unsigned long elapsed = millis() - _state_entry_time;

    // Configuration is done in transitionTo() when entering STARTUP
    // Just wait for the startup hold duration
    if (elapsed >= Timing::STARTUP_HOLD_DURATION_MS) {
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

        // --- Pre-flight Integrity Check ---
        // Before changing current, verify reality matches our last expectation.
        // We use a stateless check here (no counters) for immediate rejection.
        if (_dps.hasAnyMismatch()) {
            _logger.log("TC: Pre-flight mismatch detected!");
            transitionTo(ThermalState::MANUAL_OVERRIDE, "Pre-flight check");
            return;
        }
        // -----------------------------------

        _dps.setSymmetricCurrent(decision.new_current);
        _dps.resetOverrideCounter();
        _last_adjustment_time = snapshot_now;
    }
}

void ThermalController::handleRampUp() {
    ThermalSnapshot snapshot = _metrics.buildSnapshot(_sensors, _dps, _safety);

    if (_benchmark.isEnabled()) {
        Benchmark::StepResult step =
            _benchmark.update(_state, snapshot, _dps, _last_adjustment_time);
        if (step.should_transition) {
            transitionTo(step.next_state, step.reason);
        }
        return;
    }

    float current = snapshot.current_setpoint;
    float cooling_rate = snapshot.cooling_rate;

    // Check termination conditions via optimizer
    bool has_enough_history =
        _metrics.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2);
    bool stall_detected =
        has_enough_history &&
        snapshot.current_setpoint >= Tuning::MIN_CURRENT_FOR_STALL_CHECK_A &&
        fabs(cooling_rate) < Tuning::COOLING_STALL_THRESHOLD_C;

    // Require multiple consecutive stall detections before exiting ramp
    if (stall_detected) {
        _consecutive_stall_detects.inc();
    } else {
        _consecutive_stall_detects.reset();
    }

    bool should_exit = _optimizer.shouldExitRamp(snapshot, has_enough_history);
    bool exit_on_stall = stall_detected && _consecutive_stall_detects.atLeast(3);

    char exit_reason_buf[96];
    const char *exit_reason = "Max current";

    if (should_exit &&
        (_safety.isHotSideAlarm() ||
         snapshot.current_setpoint >= Limits::MAX_CURRENT_PER_CHANNEL ||
         exit_on_stall)) {
        _optimizer.updateBest(current, snapshot.cold_temp);

        if (_safety.isHotSideAlarm()) {
            _optimizer.setProbeDirection(-1);
            exit_reason = "Hot side limit";
        } else if (exit_on_stall) {
            // After a stall-driven exit we still want to probe upward first
            // to continue searching for the optimum.
            _optimizer.setProbeDirection(1);
            unsigned long window_seconds =
                (COOLING_RATE_WINDOW_SAMPLES * HISTORY_SAMPLE_INTERVAL_MS) /
                1000;
            snprintf(exit_reason_buf, sizeof(exit_reason_buf),
                     "Cooling stalled (3x): rate %.3f K/m < %.3f over %lus",
                     cooling_rate, Tuning::COOLING_STALL_THRESHOLD_C,
                     window_seconds);
            exit_reason = exit_reason_buf;
        }

        transitionTo(ThermalState::STEADY_STATE, exit_reason);
        return;
    }

    // Let optimizer decide what to do
    bool psus_ready = canControlPower() && _dps.areBothSettled();
    ThermalOptimizationDecision decision = _optimizer.update(
        snapshot, ThermalControlPhase::RAMP_UP, _last_adjustment_time,
        Timing::RAMP_ADJUSTMENT_INTERVAL_MS, psus_ready);

    applyOptimizationDecision(decision, snapshot.now);

    // Handle revert on degraded evaluation
    if (decision.evaluation_complete && decision.should_transition) {
        transitionTo(ThermalState::STEADY_STATE, "Degradation detected");
    }
}

void ThermalController::handleSteadyState() {
    ThermalSnapshot snapshot = _metrics.buildSnapshot(_sensors, _dps, _safety);

    if (_benchmark.isEnabled()) {
        Benchmark::StepResult step =
            _benchmark.update(_state, snapshot, _dps, _last_adjustment_time);
        if (step.should_transition) {
            transitionTo(step.next_state, step.reason);
        }
        return;
    }

    // If we're warming up after reaching "steady", react quickly by stepping
    // down instead of waiting the full recheck interval.
    unsigned long adjustment_interval_ms = Timing::STEADY_STATE_RECHECK_INTERVAL_MS;
    const bool has_rate = snapshot.cooling_rate != RATE_INSUFFICIENT_HISTORY;
    const bool warming =
        has_rate && snapshot.cooling_rate > Tuning::COOLING_STALL_THRESHOLD_C;
    const bool temp_regressed =
        snapshot.cold_temp >
        (_optimizer.getTempAtOptimal() + Tuning::OVERCURRENT_WARMING_THRESHOLD_C);
    if (warming && temp_regressed) {
        adjustment_interval_ms = Timing::CURRENT_EVALUATION_DELAY_MS;
        _optimizer.setProbeDirection(-1);
    }

    // Reset converged state at recheck interval
    if (_optimizer.isConverged()) {
        if (_metrics.isTimeForAdjustment(_last_adjustment_time,
                                         Timing::STEADY_STATE_RECHECK_INTERVAL_MS)) {
            _optimizer.clearConverged();
            _logger.log("TC: Recheck, probing again", true);
        }
    }

    // Let optimizer decide what to do
    bool psus_ready = canControlPower() && _dps.areBothSettled();
    ThermalOptimizationDecision decision = _optimizer.update(
        snapshot, ThermalControlPhase::STEADY_STATE, _last_adjustment_time,
        adjustment_interval_ms, psus_ready);

    applyOptimizationDecision(decision, snapshot.now);
}

void ThermalController::handleManualOverride() {}

void ThermalController::handleThermalFault() {}

void ThermalController::handleSensorFault() {
    unsigned long elapsed = millis() - _sensor_fault_time;

    // Reduce to safe current
    if (_dps.getTargetCurrent() > Limits::DEGRADED_MODE_CURRENT / 2) {
        _dps.setSymmetricCurrent(Limits::DEGRADED_MODE_CURRENT / 2);
    }

    // Check recovery
    bool cold_ok = !_sensors.isColdPlateError();
    bool hot_ok = _sensors.isHotPlateConnected();

    if (cold_ok && hot_ok) {
        _logger.log("TC: Sensors recovered");
        transitionTo(_state_before_fault);
        return;
    }

    if (elapsed > Timing::SENSOR_RECOVERY_TIMEOUT_MS) {
        _logger.log("CRIT: SENSOR_FAULT Recovery timeout", true);
        enterThermalFault("Sensor timeout");
    }
}

void ThermalController::handleDpsDisconnected() {
    unsigned long now = millis();
    constexpr unsigned long DPS_RESTORE_TIMEOUT_MS = 10000;

    // If we are not fully reconnected, wait quietly and cancel any restore
    // attempt.
    if (!_dps.areBothConnected()) {
        _dps_restore_in_progress = false;

        if (_dps_disconnect_log_time == 0 ||
            now - _dps_disconnect_log_time > 5000) { // log at most every 5s
            _logger.log("TC: DPS disconnected, waiting for reconnection");
            _dps_disconnect_log_time = now;
        }
        return;
    }

    // Reconnected - restore last known setpoint/state and resume.
    if (!_dps_restore_in_progress) {
        _logger.log("TC: DPS reconnected");

        ThermalState resume_state = _state_before_fault;
        float resume_current = _dps.getTargetCurrent();
        bool resume_output = _dps.getTargetOutput();

        // Hot reset recovery: prefer adopted current and resume path
        if (_hot_reset_active && _hot_reset_current > 0.0f) {
            resume_current = _hot_reset_current;
            resume_output = true;
            resume_state = (_hot_reset_current >= HOT_RESET_NEAR_MAX_A)
                               ? ThermalState::STEADY_STATE
                               : ThermalState::RAMP_UP;
        }

        // Defensive fallbacks
        if (resume_state == ThermalState::INITIALIZING ||
            resume_state == ThermalState::SELF_TEST) {
            _dps_restore_in_progress = false;
            transitionTo(ThermalState::SELF_TEST, "DPS reconnected");
            return;
        }
        if (resume_state == ThermalState::DPS_DISCONNECTED ||
            resume_state == ThermalState::THERMAL_FAULT) {
            resume_state = ThermalState::STARTUP;
            resume_current = Limits::STARTUP_CURRENT;
            resume_output = true;
        }

        // Clamp to sane operational range (per-channel setpoint)
        if (resume_output) {
            if (resume_current < Limits::MIN_CURRENT_PER_CHANNEL) {
                resume_current = Limits::STARTUP_CURRENT;
            } else if (resume_current > Limits::MAX_CURRENT_PER_CHANNEL) {
                resume_current = Limits::MAX_CURRENT_PER_CHANNEL;
            }
        } else {
            if (resume_current < 0.0f) {
                resume_current = 0.0f;
            } else if (resume_current > Limits::MAX_CURRENT_PER_CHANNEL) {
                resume_current = Limits::MAX_CURRENT_PER_CHANNEL;
            }
        }

        _dps_restore_state = resume_state;
        _dps_restore_current = resume_current;
        _dps_restore_output = resume_output;
        _dps_restore_in_progress = true;
        _dps_restore_start_time = now;

        // STARTUP transition handles its own PSU configuration.
        if (_dps_restore_state == ThermalState::STARTUP) {
            _dps_restore_in_progress = false;
            transitionTo(ThermalState::STARTUP, "DPS reconnected");
            return;
        }

        _dps.configure(Limits::TEC_VOLTAGE_SETPOINT, _dps_restore_current,
                       _dps_restore_output);
        return;
    }

    // Wait until both PSUs have processed our restore commands, then resume.
    if (_dps.areBothSettled()) {
        _dps_restore_in_progress = false;
        transitionTo(_dps_restore_state, "DPS restored");
        return;
    }

    // If the restore never settles (e.g., due to comm flakiness), retry
    // periodically instead of getting stuck forever.
    if (_dps_restore_start_time != 0 &&
        (now - _dps_restore_start_time) > DPS_RESTORE_TIMEOUT_MS) {
        _logger.log("TC: DPS restore timeout, retrying");
        _dps_restore_start_time = now;
        _dps.configure(Limits::TEC_VOLTAGE_SETPOINT, _dps_restore_current,
                       _dps_restore_output);
    }
}

void ThermalController::updateDisplay() {
    float display_current = _dps.getTargetCurrent();
    if (_state == ThermalState::MANUAL_OVERRIDE) {
        display_current = _dps.getAverageSetCurrent();
    }
    _metrics.updateDisplay(stateToString(_state), display_current);
}
