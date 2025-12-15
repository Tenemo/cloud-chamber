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
    if (isOperationalState()) {
        // Centralized manual override detection (outside SafetyMonitor)
        OverrideStatus override = _dps.checkManualOverride();
        if (override == OverrideStatus::DETECTED) {
            _logger.log("TC: Manual override detected");
            transitionTo(ThermalState::MANUAL_OVERRIDE);
            updateDisplay();
            return;
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

void ThermalController::transitionTo(ThermalState newState,
                                     const char *reason) {
    if (newState == _state)
        return;

    _state_before_fault = _state;
    _state = newState;
    _state_entry_time = millis();

    if (reason) {
        _logger.logf(false, "TC: -> %s (%s)", stateToString(_state), reason);
    } else {
        _logger.logf(false, "TC: -> %s", stateToString(_state));
    }

    // State entry actions
    switch (newState) {
    case ThermalState::STARTUP:
        // Normal boot path - clear hot reset state since we're starting fresh
        _hot_reset_active = false;
        _hot_reset_current = 0.0f;

        // Configure PSUs immediately on entering STARTUP
        _dps.configure(TEC_VOLTAGE_SETPOINT, STARTUP_CURRENT, true);
        _logger.logf(false, "TC: Config %.0fV %.2fA", TEC_VOLTAGE_SETPOINT,
                     STARTUP_CURRENT);
        break;
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
            _last_adjustment_time = millis() - STEADY_STATE_RECHECK_INTERVAL_MS;
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
            _dps.configure(TEC_VOLTAGE_SETPOINT, adopted, true);

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
    if (elapsed > INIT_TIMEOUT_MS) {
        if (!cold_plate_ok || !hot_plate_ok) {
            CrashLog::logCritical("INIT_FAIL", "Sensor timeout");
            transitionTo(ThermalState::SENSOR_FAULT);
        } else {
            // Note: In hot reset, if one PSU is dead, we might want to try
            // running on one? For now, stick to safe symmetric policy.
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
    float current = snapshot.current_setpoint;
    float cooling_rate = snapshot.cooling_rate;

    // Check termination conditions via optimizer
    bool has_enough_history =
        _metrics.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2);
    bool stall_detected =
        has_enough_history &&
        snapshot.current_setpoint >= MIN_CURRENT_FOR_STALL_CHECK_A &&
        fabs(cooling_rate) < COOLING_STALL_THRESHOLD_C;

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
         snapshot.current_setpoint >= MAX_CURRENT_PER_CHANNEL ||
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
                     cooling_rate, COOLING_STALL_THRESHOLD_C, window_seconds);
            exit_reason = exit_reason_buf;
        }

        transitionTo(ThermalState::STEADY_STATE, exit_reason);
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
        transitionTo(ThermalState::STEADY_STATE, "Degradation detected");
    }
}

void ThermalController::handleSteadyState() {
    ThermalSnapshot snapshot = _metrics.buildSnapshot(_sensors, _dps, _safety);

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

        // If we were in hot reset recovery, restore the adopted current
        // instead of going through STARTUP which would reset to STARTUP_CURRENT
        if (_hot_reset_active && _hot_reset_current > 0.0f) {
            _dps.configure(TEC_VOLTAGE_SETPOINT, _hot_reset_current, true);
            transitionTo(ThermalState::RAMP_UP, "Hot reset restore");
        } else {
            transitionTo(ThermalState::STARTUP);
        }
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
