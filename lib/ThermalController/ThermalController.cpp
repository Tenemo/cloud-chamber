/**
 * @file ThermalController.cpp
 * @brief Implementation of the thermal control system
 *
 * Refactored to use delegate modules:
 * - ThermalHistory for data buffering and rate calculations
 * - ThermalMetrics for NVS persistence
 * - SafetyMonitor for centralized safety checks
 * - CrashLog for persistent crash logging
 */

#include "ThermalController.h"
#include <cmath>

// =============================================================================
// Constructor and Initialization
// =============================================================================

ThermalController::ThermalController(Logger &logger, PT100Sensor &coldPlate,
                                     DS18B20Sensor &hotPlate,
                                     DS18B20Sensor *ambientSensors,
                                     size_t numAmbient, DPS5015 *psus)
    : _logger(logger), _cold_plate(coldPlate), _hot_plate(hotPlate),
      _ambient_sensors(ambientSensors), _num_ambient(numAmbient), _psus(psus),
      _metrics(logger), _safety(logger, coldPlate, hotPlate, psus),
      _state(ThermalState::INITIALIZING),
      _previous_state(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_sample_time(0), _target_current{0.0f, 0.0f}, _optimal_current(0.0f),
      _temp_at_optimal(100.0f), _temp_before_last_increase(100.0f),
      _rate_before_last_increase(0.0f), _awaiting_evaluation(false),
      _evaluation_start_time(0), _min_cold_temp_achieved(100.0f),
      _last_adjustment_time(0), _steady_state_start_time(0),
      _ramp_start_time(0), _sensor_fault_time(0), _shutdown_in_progress(false),
      _shutdown_current(0.0f), _last_shutdown_step_time(0), _selftest_phase(0),
      _selftest_phase_start(0), _selftest_passed{false, false},
      _last_imbalance_log_time(0) {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_sample_time = millis();

    // Initialize delegate modules
    _metrics.begin();
    _safety.setHistory(&_history);

    registerDisplayLines();
    _logger.log("ThermalCtrl: init");
}

void ThermalController::registerDisplayLines() {
    _logger.registerTextLine(LINE_STATE, "State:", "INIT");
    _logger.registerLine(LINE_RATE, "dT/dt:", "K/m", 0.0f);
    _logger.registerLine(LINE_I1_SET, "I1 SET:", "A", 0.0f);
    _logger.registerLine(LINE_I2_SET, "I2 SET:", "A", 0.0f);
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

float ThermalController::getTargetCurrent(size_t channel) const {
    return (channel < 2) ? _target_current[channel] : 0.0f;
}

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

    // Update metrics (handles NVS saves with rate limiting)
    _metrics.update();

    // Update hot-side hysteresis tracking
    _safety.updateHysteresis();

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
// State Transitions
// =============================================================================

void ThermalController::transitionTo(ThermalState newState) {
    if (newState == _state)
        return;

    _previous_state = _state;
    _state = newState;
    _state_entry_time = millis();

    char buf[32];
    snprintf(buf, sizeof(buf), "TC: -> %s", getStateString());
    _logger.log(buf);

    // State entry actions
    switch (newState) {
    case ThermalState::STEADY_STATE:
        _steady_state_start_time = millis();
        _min_cold_temp_achieved = _cold_plate.getTemperature();
        break;
    case ThermalState::RAMP_UP:
        _ramp_start_time = millis();
        break;
    case ThermalState::SELF_TEST:
        _selftest_phase = 0;
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
    char buf[48];
    snprintf(buf, sizeof(buf), "TC FAULT: %s", reason);
    _logger.log(buf);
    CrashLog::logCritical("THERMAL_FAULT", reason);

    float hot_temp = _hot_plate.getTemperature();

    // HARD CUT at critical temperature
    if (hot_temp >= HOT_SIDE_FAULT_C) {
        _logger.log("TC: HARD CUT!");
        CrashLog::logCritical("SHUTDOWN", "Hard cut - hot side critical");

        if (!_psus[0].disableOutput()) {
            _logger.log("CRIT: PSU0 shutdown FAIL!");
        }
        if (!_psus[1].disableOutput()) {
            _logger.log("CRIT: PSU1 shutdown FAIL!");
        }

        _target_current[0] = 0;
        _target_current[1] = 0;
        _shutdown_in_progress = false;
    } else {
        // Non-critical: controlled ramp-down
        startEmergencyShutdown();
    }

    _state = ThermalState::THERMAL_FAULT;
    _state_entry_time = millis();
}

// =============================================================================
// Safety Check Wrapper
// =============================================================================

bool ThermalController::runSafetyChecks(bool include_plausibility,
                                        bool include_cross_check) {
    SafetyStatus status = _safety.runAllChecks();

    // Handle result based on status
    switch (status) {
    case SafetyStatus::OK:
        break;

    case SafetyStatus::THERMAL_FAULT:
        enterThermalFault(_safety.getLastFaultReason());
        return false;

    case SafetyStatus::SENSOR_FAULT:
        transitionTo(ThermalState::SENSOR_FAULT);
        return false;

    case SafetyStatus::DPS_DISCONNECTED:
        transitionTo(ThermalState::DPS_DISCONNECTED);
        return false;

    case SafetyStatus::MANUAL_OVERRIDE:
        _logger.log(_safety.getLastFaultReason());
        transitionTo(ThermalState::MANUAL_OVERRIDE);
        return false;

    case SafetyStatus::WARNING:
        // Warnings are logged but don't block
        break;
    }

    // Additional optional checks
    if (include_plausibility && !shouldSkipPlausibilityCheck()) {
        float avg_current = (_target_current[0] + _target_current[1]) / 2.0f;
        status = _safety.checkPT100Plausibility(avg_current, false);
        if (status == SafetyStatus::THERMAL_FAULT) {
            enterThermalFault(_safety.getLastFaultReason());
            return false;
        }
    }

    if (include_cross_check && !shouldSkipCrossCheck()) {
        _safety.checkCrossSensorValidation(false);
        // Cross-check is warning-only, doesn't block
    }

    return true;
}

bool ThermalController::shouldSkipPlausibilityCheck() const {
    if (_state == ThermalState::STARTUP ||
        _state == ThermalState::INITIALIZING ||
        _state == ThermalState::SELF_TEST) {
        return true;
    }
    if (_state == ThermalState::RAMP_UP) {
        return (millis() - _ramp_start_time) < PLAUSIBILITY_CHECK_GRACE_MS;
    }
    return false;
}

bool ThermalController::shouldSkipCrossCheck() const {
    if (_state == ThermalState::RAMP_UP) {
        return (millis() - _ramp_start_time) < SENSOR_CROSS_CHECK_GRACE_MS;
    }
    return false;
}

// =============================================================================
// State Handlers
// =============================================================================

void ThermalController::handleInitializing() {
    unsigned long elapsed = millis() - _state_entry_time;

    bool cold_plate_ok = !_cold_plate.isInError();
    bool hot_plate_ok = _hot_plate.isConnected();
    bool psu0_ok = _psus[0].isConnected();
    bool psu1_ok = _psus[1].isConnected();

    // Check for hot reset (DPS already running)
    if (psu0_ok || psu1_ok) {
        bool output0_on = _psus[0].isOutputOn();
        bool output1_on = _psus[1].isOutputOn();

        if (output0_on || output1_on) {
            float actual_current_0 = _psus[0].getSetCurrent();
            float actual_current_1 = _psus[1].getSetCurrent();

            // Wait for Modbus read to complete
            if ((output0_on &&
                 actual_current_0 < MODBUS_READ_COMPLETE_THRESHOLD_A) ||
                (output1_on &&
                 actual_current_1 < MODBUS_READ_COMPLETE_THRESHOLD_A)) {
                return;
            }

            float max_actual = fmax(actual_current_0, actual_current_1);

            if (max_actual > HOT_RESET_CURRENT_THRESHOLD_A) {
                _logger.log("TC: Hot reset detected");
                CrashLog::logCritical("HOT_RESET", "DPS was running on boot");

                char buf[48];
                snprintf(buf, sizeof(buf), "TC: Adopting %.1fA", max_actual);
                _logger.log(buf);

                float adopted_current =
                    fmin(max_actual, MAX_CURRENT_PER_CHANNEL);
                _target_current[0] = adopted_current;
                _target_current[1] = adopted_current;
                _psus[0].configure(TEC_VOLTAGE_SETPOINT, adopted_current, true);
                _psus[1].configure(TEC_VOLTAGE_SETPOINT, adopted_current, true);

                _optimal_current = adopted_current;
                _temp_at_optimal = _cold_plate.getTemperature();

                if (adopted_current >= MAX_CURRENT_PER_CHANNEL - 0.5f) {
                    transitionTo(ThermalState::STEADY_STATE);
                } else {
                    transitionTo(ThermalState::RAMP_UP);
                }
                return;
            }
        }
    }

    // All hardware ready?
    if (cold_plate_ok && hot_plate_ok && psu0_ok && psu1_ok) {
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

    switch (_selftest_phase) {
    case 0:
        _logger.log("TC: Self-test start");
        for (size_t i = 0; i < 2; i++) {
            _psus[i].setVoltage(DPS_SELFTEST_VOLTAGE);
            _psus[i].setCurrent(DPS_SELFTEST_CURRENT);
            _psus[i].setOutput(false);
        }
        _selftest_phase = 1;
        _selftest_phase_start = now;
        break;

    case 1:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (!_psus[0].isConnected() || !_psus[1].isConnected()) {
            _logger.log("TC: Self-test FAIL: DPS offline");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS offline");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }

        if (fabs(_psus[0].getSetVoltage() - DPS_SELFTEST_VOLTAGE) >
                DPS_SELFTEST_VOLTAGE_TOLERANCE ||
            fabs(_psus[1].getSetVoltage() - DPS_SELFTEST_VOLTAGE) >
                DPS_SELFTEST_VOLTAGE_TOLERANCE) {
            _logger.log("TC: Self-test FAIL: V mismatch");
            CrashLog::logCritical("SELFTEST_FAIL", "Voltage mismatch");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }

        _selftest_phase = 2;
        _selftest_phase_start = now;
        break;

    case 2:
        _psus[0].setOutput(true);
        _selftest_phase = 3;
        _selftest_phase_start = now;
        break;

    case 3:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (_psus[0].isOutputOn()) {
            _selftest_passed[0] = true;
            _psus[0].setOutput(false);
            _selftest_phase = 4;
            _selftest_phase_start = now;
        } else if (phase_elapsed > DPS_SELFTEST_TIMEOUT_MS) {
            _logger.log("TC: Self-test FAIL: DPS0 output");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS0 output");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;

    case 4:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        _psus[1].setOutput(true);
        _selftest_phase = 5;
        _selftest_phase_start = now;
        break;

    case 5:
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (_psus[1].isOutputOn()) {
            _selftest_passed[1] = true;
            _psus[1].setOutput(false);
            _selftest_phase = 6;
            _selftest_phase_start = now;
        } else if (phase_elapsed > DPS_SELFTEST_TIMEOUT_MS) {
            _logger.log("TC: Self-test FAIL: DPS1 output");
            CrashLog::logCritical("SELFTEST_FAIL", "DPS1 output");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;

    case 6:
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
            for (size_t i = 0; i < 2; i++) {
                _psus[i].setVoltage(TEC_VOLTAGE_SETPOINT);
                _psus[i].setCurrent(STARTUP_CURRENT);
                _psus[i].setOutput(true);
                _target_current[i] = STARTUP_CURRENT;
            }
            char buf[32];
            snprintf(buf, sizeof(buf), "TC: Start %.1fA/ch", STARTUP_CURRENT);
            _logger.log(buf);
        }
        return;
    }

    // Run safety checks (no plausibility/cross-check during startup)
    if (!runSafetyChecks(false, false))
        return;

    // Hold for startup duration
    if (elapsed >= STARTUP_HOLD_DURATION_MS) {
        transitionTo(ThermalState::RAMP_UP);
    }
}

void ThermalController::handleRampUp() {
    if (!runSafetyChecks())
        return;

    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();

    // Track minimum temperature
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }

    // Evaluate pending current change
    if (_awaiting_evaluation) {
        EvaluationResult result = evaluateCurrentChange();

        switch (result) {
        case EvaluationResult::WAITING:
            return;

        case EvaluationResult::IMPROVED:
            _optimal_current = _target_current[0];
            _temp_at_optimal = cold_temp;
            {
                char buf[40];
                snprintf(buf, sizeof(buf), "TC: Opt %.1fA=%.1fC",
                         _optimal_current, _temp_at_optimal);
                _logger.log(buf, true);
            }
            _awaiting_evaluation = false;
            break;

        case EvaluationResult::DEGRADED: {
            float previous = _target_current[0] - RAMP_CURRENT_STEP_A;
            previous = fmax(previous, MIN_CURRENT_PER_CHANNEL);

            char buf[48];
            snprintf(buf, sizeof(buf), "TC: Past opt, back %.1fA", previous);
            _logger.log(buf);

            setAllCurrents(previous);
            _optimal_current = previous;
            _temp_at_optimal = _temp_before_last_increase;
            transitionTo(ThermalState::STEADY_STATE);
            return;
        }

        case EvaluationResult::UNCHANGED:
            _awaiting_evaluation = false;
            break;
        }
    }

    // Check termination conditions
    bool at_max = (_target_current[0] >= MAX_CURRENT_PER_CHANNEL &&
                   _target_current[1] >= MAX_CURRENT_PER_CHANNEL);
    bool hot_limited = _safety.isHotSideAlarm();
    bool cooling_stalled = false;

    if (_history.hasMinimumHistory(COOLING_RATE_WINDOW_SAMPLES * 2)) {
        cooling_stalled =
            (fabs(_history.getColdPlateRate()) < COOLING_STALL_THRESHOLD_C);
    }

    if (at_max || hot_limited || cooling_stalled) {
        if (at_max)
            _logger.log("TC: At max current");
        else if (hot_limited)
            _logger.log("TC: Hot side limited");
        else
            _logger.log("TC: Cooling stalled");

        _optimal_current = _target_current[0];
        _temp_at_optimal = cold_temp;
        transitionTo(ThermalState::STEADY_STATE);
        return;
    }

    // Time for adjustment?
    if (now - _last_adjustment_time < RAMP_ADJUSTMENT_INTERVAL_MS) {
        return;
    }

    // Calculate step size (reduced in warning zone)
    float step = RAMP_CURRENT_STEP_A;
    if (_safety.isHotSideWarning()) {
        step = RAMP_CURRENT_STEP_A / 2.0f;
    }

    // Record baseline before increase
    _temp_before_last_increase = cold_temp;
    _rate_before_last_increase = _history.getColdPlateRate();

    // Increase current
    float new_current =
        fmin(_target_current[0] + step, MAX_CURRENT_PER_CHANNEL);
    setAllCurrents(new_current);
    _last_adjustment_time = now;

    _awaiting_evaluation = true;
    _evaluation_start_time = now;

    char buf[32];
    snprintf(buf, sizeof(buf), "TC: Ramp %.1fA", new_current);
    _logger.log(buf, true);
}

void ThermalController::handleSteadyState() {
    if (!runSafetyChecks())
        return;

    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Track temperatures
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }
    _metrics.recordNewMinimum(cold_temp, _target_current[0]);

    // Evaluate pending change
    if (_awaiting_evaluation) {
        EvaluationResult result = evaluateCurrentChange();

        switch (result) {
        case EvaluationResult::WAITING:
            return;

        case EvaluationResult::IMPROVED:
            _optimal_current = _target_current[0];
            _temp_at_optimal = cold_temp;
            _logger.log("TC: New optimal", true);
            _awaiting_evaluation = false;
            break;

        case EvaluationResult::DEGRADED:
        case EvaluationResult::UNCHANGED:
            setAllCurrents(_optimal_current);
            _awaiting_evaluation = false;
            break;
        }
        return;
    }

    // Periodic optimization probe
    if (hot_temp < HOT_SIDE_WARNING_C - 5.0f &&
        _target_current[0] < MAX_CURRENT_PER_CHANNEL &&
        now - _last_adjustment_time >= STEADY_STATE_RECHECK_INTERVAL_MS) {

        _temp_before_last_increase = cold_temp;
        float new_current =
            fmin(_target_current[0] + RAMP_CURRENT_STEP_A / 2.0f,
                 MAX_CURRENT_PER_CHANNEL);
        setAllCurrents(new_current);
        _last_adjustment_time = now;
        _awaiting_evaluation = true;
        _evaluation_start_time = now;
        _logger.log("TC: Steady probe", true);
    }
}

void ThermalController::handleManualOverride() {
    // Monitor only - no commands sent
    // Still check thermal limits for safety
    _safety.checkThermalLimits();
}

void ThermalController::handleThermalFault() { updateEmergencyShutdown(); }

void ThermalController::handleSensorFault() {
    unsigned long elapsed = millis() - _sensor_fault_time;

    // Reduce to safe current
    if (_target_current[0] > DEGRADED_MODE_CURRENT / 2 ||
        _target_current[1] > DEGRADED_MODE_CURRENT / 2) {
        setAllCurrents(DEGRADED_MODE_CURRENT / 2);
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
    bool psu0_ok = _psus[0].isConnected();
    bool psu1_ok = _psus[1].isConnected();

    if (psu0_ok && psu1_ok) {
        _logger.log("TC: DPS reconnected");
        transitionTo(ThermalState::STARTUP);
        return;
    }

    // Symmetric shutdown: if one fails, shut down both
    if (psu0_ok != psu1_ok) {
        size_t working = psu0_ok ? 0 : 1;
        if (_target_current[working] > 0.0f) {
            _logger.log("TC: Symmetric shutdown");
            CrashLog::logCritical("DPS_FAIL", "Symmetric shutdown");
            _target_current[working] = 0.0f;
            _psus[working].setCurrent(0.0f);
            _psus[working].setOutput(false);
        }
    }
}

// =============================================================================
// Control Actions
// =============================================================================

void ThermalController::setAllCurrents(float current) {
    current =
        fmax(MIN_CURRENT_PER_CHANNEL, fmin(current, MAX_CURRENT_PER_CHANNEL));

    // Pre-write validation (skip during shutdown)
    if (!_shutdown_in_progress) {
        for (size_t i = 0; i < 2; i++) {
            if (!_psus[i].isConnected())
                continue;

            if (!_psus[i].validateStateBeforeWrite(current)) {
                char buf[32];
                snprintf(buf, sizeof(buf), "TC: Override PSU%d", (int)i);
                _logger.log(buf);
                transitionTo(ThermalState::MANUAL_OVERRIDE);
                return;
            }
        }
    }

    _target_current[0] = current;
    _target_current[1] = current;

    if (_psus[0].isConnected()) {
        _psus[0].setCurrent(current);
    }
    if (_psus[1].isConnected()) {
        _psus[1].setCurrent(current);
    }
}

void ThermalController::startEmergencyShutdown() {
    if (_shutdown_in_progress)
        return;

    _logger.log("TC: Emergency shutdown");
    CrashLog::logCritical("SHUTDOWN", "Emergency ramp-down");

    _shutdown_in_progress = true;
    _shutdown_current = fmax(_target_current[0], _target_current[1]);
    _last_shutdown_step_time = millis();
}

void ThermalController::updateEmergencyShutdown() {
    if (!_shutdown_in_progress)
        return;

    unsigned long now = millis();
    if (now - _last_shutdown_step_time < EMERGENCY_SHUTDOWN_STEP_MS)
        return;

    _last_shutdown_step_time = now;

    float step = EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC *
                 (EMERGENCY_SHUTDOWN_STEP_MS / 1000.0f);
    _shutdown_current -= step;

    if (_shutdown_current <= 0.1f) {
        _psus[0].disableOutput();
        _psus[1].disableOutput();
        _target_current[0] = 0;
        _target_current[1] = 0;
        _shutdown_in_progress = false;
        _logger.log("TC: Shutdown complete");
        CrashLog::logCritical("SHUTDOWN", "Complete");
    } else {
        _psus[0].setCurrentImmediate(_shutdown_current);
        _psus[1].setCurrentImmediate(_shutdown_current);
    }
}

// =============================================================================
// Evaluation Logic
// =============================================================================

EvaluationResult ThermalController::evaluateCurrentChange() {
    unsigned long now = millis();

    // Minimum delay
    if (now - _evaluation_start_time < CURRENT_EVALUATION_DELAY_MS) {
        return EvaluationResult::WAITING;
    }

    // Wait for hot-side stabilization
    float hot_side_rate = fabs(_history.getHotPlateRate());
    bool hot_stable =
        (hot_side_rate < HOT_SIDE_STABLE_RATE_THRESHOLD_C_PER_MIN);
    bool max_wait_exceeded =
        (now - _evaluation_start_time > HOT_SIDE_STABILIZATION_MAX_WAIT_MS);

    if (!hot_stable && !max_wait_exceeded) {
        return EvaluationResult::WAITING;
    }

    if (max_wait_exceeded && !hot_stable) {
        char buf[48];
        snprintf(buf, sizeof(buf), "TC: Hot rate %.2fK/m, eval anyway",
                 hot_side_rate);
        _logger.log(buf);
    }

    float cold_temp = _cold_plate.getTemperature();
    float cooling_rate = _history.getColdPlateRate();
    float rate_delta = cooling_rate - _rate_before_last_increase;

    bool temp_improved = (cold_temp < _temp_before_last_increase - 0.1f);
    bool rate_degraded = (rate_delta > COOLING_RATE_DEGRADATION_THRESHOLD);
    bool temp_worsened = (cold_temp > _temp_before_last_increase +
                                          OVERCURRENT_WARMING_THRESHOLD_C);

    if (temp_improved && !rate_degraded) {
        return EvaluationResult::IMPROVED;
    } else if (temp_worsened || rate_degraded) {
        return EvaluationResult::DEGRADED;
    } else {
        return EvaluationResult::UNCHANGED;
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
    sample.current_setpoint_ch1 = _target_current[0];
    sample.current_setpoint_ch2 = _target_current[1];
    sample.actual_current_ch1 = _psus[0].getOutputCurrent();
    sample.actual_current_ch2 = _psus[1].getOutputCurrent();
    sample.power_ch1 = _psus[0].getOutputPower();
    sample.power_ch2 = _psus[1].getOutputPower();

    _history.recordSample(sample);
    checkChannelImbalance();
}

float ThermalController::getAmbientTemperature() const {
    if (_num_ambient == 0 || _ambient_sensors == nullptr) {
        return 25.0f;
    }

    for (size_t i = 0; i < _num_ambient; i++) {
        if (_ambient_sensors[i].isConnected()) {
            return _ambient_sensors[i].getTemperature();
        }
    }

    return 25.0f;
}

void ThermalController::checkChannelImbalance() {
    if (!_psus[0].isConnected() || !_psus[1].isConnected())
        return;

    if (fabs(_target_current[0] - _target_current[1]) > 0.1f)
        return;

    unsigned long now = millis();
    if (now - _last_imbalance_log_time < IMBALANCE_LOG_INTERVAL_MS)
        return;

    float i1 = _psus[0].getOutputCurrent();
    float i2 = _psus[1].getOutputCurrent();
    float p1 = _psus[0].getOutputPower();
    float p2 = _psus[1].getOutputPower();

    float current_diff = fabs(i1 - i2);
    float power_diff = fabs(p1 - p2);

    if (current_diff > CHANNEL_CURRENT_IMBALANCE_A ||
        power_diff > CHANNEL_POWER_IMBALANCE_W) {
        char buf[48];
        snprintf(buf, sizeof(buf), "IMBAL: dI=%.1fA dP=%.0fW", current_diff,
                 power_diff);
        _logger.log(buf);
        _last_imbalance_log_time = now;
    }
}

// =============================================================================
// Display
// =============================================================================

void ThermalController::updateDisplay() {
    _logger.updateLineText(LINE_STATE, getStateString());
    _logger.updateLine(LINE_RATE, _history.getColdPlateRate());
    _logger.updateLine(LINE_I1_SET, _target_current[0]);
    _logger.updateLine(LINE_I2_SET, _target_current[1]);
}
