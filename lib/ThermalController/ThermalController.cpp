/**
 * @file ThermalController.cpp
 * @brief Implementation of the thermal control system
 */

#include "ThermalController.h"
#include <cmath>

ThermalController::ThermalController(Logger &logger, PT100Sensor &coldPlate,
                                     DS18B20Sensor &hotPlate,
                                     DS18B20Sensor *ambientSensors,
                                     size_t numAmbient, DPS5015 *psus)
    : _logger(logger), _cold_plate(coldPlate), _hot_plate(hotPlate),
      _ambient_sensors(ambientSensors), _num_ambient(numAmbient), _psus(psus),
      _state(ThermalState::INITIALIZING),
      _previous_state(ThermalState::INITIALIZING), _state_entry_time(0),
      _last_update_time(0), _target_current{0.0f, 0.0f}, _optimal_current(0.0f),
      _temp_at_optimal(100.0f), _temp_before_last_increase(100.0f),
      _rate_before_last_increase(0.0f),
      _awaiting_evaluation(false), _evaluation_start_time(0),
      _min_cold_temp_achieved(100.0f), _last_adjustment_time(0),
      _cooling_rate(0.0f), _steady_state_start_time(0), _history_head(0),
      _history_count(0), _last_sample_time(0), _sensor_fault_time(0),
      _dps_was_connected{false, false}, _shutdown_in_progress(false),
      _shutdown_current(0.0f), _last_shutdown_step_time(0),
      _hot_side_in_warning(false), _hot_side_in_alarm(false) {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_update_time = millis();
    _last_sample_time = millis();

    registerDisplayLines();
    _logger.log("ThermalCtrl: init");
}

void ThermalController::registerDisplayLines() {
    _logger.registerTextLine(LINE_STATE, "State:", "INIT");
    _logger.registerLine(LINE_RATE, "dT/dt:", "K/m", 0.0f);
    _logger.registerLine(LINE_I1_SET, "I1 SET:", "A", 0.0f);
    _logger.registerLine(LINE_I2_SET, "I2 SET:", "A", 0.0f);
}

const char *ThermalController::getStateString() const {
    switch (_state) {
    case ThermalState::INITIALIZING:
        return "INIT";
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

float ThermalController::getTargetCurrent(size_t channel) const {
    if (channel < 2) {
        return _target_current[channel];
    }
    return 0.0f;
}

unsigned long ThermalController::getSteadyStateUptime() const {
    if (_state == ThermalState::STEADY_STATE) {
        return millis() - _steady_state_start_time;
    }
    return 0;
}

void ThermalController::update() {
    unsigned long now = millis();

    // Record history sample at regular intervals
    if (now - _last_sample_time >= HISTORY_SAMPLE_INTERVAL_MS) {
        recordSample();
        _last_sample_time = now;
        _cooling_rate = calculateCoolingRate();
    }

    // Run state-specific handler
    switch (_state) {
    case ThermalState::INITIALIZING:
        handleInitializing();
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
    _last_update_time = now;
}

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
    default:
        break;
    }
}

// ============================================================================
// State Handlers
// ============================================================================

void ThermalController::handleInitializing() {
    unsigned long elapsed = millis() - _state_entry_time;

    // Check if all hardware is ready
    bool cold_plate_ok = !_cold_plate.isInError();
    bool hot_plate_ok = _hot_plate.isConnected();
    bool psu0_ok = _psus[0].isConnected();
    bool psu1_ok = _psus[1].isConnected();

    // Check for DPS units already running (possible watchdog reset / crash)
    if (psu0_ok || psu1_ok) {
        bool output0_on = _psus[0].isOutputOn();
        bool output1_on = _psus[1].isOutputOn();

        if (output0_on || output1_on) {
            // DPS is running - read actual current to avoid thermal shock
            float actual_current_0 = _psus[0].getSetCurrent();
            float actual_current_1 = _psus[1].getSetCurrent();
            float max_actual = fmax(actual_current_0, actual_current_1);

            if (max_actual > STARTUP_CURRENT) {
                // System was running at higher current - adopt it to avoid
                // thermal shock from sudden current drop
                _logger.log("TC: Hot reset detected");

                char buf[48];
                snprintf(buf, sizeof(buf), "TC: Adopting %.1fA", max_actual);
                _logger.log(buf);

                // Set target current to actual (clamped to max)
                float adopted_current = fmin(max_actual, MAX_CURRENT_PER_CHANNEL);
                _target_current[0] = adopted_current;
                _target_current[1] = adopted_current;
                _psus[0].configure(TEC_VOLTAGE_SETPOINT, adopted_current, true);
                _psus[1].configure(TEC_VOLTAGE_SETPOINT, adopted_current, true);

                // Track for optimal current detection
                _optimal_current = adopted_current;
                _temp_at_optimal = _cold_plate.getTemperature();

                // Skip STARTUP, go directly to RAMP_UP or STEADY_STATE
                if (adopted_current >= MAX_CURRENT_PER_CHANNEL - 0.5f) {
                    transitionTo(ThermalState::STEADY_STATE);
                } else {
                    transitionTo(ThermalState::RAMP_UP);
                }
                return;
            } else {
                // Low current - safe to restart normally but log it
                _logger.log("TC: DPS active, low I");
            }
        }
    }

    // All critical sensors and PSUs ready?
    if (cold_plate_ok && hot_plate_ok && psu0_ok && psu1_ok) {
        transitionTo(ThermalState::STARTUP);
        return;
    }

    // Timeout waiting for hardware
    if (elapsed > INIT_TIMEOUT_MS) {
        if (!cold_plate_ok || !hot_plate_ok) {
            transitionTo(ThermalState::SENSOR_FAULT);
        } else {
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
    }
}

void ThermalController::handleStartup() {
    unsigned long elapsed = millis() - _state_entry_time;

    // First 100ms: Configure PSUs (only runs once on state entry)
    // This timing ensures the transition has completed before sending commands
    if (elapsed < 100) {
        for (size_t i = 0; i < 2; i++) {
            _psus[i].setVoltage(TEC_VOLTAGE_SETPOINT);
            _psus[i].setCurrent(STARTUP_CURRENT);
            _psus[i].setOutput(true);
            _target_current[i] = STARTUP_CURRENT;
        }
        char buf[32];
        snprintf(buf, sizeof(buf), "TC: Start %.1fA/ch", STARTUP_CURRENT);
        _logger.log(buf);
        return;
    }

    // Safety checks during startup
    if (!checkThermalLimits())
        return;
    if (!checkSensorHealth())
        return;
    if (!checkDpsConnection())
        return;
    if (checkForManualOverride())
        return;

    // Hold for startup duration
    if (elapsed >= STARTUP_HOLD_DURATION_MS) {
        transitionTo(ThermalState::RAMP_UP);
    }
}

void ThermalController::handleRampUp() {
    // Safety checks
    if (!checkThermalLimits())
        return;
    if (!checkSensorHealth())
        return;
    if (!checkDpsConnection())
        return;
    if (checkForManualOverride())
        return;

    unsigned long now = millis();
    float hot_temp = _hot_plate.getTemperature();
    float cold_temp = _cold_plate.getTemperature();

    // Update hysteresis state for hot side
    if (_hot_side_in_alarm) {
        if (hot_temp < HOT_SIDE_ALARM_EXIT_C)
            _hot_side_in_alarm = false;
    } else {
        if (hot_temp >= HOT_SIDE_ALARM_C)
            _hot_side_in_alarm = true;
    }

    if (_hot_side_in_warning) {
        if (hot_temp < HOT_SIDE_WARNING_EXIT_C)
            _hot_side_in_warning = false;
    } else {
        if (hot_temp >= HOT_SIDE_WARNING_C)
            _hot_side_in_warning = true;
    }

    // Track minimum achieved temperature
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }

    // If awaiting evaluation of a current increase, check if it helped
    if (_awaiting_evaluation) {
        if (now - _evaluation_start_time < CURRENT_EVALUATION_DELAY_MS) {
            return; // Still waiting for system to stabilize
        }

        _awaiting_evaluation = false;

        // DERIVATIVE CONTROL: Check both absolute temperature AND cooling rate
        // The cooling rate is more responsive than absolute temperature due to
        // thermal inertia of the water cooling loop

        float rate_delta = _cooling_rate - _rate_before_last_increase;
        // Note: cooling_rate is negative when cooling (K/min)
        // If rate becomes less negative (or positive), cooling has slowed

        bool temp_improved = (cold_temp < _temp_before_last_increase - 0.1f);
        bool rate_degraded = (rate_delta > COOLING_RATE_DEGRADATION_THRESHOLD);
        bool temp_worsened =
            (cold_temp > _temp_before_last_increase + OVERCURRENT_WARMING_THRESHOLD_C);

        if (temp_improved && !rate_degraded) {
            // Temperature improved and rate didn't degrade - this current is
            // better
            _optimal_current = _target_current[0];
            _temp_at_optimal = cold_temp;
            char buf[40];
            snprintf(buf, sizeof(buf), "TC: Opt %.1fA=%.1fC", _optimal_current,
                     _temp_at_optimal);
            _logger.log(buf, true);
        } else if (temp_worsened || rate_degraded) {
            // Temperature got WORSE or cooling rate degraded significantly
            // This indicates we've passed the inflection point
            float previous_current = _target_current[0] - RAMP_CURRENT_STEP_A;
            if (previous_current < MIN_CURRENT_PER_CHANNEL) {
                previous_current = MIN_CURRENT_PER_CHANNEL;
            }

            char buf[48];
            if (rate_degraded && !temp_worsened) {
                snprintf(buf, sizeof(buf), "TC: Rate deg, back %.1fA",
                         previous_current);
            } else {
                snprintf(buf, sizeof(buf), "TC: Past opt, back %.1fA",
                         previous_current);
            }
            _logger.log(buf);

            setAllCurrents(previous_current);
            _optimal_current = previous_current;
            _temp_at_optimal = _temp_before_last_increase;

            // Transition to steady state - we found the optimum
            transitionTo(ThermalState::STEADY_STATE);
            return;
        }
        // else: temperature about the same and rate stable, continue ramping
    }

    // Check if we've reached max current on both channels
    bool at_max = (_target_current[0] >= MAX_CURRENT_PER_CHANNEL &&
                   _target_current[1] >= MAX_CURRENT_PER_CHANNEL);

    // Check if hot side is limiting further increases (uses hysteresis)
    bool hot_limited = _hot_side_in_alarm;

    // Check if cooling has stalled (rate near zero for extended period)
    bool cooling_stalled = false;
    if (_history_count >= COOLING_RATE_WINDOW_SAMPLES * 2) {
        cooling_stalled = (fabs(_cooling_rate) < COOLING_STALL_THRESHOLD_C);
    }

    // Transition to STEADY_STATE when we can't improve further
    if (at_max || hot_limited || cooling_stalled) {
        if (at_max) {
            _logger.log("TC: At max current");
        } else if (hot_limited) {
            _logger.log("TC: Hot side limited");
        } else {
            _logger.log("TC: Cooling stalled");
        }
        _optimal_current = _target_current[0];
        _temp_at_optimal = cold_temp;
        transitionTo(ThermalState::STEADY_STATE);
        return;
    }

    // Time for a current adjustment?
    if (now - _last_adjustment_time < RAMP_ADJUSTMENT_INTERVAL_MS) {
        return;
    }

    // Determine step size based on hot side temperature (uses hysteresis)
    float step = RAMP_CURRENT_STEP_A;
    if (_hot_side_in_warning) {
        step = RAMP_CURRENT_STEP_A / 2.0f; // Half step when in warning zone
    }

    // Record temperature AND cooling rate before increase for later comparison
    _temp_before_last_increase = cold_temp;
    _rate_before_last_increase = _cooling_rate;

    // Increase current on both channels equally
    float new_current =
        fmin(_target_current[0] + step, MAX_CURRENT_PER_CHANNEL);
    setAllCurrents(new_current);
    _last_adjustment_time = now;

    // Start evaluation period
    _awaiting_evaluation = true;
    _evaluation_start_time = now;

    char buf[32];
    snprintf(buf, sizeof(buf), "TC: Ramp %.1fA", new_current);
    _logger.log(buf, true);
}

void ThermalController::handleSteadyState() {
    // Safety checks
    if (!checkThermalLimits())
        return;
    if (!checkSensorHealth())
        return;
    if (!checkDpsConnection())
        return;
    if (checkForManualOverride())
        return;

    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Track minimum achieved temperature
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }

    // If awaiting evaluation of an increase attempt
    if (_awaiting_evaluation) {
        if (now - _evaluation_start_time < CURRENT_EVALUATION_DELAY_MS) {
            return;
        }

        _awaiting_evaluation = false;

        if (cold_temp < _temp_before_last_increase - 0.1f) {
            // It helped! Update optimal
            _optimal_current = _target_current[0];
            _temp_at_optimal = cold_temp;
            _logger.log("TC: New optimal", true);
        } else {
            // Didn't help or made it worse - back off
            setAllCurrents(_optimal_current);
        }
        return;
    }

    // Periodically try a small increase if conditions are favorable
    // Only if hot side has headroom and we're not already at max
    if (hot_temp < HOT_SIDE_WARNING_C - 5.0f &&
        _target_current[0] < MAX_CURRENT_PER_CHANNEL &&
        now - _last_adjustment_time >= STEADY_STATE_RECHECK_INTERVAL_MS) {

        // Record baseline and try a small bump
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
    // Just monitor - don't send any commands
    // This state can only be exited by power cycle
    checkThermalLimits(); // Still watch for thermal faults
}

void ThermalController::handleThermalFault() {
    // Continue non-blocking shutdown if in progress
    updateEmergencyShutdown();
    // Latched state otherwise - do nothing except continue monitoring display
}

void ThermalController::handleSensorFault() {
    unsigned long elapsed = millis() - _sensor_fault_time;

    // Reduce to safe current
    if (_target_current[0] > DEGRADED_MODE_CURRENT / 2 ||
        _target_current[1] > DEGRADED_MODE_CURRENT / 2) {
        setAllCurrents(DEGRADED_MODE_CURRENT / 2);
    }

    // Check if sensors recovered
    bool cold_ok = !_cold_plate.isInError();
    bool hot_ok = _hot_plate.isConnected();

    if (cold_ok && hot_ok) {
        _logger.log("TC: Sensors recovered");
        transitionTo(_previous_state);
        return;
    }

    // Timeout - go to thermal fault as precaution
    if (elapsed > SENSOR_RECOVERY_TIMEOUT_MS) {
        enterThermalFault("Sensor timeout");
    }
}

void ThermalController::handleDpsDisconnected() {
    // Check if PSUs reconnected
    bool psu0_ok = _psus[0].isConnected();
    bool psu1_ok = _psus[1].isConnected();

    if (psu0_ok && psu1_ok) {
        _logger.log("TC: DPS reconnected");
        // Restart from STARTUP for safety
        transitionTo(ThermalState::STARTUP);
        return;
    }

    // If one is connected, operate in degraded mode
    if (psu0_ok || psu1_ok) {
        size_t working = psu0_ok ? 0 : 1;
        if (_target_current[working] > DEGRADED_MODE_CURRENT) {
            setChannelCurrent(working, DEGRADED_MODE_CURRENT);
        }
    }
}

// ============================================================================
// Safety Checks
// ============================================================================

bool ThermalController::checkForManualOverride() {
    for (size_t i = 0; i < 2; i++) {
        if (!_psus[i].isConnected())
            continue;

        // Skip check if we recently sent a command (settling window)
        // This prevents false positives during Modbus propagation delay
        if (_psus[i].isInGracePeriod())
            continue;

        // Check if DPS is settled (current matches what we commanded)
        if (!_psus[i].isSettled()) {
            // Current doesn't match commanded after grace period
            // This indicates user manually changed the dial
            _logger.log("TC: Manual override");
            transitionTo(ThermalState::MANUAL_OVERRIDE);
            return true;
        }
    }
    return false;
}

bool ThermalController::checkThermalLimits() {
    float hot_temp = _hot_plate.getTemperature();

    // Critical hot side fault
    if (hot_temp >= HOT_SIDE_FAULT_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "HOT>%.0fC", hot_temp);
        enterThermalFault(buf);
        return false;
    }

    // Hot side rate check (using history if available)
    if (_history_count >= 60) {
        // Calculate hot side rate over last minute
        size_t oldest_idx =
            (_history_head + HISTORY_BUFFER_SIZE - 60) % HISTORY_BUFFER_SIZE;
        float old_hot = _history[oldest_idx].hot_plate_temp;
        float rate_per_min = hot_temp - old_hot;

        if (rate_per_min > HOT_SIDE_RATE_FAULT_C_PER_MIN) {
            enterThermalFault("RUNAWAY");
            return false;
        }
    }

    return true;
}

bool ThermalController::checkSensorHealth() {
    bool cold_ok = !_cold_plate.isInError();
    bool hot_ok = _hot_plate.isConnected();

    if (!cold_ok || !hot_ok) {
        _sensor_fault_time = millis();
        transitionTo(ThermalState::SENSOR_FAULT);
        return false;
    }
    return true;
}

bool ThermalController::checkDpsConnection() {
    bool psu0_ok = _psus[0].isConnected();
    bool psu1_ok = _psus[1].isConnected();

    // Track connection state
    bool was0 = _dps_was_connected[0];
    bool was1 = _dps_was_connected[1];
    _dps_was_connected[0] = psu0_ok;
    _dps_was_connected[1] = psu1_ok;

    // Detect disconnection
    if ((was0 && !psu0_ok) || (was1 && !psu1_ok)) {
        _logger.log("TC: DPS disconnected");
        transitionTo(ThermalState::DPS_DISCONNECTED);
        return false;
    }

    return true;
}

// ============================================================================
// Control Actions
// ============================================================================

void ThermalController::setChannelCurrent(size_t channel, float current) {
    if (channel >= 2)
        return;

    current =
        fmax(MIN_CURRENT_PER_CHANNEL, fmin(current, MAX_CURRENT_PER_CHANNEL));

    // Pre-write validation: detect manual override before commanding
    // Skip validation if this is a forced command (e.g., emergency shutdown)
    if (_psus[channel].isConnected() && !_shutdown_in_progress) {
        if (!_psus[channel].validateStateBeforeWrite(current)) {
            // DPS state doesn't match expectations - user touched the dial
            _logger.log("TC: Override detected");
            transitionTo(ThermalState::MANUAL_OVERRIDE);
            return;
        }
    }

    _target_current[channel] = current;
    _psus[channel].setCurrent(current);
}

void ThermalController::setAllCurrents(float current) {
    setChannelCurrent(0, current);
    setChannelCurrent(1, current);
}

void ThermalController::enterThermalFault(const char *reason) {
    char buf[48];
    snprintf(buf, sizeof(buf), "TC FAULT: %s", reason);
    _logger.log(buf);

    // Check if this is a critical fault requiring HARD CUT
    // At HOT_SIDE_FAULT_C, thermal shock is preferable to component damage
    float hot_temp = _hot_plate.getTemperature();
    if (hot_temp >= HOT_SIDE_FAULT_C) {
        // HARD CUT - immediate output disable, no ramp
        _logger.log("TC: HARD CUT!");
        _psus[0].disableOutput();
        _psus[1].disableOutput();
        _target_current[0] = 0;
        _target_current[1] = 0;
        _shutdown_in_progress = false; // No ramp needed
    } else {
        // Non-critical fault - use controlled ramp-down
        startEmergencyShutdown();
    }

    _state = ThermalState::THERMAL_FAULT;
    _state_entry_time = millis();
}

void ThermalController::startEmergencyShutdown() {
    if (_shutdown_in_progress)
        return;

    _logger.log("TC: Emergency shutdown");
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

    // Calculate step to ramp down over ~5 seconds
    float step = EMERGENCY_RAMP_DOWN_RATE_A_PER_SEC *
                 (EMERGENCY_SHUTDOWN_STEP_MS / 1000.0f);
    _shutdown_current -= step;

    if (_shutdown_current <= 0.1f) {
        // Final step: disable outputs
        _psus[0].disableOutput();
        _psus[1].disableOutput();
        _target_current[0] = 0;
        _target_current[1] = 0;
        _shutdown_in_progress = false;
        _logger.log("TC: Shutdown complete");
    } else {
        // Ramp down current
        _psus[0].setCurrentImmediate(_shutdown_current);
        _psus[1].setCurrentImmediate(_shutdown_current);
    }
}

void ThermalController::emergencyShutdown() {
    // Legacy blocking version - only for use during INITIALIZING
    // when we need to shut down DPS that was left running
    _logger.log("TC: Emergency shutdown");

    float current = fmax(_target_current[0], _target_current[1]);
    if (current < 0.1f) {
        current = 5.0f; // Assume some current if we don't know
    }

    // Quick ramp down - fewer steps for blocking version
    for (int i = 0; i < 5 && current > 0.1f; i++) {
        current -= current / 3.0f;
        if (current < 0.1f)
            current = 0;

        _psus[0].setCurrentImmediate(current);
        _psus[1].setCurrentImmediate(current);
        delay(200); // 200ms per step = 1 second total max
    }

    // Disable outputs
    _psus[0].disableOutput();
    _psus[1].disableOutput();

    _target_current[0] = 0;
    _target_current[1] = 0;
}

void ThermalController::requestManualOverride() {
    transitionTo(ThermalState::MANUAL_OVERRIDE);
}

// ============================================================================
// History and Analysis
// ============================================================================

void ThermalController::recordSample() {
    ThermalSample &sample = _history[_history_head];

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

    _history_head = (_history_head + 1) % HISTORY_BUFFER_SIZE;
    if (_history_count < HISTORY_BUFFER_SIZE) {
        _history_count++;
    }
}

float ThermalController::calculateCoolingRate() const {
    if (_history_count < COOLING_RATE_WINDOW_SAMPLES) {
        return 0.0f;
    }

    // Simple linear regression over the window
    size_t n = COOLING_RATE_WINDOW_SAMPLES;
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

    for (size_t i = 0; i < n; i++) {
        size_t idx =
            (_history_head + HISTORY_BUFFER_SIZE - n + i) % HISTORY_BUFFER_SIZE;
        float x = static_cast<float>(i);
        float y = _history[idx].cold_plate_temp;

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (fabs(denom) < 0.0001f) {
        return 0.0f;
    }

    // Slope in degrees per sample
    float slope = (n * sum_xy - sum_x * sum_y) / denom;

    // Convert to K/min (samples are 1 second apart)
    return slope * 60.0f;
}

ThermalTrend ThermalController::analyzeTrend() const {
    if (_history_count < 60) {
        return ThermalTrend::ANOMALOUS;
    }

    float rate = _cooling_rate;

    if (rate < -COOLING_STALL_THRESHOLD_C) {
        return ThermalTrend::COOLING;
    } else if (rate > COOLING_STALL_THRESHOLD_C) {
        return ThermalTrend::WARMING;
    } else {
        return ThermalTrend::STABLE;
    }
}

float ThermalController::getAmbientTemperature() const {
    if (_num_ambient == 0 || _ambient_sensors == nullptr) {
        return 25.0f; // Default assumption
    }

    // Return first connected ambient sensor reading
    for (size_t i = 0; i < _num_ambient; i++) {
        if (_ambient_sensors[i].isConnected()) {
            return _ambient_sensors[i].getTemperature();
        }
    }

    return 25.0f;
}

// ============================================================================
// Display Updates
// ============================================================================

void ThermalController::updateDisplay() {
    _logger.updateLineText(LINE_STATE, getStateString());
    _logger.updateLine(LINE_RATE, _cooling_rate);
    _logger.updateLine(LINE_I1_SET, _target_current[0]);
    _logger.updateLine(LINE_I2_SET, _target_current[1]);
}
