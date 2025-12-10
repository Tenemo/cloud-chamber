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
      _rate_before_last_increase(0.0f), _awaiting_evaluation(false),
      _evaluation_start_time(0), _min_cold_temp_achieved(100.0f),
      _last_adjustment_time(0), _cooling_rate(0.0f),
      _steady_state_start_time(0), _history_head(0), _history_count(0),
      _last_sample_time(0), _sensor_fault_time(0),
      _dps_was_connected{false, false}, _shutdown_in_progress(false),
      _shutdown_current(0.0f), _last_shutdown_step_time(0),
      _hot_side_in_warning(false), _hot_side_in_alarm(false),
      _last_imbalance_log_time(0), _ramp_start_time(0),
      _last_cross_check_log_time(0), _cross_check_warning_active(false),
      _last_metrics_save_time(0), _last_runtime_save_time(0),
      _session_start_time(0), _total_runtime_seconds(0),
      _all_time_min_temp(100.0f), _all_time_optimal_current(0.0f),
      _session_count(0), _selftest_phase(0), _selftest_phase_start(0),
      _selftest_passed{false, false} {}

void ThermalController::begin() {
    _state_entry_time = millis();
    _last_update_time = millis();
    _last_sample_time = millis();
    _session_start_time = millis();

    // Load persisted metrics from NVS
    loadMetricsFromNvs();

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

    // Update runtime counter and save metrics periodically
    updateRuntimeCounter();
    saveMetricsToNvs();

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
    case ThermalState::RAMP_UP:
        _ramp_start_time = millis();
        break;
    case ThermalState::SELF_TEST:
        _selftest_phase = 0;
        _selftest_phase_start = millis();
        _selftest_passed[0] = false;
        _selftest_passed[1] = false;
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
            // IMPORTANT: Verify the read was successful (not default 0)
            // The first Modbus read may not have completed yet
            float actual_current_0 = _psus[0].getSetCurrent();
            float actual_current_1 = _psus[1].getSetCurrent();

            // Validate that we got real readings (not uninitialized 0)
            // If DPS output is ON but current reads as 0, the Modbus read
            // hasn't completed yet - wait for next cycle
            if ((output0_on && actual_current_0 < 0.01f) ||
                (output1_on && actual_current_1 < 0.01f)) {
                // Modbus read incomplete - wait for next update cycle
                // This prevents adopting 0A when DPS is actually running
                return;
            }

            float max_actual = fmax(actual_current_0, actual_current_1);

            if (max_actual > STARTUP_CURRENT) {
                // System was running at higher current - adopt it to avoid
                // thermal shock from sudden current drop
                _logger.log("TC: Hot reset detected");

                char buf[48];
                snprintf(buf, sizeof(buf), "TC: Adopting %.1fA", max_actual);
                _logger.log(buf);

                // Set target current to actual (clamped to max)
                float adopted_current =
                    fmin(max_actual, MAX_CURRENT_PER_CHANNEL);
                _target_current[0] = adopted_current;
                _target_current[1] = adopted_current;
                _psus[0].configure(TEC_VOLTAGE_SETPOINT, adopted_current, true);
                _psus[1].configure(TEC_VOLTAGE_SETPOINT, adopted_current, true);

                // Track for optimal current detection
                _optimal_current = adopted_current;
                _temp_at_optimal = _cold_plate.getTemperature();

                // Skip self-test and STARTUP on hot reset
                // Go directly to RAMP_UP or STEADY_STATE
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
        // Run self-test before starting normal operation
        transitionTo(ThermalState::SELF_TEST);
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

    // Phase 1: Configure PSUs (first 500ms)
    // Extended from 100ms to ensure queue is processed before safety checks
    // With 6 commands per PSU and 500ms Modbus timeout, need adequate time
    if (elapsed < 500) {
        // Only configure once at the start (when elapsed is very small)
        if (elapsed < 50) {
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

    // Safety checks during startup
    if (!checkThermalLimits())
        return;
    if (!checkSensorHealth())
        return;
    if (!checkSensorSanity())
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

/**
 * @brief Handle DPS self-test state
 *
 * Self-test phases:
 * 0: Start - set both DPS to safe test voltage/current, output OFF
 * 1: Verify settings applied
 * 2: Enable output on DPS0
 * 3: Verify DPS0 output enabled, disable it
 * 4: Enable output on DPS1
 * 5: Verify DPS1 output enabled, disable it
 * 6: Complete - all tests passed
 */
void ThermalController::handleSelfTest() {
    unsigned long now = millis();
    unsigned long phase_elapsed = now - _selftest_phase_start;

    switch (_selftest_phase) {
    case 0: {
        // Phase 0: Configure both DPS to safe test settings with output OFF
        _logger.log("TC: Self-test start");
        for (size_t i = 0; i < 2; i++) {
            _psus[i].setVoltage(DPS_SELFTEST_VOLTAGE);
            _psus[i].setCurrent(DPS_SELFTEST_CURRENT);
            _psus[i].setOutput(false);
        }
        _selftest_phase = 1;
        _selftest_phase_start = now;
        break;
    }

    case 1: {
        // Phase 1: Wait for settings to apply, verify communication
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        // Check if both DPS responded
        bool ok = _psus[0].isConnected() && _psus[1].isConnected();
        if (!ok) {
            _logger.log("TC: Self-test FAIL: DPS offline");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }

        // Verify voltage setting was accepted (with tolerance)
        float v0 = _psus[0].getSetVoltage();
        float v1 = _psus[1].getSetVoltage();
        if (fabs(v0 - DPS_SELFTEST_VOLTAGE) > 0.2f ||
            fabs(v1 - DPS_SELFTEST_VOLTAGE) > 0.2f) {
            char buf[48];
            snprintf(buf, sizeof(buf), "TC: Self-test FAIL: V0=%.1f V1=%.1f",
                     v0, v1);
            _logger.log(buf);
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }

        _selftest_phase = 2;
        _selftest_phase_start = now;
        break;
    }

    case 2: {
        // Phase 2: Enable output on DPS0
        _psus[0].setOutput(true);
        _selftest_phase = 3;
        _selftest_phase_start = now;
        break;
    }

    case 3: {
        // Phase 3: Verify DPS0 output enabled, then disable
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (_psus[0].isOutputOn()) {
            _selftest_passed[0] = true;
            _psus[0].setOutput(false);
            _selftest_phase = 4;
            _selftest_phase_start = now;
        } else if (phase_elapsed > DPS_SELFTEST_TIMEOUT_MS) {
            _logger.log("TC: Self-test FAIL: DPS0 output");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }
        break;
    }

    case 4: {
        // Phase 4: Enable output on DPS1
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return; // Wait for DPS0 to turn off

        _psus[1].setOutput(true);
        _selftest_phase = 5;
        _selftest_phase_start = now;
        break;
    }

    case 5: {
        // Phase 5: Verify DPS1 output enabled, then disable
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return;

        if (_psus[1].isOutputOn()) {
            _selftest_passed[1] = true;
            _psus[1].setOutput(false);
            _selftest_phase = 6;
            _selftest_phase_start = now;
        } else if (phase_elapsed > DPS_SELFTEST_TIMEOUT_MS) {
            _logger.log("TC: Self-test FAIL: DPS1 output");
            transitionTo(ThermalState::DPS_DISCONNECTED);
            return;
        }
        break;
    }

    case 6: {
        // Phase 6: All tests passed
        if (phase_elapsed < DPS_SELFTEST_SETTLE_MS)
            return; // Wait for DPS1 to turn off

        if (_selftest_passed[0] && _selftest_passed[1]) {
            _logger.log("TC: Self-test PASS");
            transitionTo(ThermalState::STARTUP);
        } else {
            _logger.log("TC: Self-test incomplete");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;
    }

    default: {
        // Timeout safeguard
        if (millis() - _state_entry_time > DPS_SELFTEST_TIMEOUT_MS * 3) {
            _logger.log("TC: Self-test timeout");
            transitionTo(ThermalState::DPS_DISCONNECTED);
        }
        break;
    }
    }
}

void ThermalController::handleRampUp() {
    // Safety checks
    if (!checkThermalLimits())
        return;
    if (!checkSensorHealth())
        return;
    if (!checkSensorSanity())
        return;
    if (!checkPT100Plausibility())
        return; // Physics-based PT100 validation
    if (!checkCrossSensorValidation())
        return; // Cross-validate cold < hot after grace period
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
        bool temp_worsened = (cold_temp > _temp_before_last_increase +
                                              OVERCURRENT_WARMING_THRESHOLD_C);

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
    if (!checkSensorSanity())
        return;
    if (!checkPT100Plausibility())
        return; // Physics-based PT100 validation
    if (!checkCrossSensorValidation())
        return;
    if (!checkDpsConnection())
        return;
    if (checkForManualOverride())
        return;

    unsigned long now = millis();
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Track minimum achieved temperature (session and all-time)
    if (cold_temp < _min_cold_temp_achieved) {
        _min_cold_temp_achieved = cold_temp;
    }
    if (cold_temp < _all_time_min_temp) {
        _all_time_min_temp = cold_temp;
        _all_time_optimal_current = _target_current[0];
        // Force save when we achieve a new record
        saveMetricsToNvs(true);
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

        // Skip check if there are pending writes or we're in grace period
        // This prevents false positives during Modbus propagation delay
        if (_psus[i].hasPendingWrites() || _psus[i].isInGracePeriod())
            continue;

        // Require N consecutive mismatches before declaring override
        // This filters out transient read errors or timing glitches
        if (_psus[i].getConsecutiveMismatches() >=
            MANUAL_OVERRIDE_MISMATCH_COUNT) {
            // Sustained mismatch detected - user manually changed the dial
            _logger.log("TC: Manual override");
            transitionTo(ThermalState::MANUAL_OVERRIDE);
            return true;
        }
    }
    return false;
}

bool ThermalController::checkThermalLimits() {
    // Hot-side sensor fault is an immediate safety concern
    // We cannot trust thermal limits if we can't read the temperature
    if (!_hot_plate.isConnected()) {
        enterThermalFault("HOT SENSOR LOST");
        return false;
    }

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

/**
 * @brief Cross-validate sensor readings
 *
 * During normal cooling operation (after grace period), the cold plate
 * should always be colder than the hot plate. If not, either:
 * - Sensors are swapped/misconfigured
 * - A sensor is reading incorrectly
 * - TECs are not cooling (failed or reversed polarity)
 *
 * This is a warning, not a fault, because it could be a transient condition.
 */
bool ThermalController::checkCrossSensorValidation() {
    // Only check after grace period in RAMP_UP
    if (_state == ThermalState::RAMP_UP) {
        unsigned long ramp_elapsed = millis() - _ramp_start_time;
        if (ramp_elapsed < SENSOR_CROSS_CHECK_GRACE_MS) {
            return true; // Still in grace period
        }
    }

    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Cold plate should be significantly colder than hot plate
    if (cold_temp >= hot_temp - SENSOR_CROSS_CHECK_MARGIN_C) {
        unsigned long now = millis();

        if (!_cross_check_warning_active) {
            _cross_check_warning_active = true;
            char buf[48];
            snprintf(buf, sizeof(buf), "WARN: Cold>=Hot! C=%.1f H=%.1f",
                     cold_temp, hot_temp);
            _logger.log(buf);
            _last_cross_check_log_time = now;
        } else if (now - _last_cross_check_log_time >=
                   SENSOR_CROSS_CHECK_LOG_INTERVAL_MS) {
            // Rate-limited repeat warning
            char buf[48];
            snprintf(buf, sizeof(buf), "WARN: Cold>=Hot C=%.1f H=%.1f",
                     cold_temp, hot_temp);
            _logger.log(buf);
            _last_cross_check_log_time = now;
        }

        // This is a warning, not a fault - TECs might just need time
        // But if this persists, something is wrong
    } else {
        // Condition cleared
        if (_cross_check_warning_active) {
            _cross_check_warning_active = false;
            _logger.log("Cross-check OK: Cold<Hot");
        }
    }

    return true; // Always return true - this is informational, not blocking
}

/**
 * @brief Check for impossible sensor values
 *
 * Detects obviously wrong readings that indicate sensor malfunction:
 * - Cold plate below -60°C (PT100 limit / physically impossible)
 * - Cold plate above 80°C (should never happen with TECs)
 * - Hot plate below -20°C (impossible if TECs are on)
 * - Hot plate above 100°C (way past fault threshold)
 */
bool ThermalController::checkSensorSanity() {
    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();

    // Check cold plate sanity
    if (cold_temp < COLD_PLATE_MIN_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "FAULT: Cold=%.1fC impossible", cold_temp);
        enterThermalFault(buf);
        return false;
    }
    if (cold_temp > COLD_PLATE_MAX_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "FAULT: Cold=%.1fC too hot", cold_temp);
        enterThermalFault(buf);
        return false;
    }

    // Check hot plate sanity
    if (hot_temp < HOT_PLATE_MIN_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "FAULT: Hot=%.1fC impossible", hot_temp);
        enterThermalFault(buf);
        return false;
    }
    if (hot_temp > HOT_PLATE_MAX_VALID_C) {
        char buf[48];
        snprintf(buf, sizeof(buf), "FAULT: Hot=%.1fC extreme", hot_temp);
        enterThermalFault(buf);
        return false;
    }

    return true;
}

/**
 * @brief Physics-based PT100 plausibility check
 *
 * Detects PT100 failures that produce "valid" but physically impossible
 * readings.
 *
 * Key insight: If the PT100 shows extreme cold (e.g., -40°C) but the hot side
 * is only moderately warm (e.g., 30°C), the reading is implausible because:
 * - TEC cascade deltaT is roughly proportional to power input
 * - Extreme cold requires significant heat rejection on hot side
 * - A 70°C deltaT at low hot-side temp indicates sensor failure
 *
 * This catches:
 * - Shorted PT100 (reads artificially low resistance = cold)
 * - Miscalibrated MAX31865
 * - Wiring issues causing resistance drop
 *
 * @return true if PT100 readings are plausible, false if sensor likely failed
 */
bool ThermalController::checkPT100Plausibility() {
    // Skip check during startup - thermal gradient needs time to establish
    if (_state == ThermalState::STARTUP ||
        _state == ThermalState::INITIALIZING ||
        _state == ThermalState::SELF_TEST) {
        return true;
    }

    // Grace period after entering RAMP_UP
    if (_state == ThermalState::RAMP_UP) {
        unsigned long ramp_elapsed = millis() - _ramp_start_time;
        if (ramp_elapsed < PLAUSIBILITY_CHECK_GRACE_MS) {
            return true;
        }
    }

    float cold_temp = _cold_plate.getTemperature();
    float hot_temp = _hot_plate.getTemperature();
    float avg_current = (_target_current[0] + _target_current[1]) / 2.0f;

    // Only check if cold plate reports very low temperature
    if (cold_temp > PLAUSIBILITY_MAX_COLD_IMPLAUSIBLE_C) {
        return true; // Not in suspicious range
    }

    // If cold plate is extremely cold, hot side MUST be significantly warm
    // TEC heat pumping: Q_hot = Q_cold + P_electrical
    // At -40°C cold with 16V × 10A × 2 = 320W input, hot side should be 50°C+

    if (hot_temp < PLAUSIBILITY_HOT_THRESHOLD_FOR_CHECK_C) {
        // Cold plate claims extreme cold, but hot side is barely warm
        // This is physically impossible with active TECs
        char buf[64];
        snprintf(buf, sizeof(buf), "WARN: PT100 implausible! C=%.1f H=%.1f",
                 cold_temp, hot_temp);
        _logger.log(buf);

        // Check if current draw supports the cold temperature claim
        // At high current, we expect high deltaT
        float expected_min_delta =
            avg_current * PLAUSIBILITY_MIN_DELTA_T_PER_AMP;
        float actual_delta = hot_temp - cold_temp;

        if (actual_delta > expected_min_delta * 3) {
            // DeltaT is much higher than expected for this current level
            // Strong indicator of PT100 failure (reading too low)
            _logger.log("FAULT: PT100 likely failed low");
            enterThermalFault("PT100 implausible");
            return false;
        }
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

        // Verify shutdown commands succeeded
        bool shutdown_ok = true;
        if (!_psus[0].disableOutput()) {
            _logger.log("CRIT: PSU0 shutdown FAIL!");
            shutdown_ok = false;
        }
        if (!_psus[1].disableOutput()) {
            _logger.log("CRIT: PSU1 shutdown FAIL!");
            shutdown_ok = false;
        }

        if (!shutdown_ok) {
            // Modbus failed - try again, then log persistent failure
            // In production, this should trigger a hardware interlock
            delay(100); // Brief delay before retry
            _psus[0].disableOutput();
            _psus[1].disableOutput();
            _logger.log("CRIT: MODBUS DOWN!");
        }

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

/**
 * @brief Start non-blocking emergency shutdown
 *
 * IMPORTANT: This emergency shutdown still relies on Modbus communication
 * to the DPS5015 units. If Modbus is down, shutdown commands will not reach
 * the PSUs. For true fail-safe behavior, external hardware interlocks
 * (thermal cutoff switches, contactors, etc.) should be used in addition
 * to software control.
 *
 * The shutdown ramps down current rather than cutting immediately to avoid
 * thermal shock to TEC modules, but will disable outputs if comms fail.
 */
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

    // Check for channel imbalance (only when both channels are commanded
    // equally)
    checkChannelImbalance();
}

void ThermalController::checkChannelImbalance() {
    // Only check if both PSUs are connected and commanded to same current
    if (!_psus[0].isConnected() || !_psus[1].isConnected())
        return;

    float cmd_diff = fabs(_target_current[0] - _target_current[1]);
    if (cmd_diff > 0.1f)
        return; // Channels intentionally set differently

    // Rate limit imbalance logging
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
// NVS Metrics Persistence
// ============================================================================

// NVS namespace and keys (keep short to save space)
static constexpr const char *NVS_METRICS_NAMESPACE = "tc_metrics";
static constexpr const char *NVS_KEY_MIN_TEMP = "min_t";
static constexpr const char *NVS_KEY_OPT_CURRENT = "opt_i";
static constexpr const char *NVS_KEY_RUNTIME = "runtime";
static constexpr const char *NVS_KEY_SESSIONS = "sessions";

// NVS space management
// ESP32 NVS partition is typically 20KB-24KB
// Our usage: 4 keys × ~8 bytes each = ~32 bytes data + overhead
// We use a single namespace with fixed keys, so space usage is bounded
static constexpr size_t NVS_MIN_FREE_ENTRIES =
    10; // Minimum free entries to allow writes

/**
 * @brief Check if NVS has enough free space for writes
 *
 * ESP32 NVS uses a key-value store with entries. Each entry can hold
 * up to 1984 bytes. We check that there are enough free entries
 * before attempting writes.
 *
 * @return true if safe to write, false if NVS is nearly full
 */
bool ThermalController::checkNvsSpace() {
    nvs_stats_t nvs_stats;
    esp_err_t err = nvs_get_stats(NULL, &nvs_stats);

    if (err != ESP_OK) {
        // Can't get stats - allow writes but log warning
        _logger.log("NVS: Stats unavailable", true);
        return true;
    }

    size_t free_entries = nvs_stats.free_entries;

    if (free_entries < NVS_MIN_FREE_ENTRIES) {
        // Log warning (rate-limited)
        static unsigned long last_nvs_warning = 0;
        unsigned long now = millis();
        if (now - last_nvs_warning > 60000) { // Once per minute max
            char buf[48];
            snprintf(buf, sizeof(buf), "NVS: Low space! %d free",
                     (int)free_entries);
            _logger.log(buf);
            last_nvs_warning = now;
        }
        return false;
    }

    return true;
}

/**
 * @brief Load persisted metrics from NVS
 *
 * Called once during begin(). Loads:
 * - All-time minimum temperature achieved
 * - Optimal current that achieved that temperature
 * - Total accumulated runtime (seconds)
 * - Session count (number of boots)
 */
void ThermalController::loadMetricsFromNvs() {
    if (!_metrics_prefs.begin(NVS_METRICS_NAMESPACE, true)) { // Read-only
        _logger.log("TC: NVS init (first run)");
        _metrics_prefs.end();

        // First run - initialize with defaults
        _all_time_min_temp = 100.0f;
        _all_time_optimal_current = 0.0f;
        _total_runtime_seconds = 0;
        _session_count = 1;

        // Check space before writing
        if (!checkNvsSpace()) {
            _logger.log("NVS: No space for init!");
            return;
        }

        // Write initial values
        if (_metrics_prefs.begin(NVS_METRICS_NAMESPACE, false)) {
            bool ok = true;
            ok &= _metrics_prefs.putFloat(NVS_KEY_MIN_TEMP,
                                          _all_time_min_temp) > 0;
            ok &= _metrics_prefs.putFloat(NVS_KEY_OPT_CURRENT,
                                          _all_time_optimal_current) > 0;
            ok &= _metrics_prefs.putULong(NVS_KEY_RUNTIME,
                                          _total_runtime_seconds) > 0;
            ok &= _metrics_prefs.putULong(NVS_KEY_SESSIONS, _session_count) > 0;
            _metrics_prefs.end();

            if (!ok) {
                _logger.log("NVS: Init write failed!");
            }
        } else {
            _logger.log("NVS: Failed to open for init");
        }
        return;
    }

    // Load existing values
    _all_time_min_temp = _metrics_prefs.getFloat(NVS_KEY_MIN_TEMP, 100.0f);
    _all_time_optimal_current =
        _metrics_prefs.getFloat(NVS_KEY_OPT_CURRENT, 0.0f);
    _total_runtime_seconds = _metrics_prefs.getULong(NVS_KEY_RUNTIME, 0);
    _session_count = _metrics_prefs.getULong(NVS_KEY_SESSIONS, 0);
    _metrics_prefs.end();

    // Increment session count for this boot
    _session_count++;

    // Check space before writing session count
    if (!checkNvsSpace()) {
        _logger.log("NVS: No space for session++");
    } else if (_metrics_prefs.begin(NVS_METRICS_NAMESPACE, false)) {
        size_t written =
            _metrics_prefs.putULong(NVS_KEY_SESSIONS, _session_count);
        _metrics_prefs.end();
        if (written == 0) {
            _logger.log("NVS: Session write failed");
        }
    } else {
        _logger.log("NVS: Failed to open for session");
    }

    // Log loaded values
    char buf[64];
    snprintf(buf, sizeof(buf), "TC: Best=%.1fC@%.1fA", _all_time_min_temp,
             _all_time_optimal_current);
    _logger.log(buf);

    unsigned long hours = _total_runtime_seconds / 3600;
    unsigned long mins = (_total_runtime_seconds % 3600) / 60;
    snprintf(buf, sizeof(buf), "TC: Runtime=%luh%lum #%lu", hours, mins,
             _session_count);
    _logger.log(buf);
}

/**
 * @brief Save metrics to NVS periodically
 *
 * Only saves if enough time has passed since last save to avoid
 * wearing out flash. NVS has limited write cycles (~100,000).
 *
 * At 5-minute intervals, that's 20 years of continuous operation.
 *
 * @param force If true, save immediately regardless of interval
 */
void ThermalController::saveMetricsToNvs(bool force) {
    unsigned long now = millis();

    if (!force &&
        now - _last_metrics_save_time < NVS_METRICS_SAVE_INTERVAL_MS) {
        return;
    }

    // Check NVS space before writing
    if (!checkNvsSpace()) {
        // Already logged in checkNvsSpace
        return;
    }

    if (!_metrics_prefs.begin(NVS_METRICS_NAMESPACE, false)) {
        _logger.log("NVS: Failed to open for save", true);
        return;
    }

    // Only write values that have changed to minimize wear
    // NVS internally checks for changes, but we can skip the call entirely

    float stored_min = _metrics_prefs.getFloat(NVS_KEY_MIN_TEMP, 100.0f);
    if (fabs(_all_time_min_temp - stored_min) > 0.01f) {
        size_t w1 =
            _metrics_prefs.putFloat(NVS_KEY_MIN_TEMP, _all_time_min_temp);
        size_t w2 = _metrics_prefs.putFloat(NVS_KEY_OPT_CURRENT,
                                            _all_time_optimal_current);

        if (w1 == 0 || w2 == 0) {
            _logger.log("NVS: Metrics write failed!", true);
        } else if (force) {
            // Log when forced save succeeds (new record achieved)
            char buf[48];
            snprintf(buf, sizeof(buf), "NVS: Saved best=%.1fC",
                     _all_time_min_temp);
            _logger.log(buf, true);
        }
    }

    _metrics_prefs.end();
    _last_metrics_save_time = now;
}

/**
 * @brief Update runtime counter and save periodically
 *
 * Accumulates runtime in 1-minute increments to avoid excessive
 * NVS writes while still tracking operational time reasonably.
 */
void ThermalController::updateRuntimeCounter() {
    unsigned long now = millis();

    if (now - _last_runtime_save_time < NVS_RUNTIME_SAVE_INTERVAL_MS) {
        return;
    }

    // Calculate runtime since last save
    unsigned long elapsed_ms = now - _last_runtime_save_time;
    if (_last_runtime_save_time == 0) {
        // First call - use session start time
        elapsed_ms = now - _session_start_time;
    }

    _total_runtime_seconds += elapsed_ms / 1000;
    _last_runtime_save_time = now;

    // Check NVS space before writing
    if (!checkNvsSpace()) {
        // Already logged in checkNvsSpace
        return;
    }

    // Save to NVS
    if (_metrics_prefs.begin(NVS_METRICS_NAMESPACE, false)) {
        size_t written =
            _metrics_prefs.putULong(NVS_KEY_RUNTIME, _total_runtime_seconds);
        _metrics_prefs.end();

        if (written == 0) {
            _logger.log("NVS: Runtime write failed!", true);
        }
    } else {
        _logger.log("NVS: Failed to open for runtime", true);
    }
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
