#include "TECController.h"
#include "Display.h"

TECController::TECController(const Config &config, Logger &logger)
    : _config(config),
      _tec1_sensor(config.pin_acs1, config.adc_ref_v, 12,
                   config.acs_sensitivity, config.filter_alpha),
      _tec2_sensor(config.pin_acs2, config.adc_ref_v, 12,
                   config.acs_sensitivity, config.filter_alpha),
      _logger(logger), _state(STATE_INIT), _soft_start_begin_time(0),
      _pi_integral(0.0f), _pi_last_output(0.0f) {}

void TECController::begin() {
    // Configure enable pins
    pinMode(_config.pin_l_en, OUTPUT);
    pinMode(_config.pin_r_en, OUTPUT);
    digitalWrite(_config.pin_l_en, LOW);
    digitalWrite(_config.pin_r_en, LOW);

    // Configure PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(_config.pin_rpwm, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    // Configure ADC
    analogReadResolution(12);
}

bool TECController::calibrateSensors() {
    _state = STATE_CALIBRATING;
    _logger.logCalibrationStart();

    bool cal1_success = _tec1_sensor.calibrate(50);
    bool cal2_success = _tec2_sensor.calibrate(50);

    if (!cal1_success || !cal2_success) {
        Serial.println("ERROR: Sensor calibration failed!");
        return false;
    }

    _logger.logCalibrationResults(_tec1_sensor.getOffsetVoltage(),
                                  _tec2_sensor.getOffsetVoltage());
    return true;
}

void TECController::startPowerDetection() {
    _logger.logWaitingForPower();

    // Enable H-bridge
    digitalWrite(_config.pin_l_en, HIGH);
    digitalWrite(_config.pin_r_en, HIGH);

    // Start with detection duty
    setPwmDuty(_config.detection_duty);
    _state = STATE_WAITING_FOR_POWER;

    _logger.logControlActive();
}

void TECController::setPwmDuty(float duty) {
    duty = constrain(duty, _config.min_duty, _config.max_duty);
    uint32_t maxVal = (1u << PWM_RES_BITS) - 1u;
    uint32_t val = uint32_t(duty * maxVal + 0.5f);
    ledcWrite(PWM_CHANNEL, val);
}

void TECController::readCurrents(float &tec1, float &tec2, float &total) {
    float I1_raw = _tec1_sensor.readCurrent(_config.adc_samples);
    float I2_raw = _tec2_sensor.readCurrent(_config.adc_samples);

    tec1 = CurrentSensor::clampSmallCurrent(I1_raw);
    tec2 = CurrentSensor::clampSmallCurrent(I2_raw);
    total = tec1 + tec2;
}

bool TECController::handleWaitingForPower(float total_current,
                                          float &current_duty) {
    if (total_current >= _config.detection_threshold) {
        _logger.logPowerDetected(total_current);
        _logger.logSoftStartBegin(_config.target_current_per_tec,
                                  _config.max_duty);
        _state = STATE_SOFT_START;
        _soft_start_begin_time = millis();
        return true;
    }

    // Stay at detection duty
    setPwmDuty(_config.detection_duty);
    current_duty = _config.detection_duty;
    return false;
}

void TECController::handleSoftStart(unsigned long current_time,
                                    float &target_current) {
    unsigned long elapsed = current_time - _soft_start_begin_time;

    if (elapsed < _config.soft_start_duration_ms) {
        float ramp_fraction = (float)elapsed / _config.soft_start_duration_ms;
        target_current *= ramp_fraction;
    } else {
        _state = STATE_RUNNING;
        _logger.logSoftStartComplete();
    }
}

void TECController::computeControl(float target_current, float total_current,
                                   float tec1_current, float tec2_current,
                                   float dt, float &output_duty) {
    // Check if any individual TEC exceeds its limit
    bool tec_limit_exceeded = (tec1_current > _config.target_current_per_tec) ||
                              (tec2_current > _config.target_current_per_tec);

    if (tec_limit_exceeded) {
        // Reset PI and force duty reduction
        resetPI();
        output_duty = _pi_last_output - 0.01f; // Aggressive reduction
        output_duty =
            constrain(output_duty, _config.min_duty, _config.max_duty);
        _pi_last_output = output_duty;
    } else {
        // Normal PI control
        float error = target_current - total_current;
        output_duty = computePI(error, dt);
    }

    setPwmDuty(output_duty);
}

float TECController::computePI(float error, float dt) {
    // Accumulate integral with time weighting
    _pi_integral += error * dt;

    // Apply anti-windup by clamping integral term
    _pi_integral =
        constrain(_pi_integral, -_config.integral_max, _config.integral_max);

    // Compute PI output
    float output = _config.kp * error + _config.ki * _pi_integral;

    // Add to previous output (incremental control)
    output = _pi_last_output + output;

    // Clamp output to valid range
    output = constrain(output, _config.min_duty, _config.max_duty);

    _pi_last_output = output;
    return output;
}

void TECController::resetPI() {
    _pi_integral = 0.0f;
    _pi_last_output = 0.0f;
}

bool TECController::checkOvercurrent(float total_current) {
    float threshold =
        _config.target_current_per_tec * 2.0f * _config.overcurrent_multiplier;
    return (total_current > threshold);
}

void TECController::handleOvercurrentError() {
    _logger.logErrorOvercurrent();
    _state = STATE_ERROR;

    emergencyShutdown();

    // Continuous error logging loop
    while (true) {
        float tec1, tec2, total;
        readCurrents(tec1, tec2, total);

        float volts1 =
            _tec1_sensor.getOffsetVoltage() + (tec1 * _config.acs_sensitivity);
        float volts2 =
            _tec2_sensor.getOffsetVoltage() + (tec2 * _config.acs_sensitivity);

        Serial.print("OVERCURRENT - TEC1: ");
        Serial.print(tec1, 2);
        Serial.print("A (");
        Serial.print(volts1, 3);
        Serial.print("V) | TEC2: ");
        Serial.print(tec2, 2);
        Serial.print("A (");
        Serial.print(volts2, 3);
        Serial.print("V) | Total: ");
        Serial.print(total, 2);
        Serial.println("A");

        delay(1000);
    }
}

void TECController::getCalibrationOffsets(float &offset1,
                                          float &offset2) const {
    offset1 = _tec1_sensor.getOffsetVoltage();
    offset2 = _tec2_sensor.getOffsetVoltage();
}

void TECController::emergencyShutdown() {
    setPwmDuty(0.0f);
    digitalWrite(_config.pin_l_en, LOW);
    digitalWrite(_config.pin_r_en, LOW);
}

void TECController::update(unsigned long control_interval_ms,
                           unsigned long display_interval_ms) {
    unsigned long current_time = millis();

    // Read currents
    float tec1_current, tec2_current, total_current;
    readCurrents(tec1_current, tec2_current, total_current);

    float target_total_current = _config.target_current_per_tec * 2.0f;
    float current_duty = _pi_last_output;

    // State machine handling
    if (_state == STATE_WAITING_FOR_POWER) {
        bool power_detected =
            handleWaitingForPower(total_current, current_duty);

        // Update display and return if still waiting
        _logger.updateDisplay(tec1_current, tec2_current, current_duty, _state,
                              _config.target_current_per_tec,
                              display_interval_ms);

        if (!power_detected) {
            return;
        }
    }

    if (_state == STATE_SOFT_START) {
        handleSoftStart(current_time, target_total_current);
    }

    // Compute control output
    float dt = control_interval_ms / 1000.0f;
    computeControl(target_total_current, total_current, tec1_current,
                   tec2_current, dt, current_duty);

    // Update display
    _logger.updateDisplay(tec1_current, tec2_current, current_duty, _state,
                          _config.target_current_per_tec, display_interval_ms);

    // Log measurements if significant change
    float error = target_total_current - total_current;
    CurrentMeasurements measurements = {tec1_current, tec2_current,
                                        total_current, current_duty, error};

    if (_logger.shouldLogMeasurements(measurements)) {
        _logger.logMeasurements(target_total_current, total_current,
                                tec1_current, tec2_current, current_duty,
                                error);
    }

    // Check for imbalance
    float current_imbalance = abs(tec1_current - tec2_current);
    if (current_imbalance > 1.0f && total_current > 1.0f) {
        _logger.logWarningImbalance(current_imbalance);
    }

    // Check for overcurrent
    if (checkOvercurrent(total_current)) {
        handleOvercurrentError();
    }
}
