// Contents of: "lib/TECController/TECController.cpp":
#include "TECController.h"
#include "config.h"

TECController::TECController(Logger &logger)
    : _logger(logger), _state(STATE_INIT), _soft_start_begin_time(0),
      _pi_integral(0.0f), _pi_last_output(0.0f), _tec1_offset_voltage(0.0f),
      _tec2_offset_voltage(0.0f), _tec1_filtered_current(0.0f),
      _tec2_filtered_current(0.0f), _sensors_calibrated(false),
      _adc_max_value((1 << 12) - 1) {}

void TECController::begin() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    pinMode(PIN_L_EN, OUTPUT);
    pinMode(PIN_R_EN, OUTPUT);
    digitalWrite(PIN_L_EN, LOW);
    digitalWrite(PIN_R_EN, LOW);

    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_RPWM, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    analogReadResolution(12);
}

bool TECController::calibrateSensors() {
    _state = STATE_CALIBRATING;
    _logger.logCalibrationStart();

    bool cal1_success = calibrateSensor(PIN_ACS1, _tec1_offset_voltage);
    bool cal2_success = calibrateSensor(PIN_ACS2, _tec2_offset_voltage);

    if (!cal1_success || !cal2_success) {
        Serial.println("ERROR: Sensor calibration failed!");
        return false;
    }

    _tec1_filtered_current = 0.0f;
    _tec2_filtered_current = 0.0f;
    _sensors_calibrated = true;

    _logger.logCalibrationResults(_tec1_offset_voltage, _tec2_offset_voltage);
    return true;
}

void TECController::startPowerDetection() {
    _logger.logWaitingForPower();

    digitalWrite(STATUS_LED_PIN, HIGH);
    if (PWM_ENABLED) {
        digitalWrite(PIN_L_EN, HIGH);
        digitalWrite(PIN_R_EN, HIGH);
        setPwmDuty(DETECTION_DUTY);
        _state = STATE_WAITING_FOR_POWER;
        _logger.logControlActive();
    } else {
        // PWM disabled: keep bridge off, but still allow current
        // measurements and status display; remain in INIT state.
        setPwmDuty(0.0f);
        _state = STATE_INIT;
    }
}

void TECController::setPwmDuty(float duty) {
    if (!PWM_ENABLED) {
        duty = 0.0f;
    }
    duty = constrain(duty, MIN_DUTY, MAX_DUTY);
    uint32_t maxVal = (1u << PWM_RES_BITS) - 1u;
    uint32_t val = uint32_t(duty * maxVal + 0.5f);
    ledcWrite(PWM_CHANNEL, val);
}

void TECController::readCurrents(float &tec1, float &tec2, float &total) {
    if (!_sensors_calibrated) {
        tec1 = 0.0f;
        tec2 = 0.0f;
        total = 0.0f;
        return;
    }

    float I1_raw = readSensorCurrent(PIN_ACS1, _tec1_offset_voltage,
                                     _tec1_filtered_current);
    float I2_raw = readSensorCurrent(PIN_ACS2, _tec2_offset_voltage,
                                     _tec2_filtered_current);

    tec1 = clampSmallCurrent(I1_raw);
    tec2 = clampSmallCurrent(I2_raw);
    total = tec1 + tec2;
}

bool TECController::handleWaitingForPower(float total_current,
                                          float &current_duty) {
    if (total_current >= DETECTION_THRESHOLD) {
        _logger.logPowerDetected(total_current);
        _logger.logSoftStartBegin();
        _state = STATE_SOFT_START;
        _soft_start_begin_time = millis();
        return true;
    }

    setPwmDuty(DETECTION_DUTY);
    current_duty = DETECTION_DUTY;
    return false;
}

void TECController::handleSoftStart(unsigned long current_time,
                                    float &target_current) {
    unsigned long elapsed = current_time - _soft_start_begin_time;

    if (elapsed < SOFT_START_DURATION_MS) {
        float ramp_fraction = (float)elapsed / SOFT_START_DURATION_MS;
        target_current *= ramp_fraction;
    } else {
        _state = STATE_RUNNING;
        _logger.logSoftStartComplete();
    }
}

void TECController::computeControl(float target_current, float total_current,
                                   float tec1_current, float tec2_current,
                                   float dt, float &output_duty) {
    // Check for dangerous overcurrent on individual branches
    float per_tec_limit = TARGET_CURRENT_PER_TEC * PER_TEC_LIMIT_MULTIPLIER;
    bool tec_overshoot =
        (tec1_current > per_tec_limit) || (tec2_current > per_tec_limit);

    float error = target_current - total_current;

    if (tec_overshoot) {
        // One TEC is drawing too much - prevent further integral windup,
        // but do not hard-reset it (that causes oscillation).
        if (_pi_integral > 0.0f) {
            _pi_integral -= KI * 0.5f * dt; // Slowly bleed off integral
            if (_pi_integral < 0.0f)
                _pi_integral = 0.0f;
        }

        // Use proportional action to gently back off
        float p_term = KP * error;
        output_duty = _pi_last_output + p_term * dt;
    } else {
        // Normal operation - full PI control
        output_duty = computePI(error, dt);
    }

    output_duty = constrain(output_duty, MIN_DUTY, MAX_DUTY);
    _pi_last_output = output_duty;
    setPwmDuty(output_duty);
}

float TECController::computePI(float error, float dt) {
    // Accumulate integral
    _pi_integral += error * dt;
    _pi_integral = constrain(_pi_integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    // Calculate PI output components
    float p_term = KP * error;
    float i_term = KI * _pi_integral;

    float delta = p_term + i_term;

    // Allow up to 5% duty change per update at 20 Hz
    const float max_step = 0.05f;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }

    float output = _pi_last_output + delta;
    return output;
}

void TECController::resetPI() {
    _pi_integral = 0.0f;
    _pi_last_output = 0.0f;
}

bool TECController::checkOvercurrent(float total_current) {
    // Only trigger emergency shutdown on severe total overcurrent
    float threshold =
        TARGET_CURRENT_PER_TEC * 2.0f * TOTAL_OVERCURRENT_MULTIPLIER;
    return (total_current > threshold);
}

void TECController::handleOvercurrentError() {
    _logger.logErrorOvercurrent();
    _state = STATE_ERROR;
    // Drop duty by 5% immediately on overcurrent, but do not fully
    // power-cycle the bridge here.
    float reduced = _pi_last_output - 0.05f;
    if (reduced < MIN_DUTY) {
        reduced = MIN_DUTY;
    }
    _pi_last_output = reduced;
    setPwmDuty(_pi_last_output);
}

bool TECController::calibrateSensor(int pin, float &offset_voltage) {
    const int num_samples = 50;
    float voltage = readAverageVoltage(pin, num_samples);

    // Sanity check: ACS758 outputs ~Vcc/2 at zero current
    if (voltage < 0.1f || voltage > (ADC_REF_V - 0.1f)) {
        return false;
    }

    offset_voltage = voltage;
    return true;
}

float TECController::readSensorCurrent(int pin, float offset_voltage,
                                       float &filtered_current) {
    float voltage = readAverageVoltage(pin, ADC_SAMPLES);
    float delta_voltage = voltage - offset_voltage;
    float raw_current = delta_voltage / ACS_SENS;

    filtered_current =
        FILTER_ALPHA * raw_current + (1.0f - FILTER_ALPHA) * filtered_current;

    return filtered_current;
}

float TECController::readAverageVoltage(int pin, int num_samples) {
    long raw_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        raw_sum += analogRead(pin);
    }

    float raw_avg = (float)raw_sum / num_samples;
    float voltage = raw_avg * (ADC_REF_V / _adc_max_value);

    return voltage;
}

float TECController::clampSmallCurrent(float current, float threshold) {
    return (abs(current) < threshold) ? 0.0f : current;
}

void TECController::getCalibrationOffsets(float &offset1,
                                          float &offset2) const {
    offset1 = _tec1_offset_voltage;
    offset2 = _tec2_offset_voltage;
}

void TECController::emergencyShutdown() {
    setPwmDuty(0.0f);
    digitalWrite(PIN_L_EN, LOW);
    digitalWrite(PIN_R_EN, LOW);
}

void TECController::update() {
    static unsigned long last_time = millis();
    unsigned long current_time = millis();

    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0.0f) {
        dt = CONTROL_INTERVAL_MS / 1000.0f;
    }
    last_time = current_time;

    float tec1_current, tec2_current, total_current;
    readCurrents(tec1_current, tec2_current, total_current);

    float target_total_current = TARGET_CURRENT_PER_TEC * 2.0f;
    float current_duty = _pi_last_output;

    if (PWM_ENABLED) {
        if (_state == STATE_WAITING_FOR_POWER) {
            bool power_detected =
                handleWaitingForPower(total_current, current_duty);

            _logger.updateDisplay(tec1_current, tec2_current, current_duty,
                                  _state);

            if (!power_detected) {
                return;
            }
        }

        if (_state == STATE_SOFT_START) {
            handleSoftStart(current_time, target_total_current);
        }

        computeControl(target_total_current, total_current, tec1_current,
                       tec2_current, dt, current_duty);
    } else {
        // PWM disabled: ensure duty is always zero and skip control loop.
        current_duty = 0.0f;
    }

    _logger.updateDisplay(tec1_current, tec2_current, current_duty, _state);

    float error = target_total_current - total_current;
    CurrentMeasurements measurements = {tec1_current, tec2_current,
                                        total_current, current_duty, error};

    if (_logger.shouldLogMeasurements(measurements)) {
        _logger.logMeasurements(target_total_current, total_current,
                                tec1_current, tec2_current, current_duty,
                                error);
    }

    float current_imbalance = abs(tec1_current - tec2_current);
    if (current_imbalance > 1.0f && total_current > 1.0f) {
        _logger.logWarningImbalance(current_imbalance);
    }

    if (PWM_ENABLED && checkOvercurrent(total_current)) {
        handleOvercurrentError();
    }
}
