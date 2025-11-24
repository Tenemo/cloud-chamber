#include "TECController.h"
#include "config.h"

TECController::TECController(Logger &logger)
    : _logger(logger), _state(STATE_INIT), _soft_start_begin_time(0),
      _pi_integral(0.0f), _pi_last_output(0.0f), _tec1_offset_voltage(0.0f),
      _tec2_offset_voltage(0.0f), _tec1_filtered_current(0.0f),
      _tec2_filtered_current(0.0f), _sensors_calibrated(false),
      _adc_max_value((1 << 12) - 1) {}

void TECController::begin() {
    // Configure enable pins
    pinMode(PIN_L_EN, OUTPUT);
    pinMode(PIN_R_EN, OUTPUT);
    digitalWrite(PIN_L_EN, LOW);
    digitalWrite(PIN_R_EN, LOW);

    // Configure PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_RPWM, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    // Configure ADC
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

    // Enable H-bridge
    digitalWrite(PIN_L_EN, HIGH);
    digitalWrite(PIN_R_EN, HIGH);

    // Start with detection duty
    setPwmDuty(DETECTION_DUTY);
    _state = STATE_WAITING_FOR_POWER;

    _logger.logControlActive();
}

void TECController::setPwmDuty(float duty) {
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
        _logger.logSoftStartBegin(TARGET_CURRENT_PER_TEC, MAX_DUTY);
        _state = STATE_SOFT_START;
        _soft_start_begin_time = millis();
        return true;
    }

    // Stay at detection duty
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
    // Check if any individual TEC exceeds its limit
    bool tec_limit_exceeded = (tec1_current > TARGET_CURRENT_PER_TEC) ||
                              (tec2_current > TARGET_CURRENT_PER_TEC);

    if (tec_limit_exceeded) {
        // Reset PI and force duty reduction
        resetPI();
        output_duty = _pi_last_output - 0.01f; // Aggressive reduction
        output_duty = constrain(output_duty, MIN_DUTY, MAX_DUTY);
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
    _pi_integral = constrain(_pi_integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    // Compute PI output
    float output = KP * error + KI * _pi_integral;

    // Add to previous output (incremental control)
    output = _pi_last_output + output;

    // Clamp output to valid range
    output = constrain(output, MIN_DUTY, MAX_DUTY);

    _pi_last_output = output;
    return output;
}

void TECController::resetPI() {
    _pi_integral = 0.0f;
    _pi_last_output = 0.0f;
}

bool TECController::checkOvercurrent(float total_current) {
    float threshold = TARGET_CURRENT_PER_TEC * 2.0f * OVERCURRENT_MULTIPLIER;
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

        float volts1 = _tec1_offset_voltage + (tec1 * ACS_SENS);
        float volts2 = _tec2_offset_voltage + (tec2 * ACS_SENS);

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

bool TECController::calibrateSensor(int pin, float &offset_voltage) {
    const int num_samples = 50;
    float voltage = readAverageVoltage(pin, num_samples);

    // Sanity check: offset should be roughly half of reference voltage
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

    // Apply exponential moving average filter
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

void TECController::update(unsigned long control_interval_ms,
                           unsigned long display_interval_ms) {
    unsigned long current_time = millis();

    // Read currents
    float tec1_current, tec2_current, total_current;
    readCurrents(tec1_current, tec2_current, total_current);

    float target_total_current = TARGET_CURRENT_PER_TEC * 2.0f;
    float current_duty = _pi_last_output;

    // State machine handling
    if (_state == STATE_WAITING_FOR_POWER) {
        bool power_detected =
            handleWaitingForPower(total_current, current_duty);

        // Update display and return if still waiting
        _logger.updateDisplay(tec1_current, tec2_current, current_duty, _state,
                              TARGET_CURRENT_PER_TEC, display_interval_ms);

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
                          TARGET_CURRENT_PER_TEC, display_interval_ms);

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
