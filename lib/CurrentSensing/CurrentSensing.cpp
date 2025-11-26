#include "CurrentSensing.h"
#include "config.h"

CurrentSensing::CurrentSensing(Logger &logger, const char *label1,
                               const char *label2, const char *labelTotal)
    : _logger(logger), _label1(label1), _label2(label2),
      _labelTotal(labelTotal), _id1(label1), _id2(label2), _idTotal(labelTotal),
      _sensor1_offset_voltage(0.0f), _sensor2_offset_voltage(0.0f),
      _sensor1_filtered_current(0.0f), _sensor2_filtered_current(0.0f),
      _initialized(false), _in_error_state(false),
      _adc_max_value((1 << 12) - 1), _last_update_time(0),
      _last_imbalance_warning_time(0) {}

void CurrentSensing::begin() {
    if (_initialized)
        return; // prevent re-initialization

    analogReadResolution(12);

    // Register display lines immediately with "calibrating" status
    char labelBuf[16];
    snprintf(labelBuf, sizeof(labelBuf), "%s:", _label1);
    _logger.registerTextLine(_id1, labelBuf, "calibrating");
    snprintf(labelBuf, sizeof(labelBuf), "%s:", _label2);
    _logger.registerTextLine(_id2, labelBuf, "calibrating");
    snprintf(labelBuf, sizeof(labelBuf), "%s:", _labelTotal);
    _logger.registerTextLine(_idTotal, labelBuf, "calibrating");

    delay(500);

    // Perform calibration
    bool cal_success = calibrateSensors();

    if (cal_success) {
        // Switch to numeric display mode
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _label1);
        _logger.registerLine(_id1, labelBuf, "A", 0.0f);
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _label2);
        _logger.registerLine(_id2, labelBuf, "A", 0.0f);
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _labelTotal);
        _logger.registerLine(_idTotal, labelBuf, "A", 0.0f);
        _logger.log("ACS758s initialized.");
    } else {
        _in_error_state = true;
        _logger.updateLineText(_id1, "ERROR");
        _logger.updateLineText(_id2, "ERROR");
        _logger.updateLineText(_idTotal, "ERROR");
        _logger.log("ACS758 calibration failed");
    }

    _initialized = true;
}

bool CurrentSensing::calibrateSensors() {
    _logger.log("Calibrating...");

    bool cal1_success = calibrateSensor(PIN_ACS1, _sensor1_offset_voltage);
    bool cal2_success = calibrateSensor(PIN_ACS2, _sensor2_offset_voltage);

    if (!cal1_success || !cal2_success) {
        _logger.log("Calibration failed!");
        return false;
    }

    _sensor1_filtered_current = 0.0f;
    _sensor2_filtered_current = 0.0f;

    char buf[64];
    snprintf(buf, sizeof(buf), "ACS1: %.4fV", _sensor1_offset_voltage);
    _logger.log(buf);
    snprintf(buf, sizeof(buf), "ACS2: %.4fV", _sensor2_offset_voltage);
    _logger.log(buf);

    return true;
}

void CurrentSensing::update() {
    if (!_initialized || _in_error_state)
        return;

    unsigned long current_time = millis();
    if (current_time - _last_update_time < SENSOR_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_time = current_time;

    static float prev_sensor1 = -999.0f;
    static float prev_sensor2 = -999.0f;
    static float prev_total = -999.0f;

    float sensor1_current, sensor2_current, total_current;
    readCurrents(sensor1_current, sensor2_current, total_current);

    // Update display lines
    _logger.updateLine(_id1, sensor1_current);
    _logger.updateLine(_id2, sensor2_current);
    _logger.updateLine(_idTotal, total_current);

    // Log only significant changes (serial only, not display)
    bool should_log = (abs(sensor1_current - prev_sensor1) > 0.01f ||
                       abs(sensor2_current - prev_sensor2) > 0.01f ||
                       abs(total_current - prev_total) > 0.01f);

    if (should_log) {
        char buf[64];
        snprintf(buf, sizeof(buf), "S1:%.1fA S2:%.1fA T:%.1fA", sensor1_current,
                 sensor2_current, total_current);
        _logger.log(buf, true); // serial only, no display

        prev_sensor1 = sensor1_current;
        prev_sensor2 = sensor2_current;
        prev_total = total_current;
    }

    // Imbalance warning (throttled to once per second)
    if (ENABLE_IMBALANCE_WARNINGS) {
        float current_imbalance = abs(sensor1_current - sensor2_current);
        if (current_imbalance > 1.0f && total_current > 1.0f) {
            if (current_time - _last_imbalance_warning_time >= 1000) {
                _last_imbalance_warning_time = current_time;
                char buf[64];
                snprintf(buf, sizeof(buf), "WARN: Imbalance %.1fA",
                         current_imbalance);
                _logger.log(buf);
            }
        }
    }
}

void CurrentSensing::readCurrents(float &sensor1, float &sensor2,
                                  float &total) {
    if (!_initialized || _in_error_state) {
        sensor1 = 0.0f;
        sensor2 = 0.0f;
        total = 0.0f;
        return;
    }

    readSensorCurrent(PIN_ACS1, _sensor1_offset_voltage,
                      _sensor1_filtered_current);
    readSensorCurrent(PIN_ACS2, _sensor2_offset_voltage,
                      _sensor2_filtered_current);

    sensor1 = clampSmallCurrent(_sensor1_filtered_current);
    sensor2 = clampSmallCurrent(_sensor2_filtered_current);
    total = sensor1 + sensor2;
}

bool CurrentSensing::calibrateSensor(int pin, float &offset_voltage) {
    const int num_samples = 50;
    float voltage = readAverageVoltage(pin, num_samples);

    // Sanity check: ACS758 outputs ~Vcc/2 at zero current
    if (voltage < 0.1f || voltage > (ADC_REF_V - 0.1f)) {
        return false;
    }

    offset_voltage = voltage;
    return true;
}

void CurrentSensing::readSensorCurrent(int pin, float offset_voltage,
                                       float &filtered_current) {
    float voltage = readAverageVoltage(pin, ADC_SAMPLES);
    float delta_voltage = voltage - offset_voltage;
    float raw_current = delta_voltage / ACS_SENS;

    filtered_current =
        FILTER_ALPHA * raw_current + (1.0f - FILTER_ALPHA) * filtered_current;
}

float CurrentSensing::readAverageVoltage(int pin, int num_samples) {
    long raw_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        raw_sum += analogRead(pin);
    }

    float raw_avg = (float)raw_sum / num_samples;
    float voltage = raw_avg * (ADC_REF_V / _adc_max_value);

    return voltage;
}

float CurrentSensing::clampSmallCurrent(float current, float threshold) {
    return (abs(current) < threshold) ? 0.0f : current;
}
