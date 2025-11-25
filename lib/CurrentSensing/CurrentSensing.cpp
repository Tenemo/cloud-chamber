#include "CurrentSensing.h"
#include "config.h"

CurrentSensing::CurrentSensing(Logger &logger)
    : _logger(logger), _tec1_offset_voltage(0.0f), _tec2_offset_voltage(0.0f),
      _tec1_filtered_current(0.0f), _tec2_filtered_current(0.0f),
      _sensors_calibrated(false), _adc_max_value((1 << 12) - 1) {}

void CurrentSensing::begin() { analogReadResolution(12); }

bool CurrentSensing::calibrateSensors() {
    Serial.println("Calibrating current sensors...");

    bool cal1_success = calibrateSensor(PIN_ACS1, _tec1_offset_voltage);
    bool cal2_success = calibrateSensor(PIN_ACS2, _tec2_offset_voltage);

    if (!cal1_success || !cal2_success) {
        Serial.println("ERROR: Sensor calibration failed!");
        return false;
    }

    _tec1_filtered_current = 0.0f;
    _tec2_filtered_current = 0.0f;
    _sensors_calibrated = true;

    Serial.print("ACS1 zero offset: ");
    Serial.print(_tec1_offset_voltage, 4);
    Serial.println(" V");
    Serial.print("ACS2 zero offset: ");
    Serial.print(_tec2_offset_voltage, 4);
    Serial.println(" V");

    return true;
}

void CurrentSensing::update() {
    static float prev_tec1 = -999.0f;
    static float prev_tec2 = -999.0f;
    static float prev_total = -999.0f;

    float tec1_current, tec2_current, total_current;
    readCurrents(tec1_current, tec2_current, total_current);

    // Update display lines
    _logger.updateLine("tec1", tec1_current);
    _logger.updateLine("tec2", tec2_current);
    _logger.updateLine("total", total_current);

    // Serial logging (only when values change significantly)
    bool should_log = (abs(tec1_current - prev_tec1) > 0.01f ||
                       abs(tec2_current - prev_tec2) > 0.01f ||
                       abs(total_current - prev_total) > 0.01f);

    if (should_log) {
        Serial.print("TEC1: ");
        Serial.print(tec1_current, 2);
        Serial.print("A | TEC2: ");
        Serial.print(tec2_current, 2);
        Serial.print("A | Total: ");
        Serial.print(total_current, 2);
        Serial.println("A");

        prev_tec1 = tec1_current;
        prev_tec2 = tec2_current;
        prev_total = total_current;
    }

    float current_imbalance = abs(tec1_current - tec2_current);
    if (current_imbalance > 1.0f && total_current > 1.0f) {
        Serial.print("WARNING: Branch current imbalance detected: ");
        Serial.print(current_imbalance, 2);
        Serial.println("A");
    }
}

void CurrentSensing::readCurrents(float &tec1, float &tec2, float &total) {
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

float CurrentSensing::readSensorCurrent(int pin, float offset_voltage,
                                        float &filtered_current) {
    float voltage = readAverageVoltage(pin, ADC_SAMPLES);
    float delta_voltage = voltage - offset_voltage;
    float raw_current = delta_voltage / ACS_SENS;

    filtered_current =
        FILTER_ALPHA * raw_current + (1.0f - FILTER_ALPHA) * filtered_current;

    return filtered_current;
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

void CurrentSensing::getCalibrationOffsets(float &offset1,
                                           float &offset2) const {
    offset1 = _tec1_offset_voltage;
    offset2 = _tec2_offset_voltage;
}
