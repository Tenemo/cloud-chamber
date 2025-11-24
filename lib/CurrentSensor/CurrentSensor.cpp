#include "CurrentSensor.h"

CurrentSensor::CurrentSensor(int pin, float adc_ref_voltage, int adc_resolution,
                             float sensitivity, float filter_alpha)
    : _pin(pin), _adc_ref_voltage(adc_ref_voltage),
      _adc_max_value((1 << adc_resolution) - 1), _sensitivity(sensitivity),
      _filter_alpha(filter_alpha), _offset_voltage(0.0f),
      _filtered_current(0.0f), _calibrated(false) {}

bool CurrentSensor::calibrate(int num_samples) {
    if (num_samples <= 0) {
        return false;
    }

    float voltage = readAverageVoltage(num_samples);

    // Sanity check: offset should be roughly half of reference voltage
    if (voltage < 0.1f || voltage > (_adc_ref_voltage - 0.1f)) {
        return false;
    }

    _offset_voltage = voltage;
    _filtered_current = 0.0f;
    _calibrated = true;

    return true;
}

float CurrentSensor::readCurrent(int num_samples) {
    if (!_calibrated) {
        return 0.0f;
    }

    float voltage = readAverageVoltage(num_samples);
    float delta_voltage = voltage - _offset_voltage;
    float raw_current = delta_voltage / _sensitivity;

    // Apply exponential moving average filter
    _filtered_current = _filter_alpha * raw_current +
                        (1.0f - _filter_alpha) * _filtered_current;

    return _filtered_current;
}

float CurrentSensor::getFilteredCurrent() const { return _filtered_current; }

float CurrentSensor::getOffsetVoltage() const { return _offset_voltage; }

bool CurrentSensor::isCalibrated() const { return _calibrated; }

float CurrentSensor::clampSmallCurrent(float current, float threshold) {
    return (abs(current) < threshold) ? 0.0f : current;
}

void CurrentSensor::resetFilter() { _filtered_current = 0.0f; }

float CurrentSensor::readAverageVoltage(int num_samples) {
    long raw_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        raw_sum += analogRead(_pin);
    }

    float raw_avg = (float)raw_sum / num_samples;
    float voltage = raw_avg * (_adc_ref_voltage / _adc_max_value);

    return voltage;
}
