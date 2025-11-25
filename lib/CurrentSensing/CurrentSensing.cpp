#include "CurrentSensing.h"
#include "config.h"

CurrentSensing::CurrentSensing(Logger &logger)
    : _logger(logger), _sensor1_offset_voltage(0.0f),
      _sensor2_offset_voltage(0.0f), _sensor1_filtered_current(0.0f),
      _sensor2_filtered_current(0.0f), _sensors_calibrated(false),
      _adc_max_value((1 << 12) - 1), _last_update_time(0) {}

void CurrentSensing::begin() {
    analogReadResolution(12);

    // Register display lines immediately with "calibration" status
    _logger.registerTextLine("sensor1", "Sensor1:", "calibration");
    _logger.registerTextLine("sensor2", "Sensor2:", "calibration");
    _logger.registerTextLine("total", "Total:", "calibration");

    delay(500);

    // Perform calibration
    bool cal_success = calibrateSensors();

    if (cal_success) {
        // Switch to numeric display mode
        _logger.registerLine("sensor1", "Sensor1:", "A", 0.0f);
        _logger.registerLine("sensor2", "Sensor2:", "A", 0.0f);
        _logger.registerLine("total", "Total:", "A", 0.0f);
    } else {
        // Show calibration failure
        _logger.updateLineText("sensor1", "CAL FAIL");
        _logger.updateLineText("sensor2", "CAL FAIL");
        _logger.updateLineText("total", "CAL FAIL");
        Serial.println("FATAL: Sensor calibration failed!");
    }
}

bool CurrentSensing::calibrateSensors() {
    Serial.println("Calibrating current sensors...");

    bool cal1_success = calibrateSensor(PIN_ACS1, _sensor1_offset_voltage);
    bool cal2_success = calibrateSensor(PIN_ACS2, _sensor2_offset_voltage);

    if (!cal1_success || !cal2_success) {
        Serial.println("ERROR: Sensor calibration failed!");
        return false;
    }

    _sensor1_filtered_current = 0.0f;
    _sensor2_filtered_current = 0.0f;
    _sensors_calibrated = true;

    Serial.print("ACS1 zero offset: ");
    Serial.print(_sensor1_offset_voltage, 4);
    Serial.println(" V");
    Serial.print("ACS2 zero offset: ");
    Serial.print(_sensor2_offset_voltage, 4);
    Serial.println(" V");

    return true;
}

void CurrentSensing::update() {
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
    _logger.updateLine("sensor1", sensor1_current);
    _logger.updateLine("sensor2", sensor2_current);
    _logger.updateLine("total", total_current);

    // Serial logging (only when values change significantly)
    bool should_log = (abs(sensor1_current - prev_sensor1) > 0.01f ||
                       abs(sensor2_current - prev_sensor2) > 0.01f ||
                       abs(total_current - prev_total) > 0.01f);

    if (should_log) {
        Serial.print("Sensor1: ");
        Serial.print(sensor1_current, 2);
        Serial.print("A | Sensor2: ");
        Serial.print(sensor2_current, 2);
        Serial.print("A | Total: ");
        Serial.print(total_current, 2);
        Serial.println("A");

        prev_sensor1 = sensor1_current;
        prev_sensor2 = sensor2_current;
        prev_total = total_current;
    }

    float current_imbalance = abs(sensor1_current - sensor2_current);
    if (current_imbalance > 1.0f && total_current > 1.0f) {
        Serial.print("WARNING: Branch current imbalance detected: ");
        Serial.print(current_imbalance, 2);
        Serial.println("A");
    }
}

void CurrentSensing::readCurrents(float &sensor1, float &sensor2,
                                  float &total) {
    if (!_sensors_calibrated) {
        sensor1 = 0.0f;
        sensor2 = 0.0f;
        total = 0.0f;
        return;
    }

    float I1_raw = readSensorCurrent(PIN_ACS1, _sensor1_offset_voltage,
                                     _sensor1_filtered_current);
    float I2_raw = readSensorCurrent(PIN_ACS2, _sensor2_offset_voltage,
                                     _sensor2_filtered_current);

    sensor1 = clampSmallCurrent(I1_raw);
    sensor2 = clampSmallCurrent(I2_raw);
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
