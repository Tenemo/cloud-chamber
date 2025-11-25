/**
 * @file CurrentSensing.h
 * @brief Dual ACS758 current sensor interface for monitoring TEC current draw
 *
 * USAGE:
 * ------
 * 1. Create instance:
 *    CurrentSensing current_sensors(logger);
 *
 * 2. Initialize and calibrate:
 *    current_sensors.begin();
 *    bool success = current_sensors.calibrateSensors();
 *
 * 3. Update current readings in main loop:
 *    current_sensors.update();
 *
 * The sensor automatically registers itself with the Logger and updates
 * current displays when values change.
 */

#ifndef CURRENT_SENSING_H
#define CURRENT_SENSING_H

#include "Logger.h"
#include "config.h"
#include <Arduino.h>

class CurrentSensing {
  public:
    CurrentSensing(Logger &logger);

    void begin();
    bool calibrateSensors();
    void update();

    void getCalibrationOffsets(float &offset1, float &offset2) const;
    float getTEC1Current() const { return _tec1_filtered_current; }
    float getTEC2Current() const { return _tec2_filtered_current; }
    float getTotalCurrent() const {
        return _tec1_filtered_current + _tec2_filtered_current;
    }

  private:
    bool calibrateSensor(int pin, float &offset_voltage);
    float readSensorCurrent(int pin, float offset_voltage,
                            float &filtered_current);
    float readAverageVoltage(int pin, int num_samples);
    void readCurrents(float &tec1, float &tec2, float &total);
    static float clampSmallCurrent(float current, float threshold = 0.1f);

    Logger &_logger;

    float _tec1_offset_voltage;
    float _tec2_offset_voltage;
    float _tec1_filtered_current;
    float _tec2_filtered_current;
    bool _sensors_calibrated;
    int _adc_max_value;
};

#endif // CURRENT_SENSING_H
