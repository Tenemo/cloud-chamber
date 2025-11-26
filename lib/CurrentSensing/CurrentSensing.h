/**
 * @file CurrentSensing.h
 * @brief Dual ACS758 current sensor interface for monitoring current draw
 *
 * USAGE:
 * ------
 * 1. Create instance:
 *    CurrentSensing current_sensors(logger, "I1", "I2", "I_Total");
 *
 * 2. Initialize (calibration happens automatically):
 *    current_sensors.begin();
 *
 * 3. Update current readings in main loop:
 *    current_sensors.update();
 *
 * The sensor automatically registers display lines, performs background
 * calibration, and updates current displays when values change.
 */

#ifndef CURRENT_SENSING_H
#define CURRENT_SENSING_H

#include "Logger.h"
#include "config.h"
#include <Arduino.h>

class CurrentSensing {
  public:
    CurrentSensing(Logger &logger, const char *label1, const char *label2,
                   const char *labelTotal);

    void begin();
    void update();
    bool isReady() const { return _initialized && !_in_error_state; }

    float getSensor1Current() const { return _sensor1_filtered_current; }
    float getSensor2Current() const { return _sensor2_filtered_current; }
    float getTotalCurrent() const {
        return _sensor1_filtered_current + _sensor2_filtered_current;
    }

  private:
    bool calibrateSensors();
    bool calibrateSensor(int pin, float &offset_voltage);
    void readSensorCurrent(int pin, float offset_voltage,
                           float &filtered_current);
    float readAverageVoltage(int pin, int num_samples);
    void readCurrents(float &sensor1, float &sensor2, float &total);
    static float clampSmallCurrent(float current, float threshold = 0.1f);

    Logger &_logger;
    const char *_label1;
    const char *_label2;
    const char *_labelTotal;
    const char *_id1;
    const char *_id2;
    const char *_idTotal;

    float _sensor1_offset_voltage;
    float _sensor2_offset_voltage;
    float _sensor1_filtered_current;
    float _sensor2_filtered_current;
    bool _initialized;
    bool _in_error_state;
    int _adc_max_value;
    unsigned long _last_update_time;
    unsigned long _last_imbalance_warning_time;
};

#endif // CURRENT_SENSING_H
