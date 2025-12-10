/**
 * @file PT100.h
 * @brief PT100 temperature sensor interface using MAX31865 RTD-to-Digital
 * converter
 *
 * USAGE:
 * ------
 * 1. Create instance:
 *    PT100Sensor pt100_sensor;
 *
 * 2. Initialize:
 *    pt100_sensor.begin();
 *
 * 3. Update temperature in main loop:
 *    pt100_sensor.update();
 *
 * The sensor automatically registers itself with the Logger and updates
 * the temperature display when values change.
 *
 * ERROR HANDLING:
 * ---------------
 * All PT100 error detection is centralized here. Callers should use
 * isInError() to check sensor health rather than interpreting raw temperature
 * values. Error conditions include:
 * - Temperature below PT100_ERROR_MIN_C or above PT100_ERROR_MAX_C
 * - MAX31865 fault flags (open wire, short, etc.)
 * - Hardware communication failures
 *
 * When isInError() returns true, getTemperature() returns the last valid
 * reading. The error state is logged and the display is updated to show
 * the error condition.
 */

#ifndef PT100_H
#define PT100_H

#include "Logger.h"
#include "config.h"
#include <Adafruit_MAX31865.h>
#include <Arduino.h>

class PT100Sensor {
  public:
    PT100Sensor(Logger &logger, const char *label);

    void begin();
    void update();
    float getTemperature() const { return _last_temperature; }
    bool isInError() const { return _in_error_state; }

  private:
    Logger &_logger;
    Adafruit_MAX31865 _rtd;
    const char *_label;
    float _last_temperature;
    bool _initialized;
    bool _in_error_state;
    unsigned long _last_update_time;
    unsigned long _last_serial_log_time;

    static constexpr float RNOMINAL = 100.0f; // PT100 nominal resistance
    static constexpr float RREF = 438.0f;     // Reference resistor value
};

#endif // PT100_H
