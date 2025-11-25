/**
 * @file PT100.h
 * @brief PT100 temperature sensor interface using MAX31865 RTD-to-Digital
 * converter
 *
 * USAGE:
 * ------
 * 1. Create instance:
 *    PT100Sensor temp_sensor;
 *
 * 2. Initialize:
 *    temp_sensor.begin();
 *
 * 3. Update temperature in main loop:
 *    temp_sensor.update();
 *
 * The sensor automatically registers itself with the Logger and updates
 * the temperature display when values change.
 */

#ifndef PT100_H
#define PT100_H

#include "Logger.h"
#include <Adafruit_MAX31865.h>
#include <Arduino.h>

class PT100Sensor {
  public:
    PT100Sensor(Logger &logger);

    void begin();
    void update();
    float getTemperature() const { return _last_temperature; }

  private:
    Logger &_logger;
    Adafruit_MAX31865 *_rtd;
    float _last_temperature;
    bool _initialized;
    unsigned long _last_update_time;

    static constexpr float RNOMINAL = 100.0f; // PT100 nominal resistance
    static constexpr float RREF = 438.0f;     // Reference resistor value
};

#endif // PT100_H
