/**
 * @file DS18B20.h
 * @brief DS18B20 digital temperature sensor interface using OneWire protocol
 *
 * USAGE:
 * ------
 * 1. Create instance:
 *    DS18B20Sensor ds_sensor(logger);
 *
 * 2. Initialize:
 *    ds_sensor.begin();
 *
 * 3. Update temperature in main loop:
 *    ds_sensor.update();
 *
 * The sensor automatically registers itself with the Logger and updates
 * the temperature display when values change.
 */

#ifndef DS18B20_H
#define DS18B20_H

#include "Logger.h"
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

class DS18B20Sensor {
  public:
    DS18B20Sensor(Logger &logger, const uint8_t *address, const char *label);

    void begin();
    void update();
    float getTemperature() const { return _last_temperature; }

  private:
    Logger &_logger;
    OneWire *_oneWire;
    DallasTemperature *_sensors;
    uint8_t _address[8];
    const char *_label;
    const char *_id;
    float _last_temperature;
    bool _initialized;
    bool _in_error_state;
    unsigned long _last_update_time;
    unsigned long _conversion_start_time;
    bool _conversion_pending;

    static constexpr float TEMP_ERROR_VALUE = -127.0f;
    static constexpr unsigned long CONVERSION_DELAY_MS =
        750; // 12-bit resolution
};

#endif // DS18B20_H
