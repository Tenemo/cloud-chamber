/**
 * @file DS18B20.h
 * @brief DS18B20 digital temperature sensor interface using OneWire protocol
 *
 * USAGE:
 * ------
 * 1. Create shared bus (in main.cpp):
 *    OneWire oneWire(PIN_DS18B20);
 *    DallasTemperature dallasSensors(&oneWire);
 *
 * 2. Create sensor instances:
 *    DS18B20Sensor ds_sensor1(logger, dallasSensors, address1, "TEMP_1");
 *    DS18B20Sensor ds_sensor2(logger, dallasSensors, address2, "TEMP_2");
 *
 * 3. Initialize shared bus once, then each sensor:
 *    dallasSensors.begin();
 *    ds_sensor1.begin();
 *    ds_sensor2.begin();
 *
 * 4. Update temperature in main loop:
 *    ds_sensor1.update();
 *    ds_sensor2.update();
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
    DS18B20Sensor(Logger &logger, DallasTemperature &sensors,
                  const uint8_t *address, const char *label);

    void begin();
    void update();
    float getTemperature() const { return _last_temperature; }

  private:
    Logger &_logger;
    DallasTemperature &_sensors;
    uint8_t _address[8];
    const char *_label;
    const char *_id;
    float _last_temperature;
    bool _initialized;
    bool _in_error_state;
    unsigned long _last_update_time;

    static constexpr float TEMP_ERROR_VALUE = -127.0f;
    static constexpr unsigned long CONVERSION_DELAY_MS =
        750; // 12-bit resolution

    // Shared state for coordinating bus-wide conversions
    static unsigned long _shared_conversion_start_time;
    static bool _shared_conversion_pending;
};

#endif // DS18B20_H
