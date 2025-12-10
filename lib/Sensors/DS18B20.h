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
 *
 * BUS COORDINATION:
 * -----------------
 * Multiple DS18B20Sensor instances share static state to coordinate
 * bus-wide temperature conversions. The first sensor to call update()
 * triggers requestTemperatures() for all sensors on the bus. Other
 * sensors wait for the conversion to complete before reading their
 * individual values. This minimizes bus traffic and ensures all
 * sensors read from the same conversion cycle.
 */

#ifndef DS18B20_H
#define DS18B20_H

#include "Logger.h"
#include <DallasTemperature.h>
#include <OneWire.h>

class DS18B20Sensor {
  public:
    DS18B20Sensor(Logger &logger, DallasTemperature &sensors,
                  const uint8_t *address, const char *label);

    void begin();
    void update();
    float getTemperature() const { return _last_temperature; }
    bool isConnected() const { return _ever_connected && !_in_error_state; }

  private:
    Logger &_logger;
    DallasTemperature &_sensors;
    uint8_t _address[8];
    const char *_label;
    float _last_temperature;
    bool _initialized;
    bool _ever_connected; // True if sensor was successfully read at least once
    bool _in_error_state;
    bool _reconnect_pending; // True if waiting for reconnection conversion
    unsigned long _reconnect_start_time;
    unsigned long _last_update_time;

    static constexpr float TEMP_ERROR_VALUE = -127.0f;

    // -------------------------------------------------------------------------
    // Shared Conversion Coordination
    // -------------------------------------------------------------------------
    // All DS18B20 sensors share a single OneWire bus. Temperature conversion
    // takes 750ms (12-bit mode) and affects all sensors simultaneously. To
    // avoid redundant conversions and ensure consistent readings:
    //
    // 1. _shared_conversion_id increments each time requestTemperatures() is
    //    called (by whichever sensor instance triggers first)
    // 2. Each instance tracks _last_read_conversion_id to know if it has
    //    already read from the current conversion cycle
    // 3. If a sensor's last_read_id matches shared_id, it skips reading
    //    (already has current data)
    // 4. _shared_conversion_pending and _shared_conversion_start_time gate
    //    the conversion timing so we wait for 750ms before reading
    //
    // This allows multiple sensors to share one bus efficiently without
    // polling more often than necessary or returning stale data.
    // -------------------------------------------------------------------------
    static unsigned long _shared_conversion_start_time;
    static bool _shared_conversion_pending;
    static uint32_t _shared_conversion_id; // Increments each conversion cycle

    // Per-instance tracking of which conversion was last read
    uint32_t _last_read_conversion_id;
};

#endif // DS18B20_H
