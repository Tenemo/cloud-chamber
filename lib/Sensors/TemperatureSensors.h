/**
 * @file TemperatureSensors.h
 * @brief Unified temperature sensor manager for cloud chamber
 *
 * This class owns and manages all temperature sensors:
 * - PT100 (cold plate) via MAX31865
 * - DS18B20 (hot plate) via OneWire
 *
 * USAGE:
 * ------
 * TemperatureSensors sensors(logger);
 * sensors.begin();
 *
 * // In loop:
 * sensors.update();
 *
 * // Access readings:
 * float coldTemp = sensors.getColdPlateTemperature();
 * float hotTemp = sensors.getHotPlateTemperature();
 *
 * // Or get sensor references for other modules:
 * PT100Sensor& cold = sensors.getColdPlateSensor();
 * DS18B20Sensor& hot = sensors.getHotPlateSensor();
 */

#ifndef TEMPERATURE_SENSORS_H
#define TEMPERATURE_SENSORS_H

#include "DS18B20.h"
#include "Logger.h"
#include "PT100.h"
#include "config.h"
#include <DallasTemperature.h>
#include <OneWire.h>

class TemperatureSensors {
  public:
    explicit TemperatureSensors(Logger &logger);

    /**
     * @brief Initialize all temperature sensors
     *
     * Sets up OneWire bus and initializes both PT100 and DS18B20 sensors.
     */
    void begin();

    /**
     * @brief Update all sensors (call in main loop)
     */
    void update();

    // =========================================================================
    // Temperature Accessors
    // =========================================================================

    /**
     * @brief Get cold plate temperature (PT100)
     */
    float getColdPlateTemperature() const { return _pt100.getTemperature(); }

    /**
     * @brief Get hot plate temperature (DS18B20)
     */
    float getHotPlateTemperature() const { return _hotPlate.getTemperature(); }

    // =========================================================================
    // Sensor Status
    // =========================================================================

    /**
     * @brief Check if cold plate sensor is in error state
     */
    bool isColdPlateError() const { return _pt100.isInError(); }

    /**
     * @brief Check if hot plate sensor is connected and valid
     */
    bool isHotPlateConnected() const { return _hotPlate.isConnected(); }

    // =========================================================================
    // Sensor References (for modules that need direct access)
    // =========================================================================

    PT100Sensor &getColdPlateSensor() { return _pt100; }
    const PT100Sensor &getColdPlateSensor() const { return _pt100; }

    DS18B20Sensor &getHotPlateSensor() { return _hotPlate; }
    const DS18B20Sensor &getHotPlateSensor() const { return _hotPlate; }

  private:
    Logger &_logger;

    // OneWire bus (owned)
    OneWire _oneWire;
    DallasTemperature _dallasSensors;

    // Sensors (owned)
    PT100Sensor _pt100;
    DS18B20Sensor _hotPlate;
};

#endif // TEMPERATURE_SENSORS_H
