/**
 * @file TemperatureSensors.cpp
 * @brief Implementation of unified temperature sensor manager
 */

#include "TemperatureSensors.h"

TemperatureSensors::TemperatureSensors(Logger &logger)
    : _logger(logger), _oneWire(PIN_DS18B20), _dallasSensors(&_oneWire),
      _pt100(logger, "PT100"),
      _hotPlate(logger, _dallasSensors, DS18B20_1_ADDRESS, "HOT") {}

void TemperatureSensors::begin() {
    // Initialize DS18B20 bus
    _dallasSensors.begin();
    _dallasSensors.setWaitForConversion(false);

    // Initialize individual sensors
    _hotPlate.begin();
    _pt100.begin();

    _logger.log("Sensors: init");
}

void TemperatureSensors::update() {
    _pt100.update();
    _hotPlate.update();
}
