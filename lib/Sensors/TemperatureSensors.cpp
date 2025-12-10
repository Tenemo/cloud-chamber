/**
 * @file TemperatureSensors.cpp
 * @brief Implementation of unified temperature sensor manager
 */

#include "TemperatureSensors.h"
#include "config.h"

TemperatureSensors::TemperatureSensors(Logger &logger)
    : _logger(logger), _oneWire(PIN_DS18B20), _dallasSensors(&_oneWire),
      _pt100(logger, "Aluminum"),
      _hotPlate(logger, _dallasSensors, DS18B20_1_ADDRESS, "Copper") {}

void TemperatureSensors::begin() {
    _dallasSensors.begin();
    _dallasSensors.setWaitForConversion(false);
    _hotPlate.begin();
    _pt100.begin();

    _logger.log("Sensors: init");
}

void TemperatureSensors::update() {
    _pt100.update();
    _hotPlate.update();
}
