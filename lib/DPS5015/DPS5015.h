/**
 * @file DPS5015.h
 * @brief DPS5015-USB programmable power supply interface via Modbus RTU
 *
 * USAGE:
 * ------
 * 1. Create instance:
 *    DPS5015 psu(logger, "PSU");
 *
 * 2. Initialize:
 *    psu.begin();
 *
 * 3. Update readings in main loop:
 *    psu.update();
 *
 * 4. Control the power supply:
 *    psu.setVoltage(12.0f);    // Set voltage to 12V
 *    psu.setCurrent(5.0f);     // Set current limit to 5A
 *    psu.setOutput(true);      // Enable output
 *
 * The sensor automatically registers itself with the Logger and updates
 * the display when values change.
 *
 * MODBUS PROTOCOL:
 * ----------------
 * The DPS5015 uses Modbus RTU at 9600 baud (8N1).
 * Default slave address is 1.
 *
 * Key registers (holding registers, function 0x03 read, 0x06 write):
 * - 0x0000: Set voltage (0.01V units)
 * - 0x0001: Set current (0.001A units)
 * - 0x0002: Actual output voltage (0.01V units)
 * - 0x0003: Actual output current (0.001A units)
 * - 0x0004: Actual output power (0.01W units)
 * - 0x0005: Input voltage (0.01V units)
 * - 0x0006: Lock (0=unlocked, 1=locked)
 * - 0x0007: Protection status
 * - 0x0008: CV/CC mode (0=CV, 1=CC)
 * - 0x0009: Output on/off (0=off, 1=on)
 */

#ifndef DPS5015_H
#define DPS5015_H

#include "Logger.h"
#include <Arduino.h>

class DPS5015 {
  public:
    DPS5015(Logger &logger, const char *label, HardwareSerial &serial = Serial1,
            uint8_t slaveAddress = 1);

    void begin(int rxPin = 44, int txPin = 43, unsigned long baud = 9600);
    void update();

    // Setters - return true on success
    bool setVoltage(float voltage);
    bool setCurrent(float current);
    bool setOutput(bool on);

    // Getters - return last read values
    float getSetVoltage() const { return _set_voltage; }
    float getSetCurrent() const { return _set_current; }
    float getOutputVoltage() const { return _output_voltage; }
    float getOutputCurrent() const { return _output_current; }
    float getOutputPower() const { return _output_power; }
    float getInputVoltage() const { return _input_voltage; }
    bool isOutputOn() const { return _output_on; }
    bool isConstantCurrent() const { return _cc_mode; }
    bool isConnected() const { return _connected; }

  private:
    Logger &_logger;
    HardwareSerial &_serial;
    const char *_label;
    uint8_t _slave_address;
    bool _initialized;
    bool _connected;
    bool _ever_connected; // True if device was successfully read at least once
    bool _in_error_state;
    unsigned long _last_update_time;

    // Cached values
    float _set_voltage;
    float _set_current;
    float _output_voltage;
    float _output_current;
    float _output_power;
    float _input_voltage;
    bool _output_on;
    bool _cc_mode;

    // Modbus register addresses
    static constexpr uint16_t REG_SET_VOLTAGE = 0x0000;
    static constexpr uint16_t REG_SET_CURRENT = 0x0001;
    static constexpr uint16_t REG_OUT_VOLTAGE = 0x0002;
    static constexpr uint16_t REG_OUT_CURRENT = 0x0003;
    static constexpr uint16_t REG_OUT_POWER = 0x0004;
    static constexpr uint16_t REG_IN_VOLTAGE = 0x0005;
    static constexpr uint16_t REG_LOCK = 0x0006;
    static constexpr uint16_t REG_PROTECTION = 0x0007;
    static constexpr uint16_t REG_CV_CC = 0x0008;
    static constexpr uint16_t REG_OUTPUT = 0x0009;

    // Timing constants
    static constexpr unsigned long RESPONSE_TIMEOUT_MS = 500;

    // Modbus functions
    bool readRegisters(uint16_t startReg, uint16_t count, uint16_t *buffer);
    bool writeRegister(uint16_t reg, uint16_t value);
    uint16_t calculateCRC(uint8_t *buffer, size_t length);
    void clearSerialBuffer();
};

#endif // DPS5015_H
