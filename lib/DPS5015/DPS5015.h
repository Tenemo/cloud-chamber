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
#include "config.h"
#include <Arduino.h>

// State machine states for non-blocking Modbus communication
enum class ModbusState {
    IDLE,             // Ready to start a new transaction
    WAITING_RESPONSE, // Request sent, waiting for response
    WRITE_PENDING,    // Write operation waiting for response
};

class DPS5015 {
  public:
    DPS5015(Logger &logger, const char *label, HardwareSerial &serial = Serial1,
            uint8_t slaveAddress = 1);

    void begin(int rxPin = PIN_DPS5015_1_RX, int txPin = PIN_DPS5015_1_TX,
               unsigned long baud = 9600);
    void update();

    // Configure desired settings (applied automatically when connected)
    void configure(float voltage, float current, bool outputOn = true);

    // Setters - these queue write operations (non-blocking)
    bool setVoltage(float voltage);
    bool setCurrent(float current);
    bool setOutput(bool on);

    // Emergency methods - bypass queue for time-critical operations
    bool setCurrentImmediate(float current);
    bool disableOutput();

    // Getters - return last commanded values (what we asked the DPS to do)
    float getCommandedVoltage() const { return _commanded_voltage; }
    float getCommandedCurrent() const { return _commanded_current; }

    // Getters - return last read values (what the DPS reports)
    float getSetVoltage() const { return _set_voltage; }
    float getSetCurrent() const { return _set_current; }
    float getOutputVoltage() const { return _output_voltage; }
    float getOutputCurrent() const { return _output_current; }
    float getOutputPower() const { return _output_power; }
    float getInputVoltage() const { return _input_voltage; }
    bool isOutputOn() const { return _output_on; }
    bool isConnected() const { return _connected; }
    bool isInGracePeriod() const; // True if recent command still propagating

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

    // Display line IDs (initialized once in registerDisplayLines)
    char _current_line_id[16];
    char _power_line_id[16];

    // Cached values (read from DPS)
    float _set_voltage;
    float _set_current;
    float _output_voltage;
    float _output_current;
    float _output_power;
    float _input_voltage;
    bool _output_on;
    bool _cc_mode;

    // Commanded values (what we sent to DPS)
    float _commanded_voltage;
    float _commanded_current;
    bool _commanded_output;

    // Non-blocking state machine
    ModbusState _state;
    unsigned long _request_start_time;
    size_t _expected_response_length;
    int _consecutive_errors; // Consecutive comm failures
    unsigned long
        _last_command_time; // When last command was sent (for grace period)

    // Write queue for non-blocking writes
    static constexpr size_t WRITE_QUEUE_SIZE = 8;
    struct WriteRequest {
        uint16_t reg;
        uint16_t value;
    };
    WriteRequest _write_queue[WRITE_QUEUE_SIZE];
    size_t _write_queue_head;
    size_t _write_queue_tail;

    // Pending configuration (applied on first connection)
    struct PendingConfig {
        float voltage;
        float current;
        bool output_on;
        bool has_config;
    };
    PendingConfig _pending_config;

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
    void registerDisplayLines();
    void sendReadRequest(uint16_t startReg, uint16_t count);
    bool checkReadResponse(uint16_t count, uint16_t *buffer);
    void sendWriteRequest(uint16_t reg, uint16_t value);
    bool checkWriteResponse();
    bool queueWrite(uint16_t reg, uint16_t value);
    void processWriteQueue();
    void handleReadComplete(uint16_t *buffer);
    void handleCommError();
    void applyPendingConfig();
    uint16_t calculateCRC(uint8_t *buffer, size_t length);
    void clearSerialBuffer();
};

#endif // DPS5015_H
