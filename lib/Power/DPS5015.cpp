#include "DPS5015.h"
#include "config.h"
#include <cmath>

DPS5015::DPS5015(Logger &logger, const char *label, HardwareSerial &serial,
                 uint8_t slaveAddress)
    : _logger(logger), _serial(serial), _label(label),
      _slave_address(slaveAddress), _initialized(false),
      _currently_online(false), _ever_seen(false), _in_error_state(false),
      _last_update_time(0), _set_voltage(0.0f), _set_current(0.0f),
      _output_voltage(0.0f), _output_current(0.0f), _output_power(0.0f),
      _input_voltage(0.0f), _output_on(false), _cc_mode(false),
      _commanded_voltage(0.0f), _commanded_current(0.0f),
      _commanded_output(false), _state(ModbusState::IDLE),
      _request_start_time(0), _expected_response_length(0),
      _consecutive_errors(0), _write_retry_count(0), _last_command_time(0),
      _current_write{0, 0}, _pending_writes(0), _write_queue_head(0),
      _write_queue_tail(0), _pending_config{0.0f, 0.0f, false, false} {}

void DPS5015::begin(int rxPin, int txPin, unsigned long baud) {
    if (_initialized)
        return; // prevent re-initialization

    // Use config baud rate if default (0) passed, otherwise use specified
    unsigned long actual_baud = (baud == 0) ? MODBUS_BAUD_RATE : baud;

    // Initialize Serial for Modbus communication
    _serial.begin(actual_baud, SERIAL_8N1, rxPin, txPin);
    delay(100); // Allow UART hardware to stabilize after configuration

    // Register placeholder display line for initial state
    char labelBuf[16];
    Logger::formatLabel(labelBuf, sizeof(labelBuf), _label);
    _logger.registerTextLine(_label, labelBuf, "INIT...");

    _initialized = true;
    _last_update_time = 0; // Force immediate first read attempt
}

void DPS5015::registerDisplayLines() {
    // Initialize line IDs once
    snprintf(_current_line_id, sizeof(_current_line_id), "%s_I", _label);
    snprintf(_power_line_id, sizeof(_power_line_id), "%s_P", _label);

    char labelBuf[16];
    snprintf(labelBuf, sizeof(labelBuf), "%s V:", _label);
    _logger.registerLine(_label, labelBuf, "V", _output_voltage);

    snprintf(labelBuf, sizeof(labelBuf), "%s I:", _label);
    _logger.registerLine(_current_line_id, labelBuf, "A", _output_current);

    snprintf(labelBuf, sizeof(labelBuf), "%s P:", _label);
    _logger.registerLine(_power_line_id, labelBuf, "W", _output_power);
}

void DPS5015::update() {
    if (!_initialized)
        return;

    unsigned long current_time = millis();

    switch (_state) {
    case ModbusState::IDLE: {
        // Process any pending writes first
        if (_write_queue_head != _write_queue_tail) {
            processWriteQueue();
            return;
        }

        // Check if it's time for a new read
        if (current_time - _last_update_time < DPS5015_UPDATE_INTERVAL_MS) {
            return;
        }

        // Start a new read request
        sendReadRequest(REG_SET_VOLTAGE, 10);
        _state = ModbusState::WAITING_RESPONSE;
        _request_start_time = current_time;
        _expected_response_length =
            5 + (10 * 2); // addr + func + count + data + crc
        break;
    }

    case ModbusState::WAITING_RESPONSE: {
        // Check for timeout
        if (current_time - _request_start_time > RESPONSE_TIMEOUT_MS) {
            _state = ModbusState::IDLE;
            _last_update_time = current_time;
            handleCommError();
            return;
        }

        // Check if we have enough data
        if (_serial.available() >= (int)_expected_response_length) {
            uint16_t buffer[10];
            if (checkReadResponse(10, buffer)) {
                handleReadComplete(buffer);
            } else {
                handleCommError();
            }
            _state = ModbusState::IDLE;
            _last_update_time = current_time;
        }
        break;
    }

    case ModbusState::WRITE_PENDING: {
        // Check for timeout
        if (current_time - _request_start_time > RESPONSE_TIMEOUT_MS) {
            _write_retry_count++;
            if (_write_retry_count < MODBUS_MAX_RETRIES) {
                // Retry the write
                sendWriteRequest(_current_write.reg, _current_write.value);
                _request_start_time = millis();
                return;
            }
            // Max retries exceeded, give up on this write
            _state = ModbusState::IDLE;
            _write_retry_count = 0;
            _logger.log("DPS: write timeout", true);
            return;
        }

        // Check if we have the response (8 bytes for write)
        if (_serial.available() >= 8) {
            if (!checkWriteResponse()) {
                _write_retry_count++;
                if (_write_retry_count < MODBUS_MAX_RETRIES) {
                    // Retry the write
                    sendWriteRequest(_current_write.reg, _current_write.value);
                    _request_start_time = millis();
                    return;
                }
                _logger.log("DPS: write fail", true);
            }
            _state = ModbusState::IDLE;
            _write_retry_count = 0;
        }
        break;
    }
    }
}

void DPS5015::handleReadComplete(uint16_t *buffer) {
    // Reset consecutive error count on successful read
    _consecutive_errors = 0;

    // Parse register values
    _set_voltage = buffer[0] / 100.0f;
    _set_current = buffer[1] / 100.0f;
    _output_voltage = buffer[2] / 100.0f;
    _output_current = buffer[3] / 100.0f;
    _output_power = buffer[4] / 100.0f;
    _input_voltage = buffer[5] / 100.0f;
    _cc_mode = (buffer[8] == 1);
    _output_on = (buffer[9] == 1);

    // First successful connection
    if (!_ever_seen) {
        _ever_seen = true;
        _currently_online = true;
        _in_error_state = false;
        registerDisplayLines();
        _logger.log("DPS5015 connected.");
        applyPendingConfig();
        return;
    }

    // Recover from error state
    if (_in_error_state) {
        _in_error_state = false;
        _currently_online = true;
        registerDisplayLines();
        _logger.log("DPS5015 recovered");
    }

    // Update display
    _logger.updateLine(_label, _output_voltage);
    _logger.updateLine(_current_line_id, _output_current);
    _logger.updateLine(_power_line_id, _output_power);
}

void DPS5015::handleCommError() {
    // Increment consecutive error count
    _consecutive_errors++;

    // If never connected, keep trying silently
    if (!_ever_seen) {
        return;
    }

    // Only transition to error state after MODBUS_MAX_RETRIES consecutive
    // failures
    if (_consecutive_errors >= MODBUS_MAX_RETRIES && !_in_error_state) {
        _in_error_state = true;
        _currently_online = false;
        _logger.log("DPS5015 comm error");
        _logger.updateLineText(_label, "ERROR");
        _logger.updateLineText(_current_line_id, "ERROR");
        _logger.updateLineText(_power_line_id, "ERROR");
    }
}

bool DPS5015::queueWrite(uint16_t reg, uint16_t value) {
    size_t next_head = (_write_queue_head + 1) % WRITE_QUEUE_SIZE;
    if (next_head == _write_queue_tail) {
        // Queue full
        return false;
    }

    _write_queue[_write_queue_head].reg = reg;
    _write_queue[_write_queue_head].value = value;
    _write_queue_head = next_head;

    // Track pending writes for manual override detection
    _pending_writes++;

    return true;
}

void DPS5015::processWriteQueue() {
    if (_write_queue_tail == _write_queue_head) {
        return; // Queue empty
    }

    WriteRequest &req = _write_queue[_write_queue_tail];
    _write_queue_tail = (_write_queue_tail + 1) % WRITE_QUEUE_SIZE;

    // Save for retry
    _current_write = req;
    _write_retry_count = 0;

    sendWriteRequest(req.reg, req.value);
    _state = ModbusState::WRITE_PENDING;
    _request_start_time = millis();

    // Decrement pending count (write has been sent, awaiting response)
    if (_pending_writes > 0)
        _pending_writes--;
}

// Helper to clamp float to range and convert to uint16_t safely
// Also updates the input value to the clamped result
static uint16_t clampAndConvert(float &value, float scale, float maxVal) {
    if (value < 0.0f)
        value = 0.0f;
    if (value > maxVal)
        value = maxVal;
    return static_cast<uint16_t>(value * scale);
}

bool DPS5015::setVoltage(float voltage) {
    if (!_currently_online && _ever_seen)
        return false;

    // Clamp voltage to valid range (0-50V for DPS5015)
    float clamped = voltage;
    uint16_t value = clampAndConvert(clamped, 100.0f, 50.0f);
    if (queueWrite(REG_SET_VOLTAGE, value)) {
        _commanded_voltage = clamped;
        _last_command_time = millis();
        return true;
    }
    return false;
}

bool DPS5015::setCurrent(float current) {
    if (!_currently_online && _ever_seen)
        return false;

    // Clamp current to valid range (0-15A for DPS5015)
    float clamped = current;
    uint16_t value = clampAndConvert(clamped, 100.0f, 15.0f);
    if (queueWrite(REG_SET_CURRENT, value)) {
        _commanded_current = clamped;
        _last_command_time = millis();
        return true;
    }
    return false;
}

bool DPS5015::setOutput(bool on) {
    if (!_currently_online && _ever_seen)
        return false;

    if (queueWrite(REG_OUTPUT, on ? 1 : 0)) {
        _commanded_output = on;
        _last_command_time = millis();
        return true;
    }
    return false;
}

bool DPS5015::setOCP(float current) {
    // Set hardware Over Current Protection limit
    // This is a failsafe that triggers if software commands an invalid current
    if (!_currently_online && _ever_seen)
        return false;

    // Clamp to valid OCP range (0-16A, slightly above max for protection
    // margin)
    float clamped = current;
    uint16_t value = clampAndConvert(clamped, 100.0f, 16.0f);
    return queueWrite(REG_OCP, value);
}

bool DPS5015::setOVP(float voltage) {
    // Set hardware Over Voltage Protection limit
    // Failsafe against Modbus errors commanding excessive voltage
    if (!_currently_online && _ever_seen)
        return false;

    // Clamp to valid OVP range (0-55V, slightly above max for protection
    // margin)
    float clamped = voltage;
    uint16_t value = clampAndConvert(clamped, 100.0f, 55.0f);
    return queueWrite(REG_OVP, value);
}

void DPS5015::configure(float voltage, float current, bool outputOn) {
    _pending_config.voltage = voltage;
    _pending_config.current = current;
    _pending_config.output_on = outputOn;
    _pending_config.has_config = true;

    // If already connected, apply immediately
    if (_currently_online) {
        applyPendingConfig();
    }
}

bool DPS5015::writeRegisterImmediate(uint16_t reg, uint16_t value) {
    // Blocking write with retries for time-critical operations
    for (int retry = 0; retry < MODBUS_MAX_RETRIES; retry++) {
        clearSerialBuffer();
        sendWriteRequest(reg, value);

        // Wait for response with timeout
        unsigned long start = millis();
        while (_serial.available() < 8) {
            if (millis() - start > RESPONSE_TIMEOUT_MS) {
                break; // Retry on timeout
            }
        }

        if (_serial.available() >= 8 && checkWriteResponse()) {
            return true;
        }
    }
    return false;
}

bool DPS5015::setCurrentImmediate(float current) {
    if (!_initialized)
        return false;

    // Clamp current to valid range (0-15A for DPS5015)
    float clamped = current;
    uint16_t value = clampAndConvert(clamped, 100.0f, 15.0f);

    if (writeRegisterImmediate(REG_SET_CURRENT, value)) {
        _commanded_current = clamped;
        return true;
    }
    return false;
}

bool DPS5015::disableOutput() {
    if (!_initialized)
        return false;

    if (writeRegisterImmediate(REG_OUTPUT, 0)) {
        _commanded_output = false;
        _last_command_time = millis();
        return true;
    }
    return false;
}

bool DPS5015::isInGracePeriod() const {
    // Returns true if a command was sent recently and we're still waiting
    // for the DPS to process it and for us to read back the new value
    if (_last_command_time == 0)
        return false;
    return (millis() - _last_command_time) < MANUAL_OVERRIDE_GRACE_MS;
}

bool DPS5015::isSettled() const {
    // Returns true if DPS state matches what we last commanded
    // Used to verify DPS has processed our commands before sending new ones

    // If we have pending writes or in grace period, not yet settled
    if (_pending_writes > 0 || isInGracePeriod())
        return false;

    // Check if current matches commanded value (within tolerance)
    float current_diff = fabs(_set_current - _commanded_current);
    return current_diff < MANUAL_OVERRIDE_CURRENT_TOLERANCE_A;
}

bool DPS5015::hasCurrentMismatch() const {
    // Returns true if DPS current doesn't match commanded current
    float current_diff = fabs(_set_current - _commanded_current);
    return current_diff > MANUAL_OVERRIDE_CURRENT_TOLERANCE_A;
}

bool DPS5015::hasVoltageMismatch() const {
    // Returns true if DPS voltage doesn't match commanded voltage
    float voltage_diff = fabs(_set_voltage - _commanded_voltage);
    return voltage_diff > MANUAL_OVERRIDE_VOLTAGE_TOLERANCE_V;
}

bool DPS5015::hasOutputMismatch() const {
    // Returns true if DPS output state doesn't match commanded state
    return _output_on != _commanded_output;
}

bool DPS5015::hasAnyMismatch() const {
    return hasCurrentMismatch() || hasVoltageMismatch() || hasOutputMismatch();
}

void DPS5015::applyPendingConfig() {
    if (!_pending_config.has_config)
        return;

    // Track commanded values
    _commanded_voltage = _pending_config.voltage;
    _commanded_current = _pending_config.current;
    _commanded_output = _pending_config.output_on;

    // Queue all the configuration writes
    queueWrite(REG_LOCK, 0); // Unlock first

    // Set hardware protection limits as failsafes
    // OVP: Protect against Modbus errors commanding excessive voltage
    queueWrite(REG_OVP, static_cast<uint16_t>(DPS_OVP_LIMIT * 100.0f));
    // OCP: Protect against software commanding excessive current
    queueWrite(REG_OCP, static_cast<uint16_t>(DPS_OCP_LIMIT * 100.0f));

    queueWrite(REG_SET_VOLTAGE,
               static_cast<uint16_t>(_pending_config.voltage * 100.0f));
    queueWrite(REG_SET_CURRENT,
               static_cast<uint16_t>(_pending_config.current * 100.0f));
    queueWrite(REG_OUTPUT, _pending_config.output_on ? 1 : 0);

    _pending_config.has_config = false; // Only apply once
}

void DPS5015::clearSerialBuffer() {
    while (_serial.available()) {
        _serial.read();
    }
}

uint16_t DPS5015::calculateCRC(uint8_t *buffer, size_t length) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

void DPS5015::sendReadRequest(uint16_t startReg, uint16_t count) {
    clearSerialBuffer();

    // Build Modbus RTU request (Function 0x03 - Read Holding Registers)
    uint8_t request[8];
    request[0] = _slave_address;
    request[1] = 0x03; // Function code: Read Holding Registers
    request[2] = (startReg >> 8) & 0xFF;
    request[3] = startReg & 0xFF;
    request[4] = (count >> 8) & 0xFF;
    request[5] = count & 0xFF;

    uint16_t crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;        // CRC low byte first
    request[7] = (crc >> 8) & 0xFF; // CRC high byte

    // Send request
    _serial.write(request, 8);
    _serial.flush();
}

bool DPS5015::checkReadResponse(uint16_t count, uint16_t *buffer) {
    size_t expected_length = 5 + (count * 2);

    // Read exactly the expected number of bytes
    uint8_t response[64];
    for (size_t i = 0; i < expected_length && i < sizeof(response); i++) {
        // Wait briefly for each byte if not immediately available
        // Timeout per byte - at 9600 baud, one byte takes ~1ms
        unsigned long start = millis();
        while (!_serial.available()) {
            if (millis() - start > MODBUS_BYTE_TIMEOUT_MS) {
                return false; // Byte timeout
            }
        }
        response[i] = _serial.read();
    }

    // Validate response header
    if (response[0] != _slave_address || response[1] != 0x03) {
        char debugBuf[32];
        snprintf(debugBuf, sizeof(debugBuf), "DPS: bad hdr %02X %02X",
                 response[0], response[1]);
        _logger.log(debugBuf, true);
        return false;
    }

    uint8_t byte_count = response[2];
    if (byte_count != count * 2) {
        return false;
    }

    // Verify CRC
    uint16_t received_crc =
        response[expected_length - 2] | (response[expected_length - 1] << 8);
    uint16_t calculated_crc = calculateCRC(response, expected_length - 2);
    if (received_crc != calculated_crc) {
        _logger.log("DPS: CRC fail", true);
        return false;
    }

    // Parse register values (big-endian)
    for (uint16_t i = 0; i < count; i++) {
        buffer[i] = (response[3 + (i * 2)] << 8) | response[4 + (i * 2)];
    }

    return true;
}

void DPS5015::sendWriteRequest(uint16_t reg, uint16_t value) {
    clearSerialBuffer();

    // Build Modbus RTU request (Function 0x06 - Write Single Register)
    uint8_t request[8];
    request[0] = _slave_address;
    request[1] = 0x06; // Function code: Write Single Register
    request[2] = (reg >> 8) & 0xFF;
    request[3] = reg & 0xFF;
    request[4] = (value >> 8) & 0xFF;
    request[5] = value & 0xFF;

    uint16_t crc = calculateCRC(request, 6);
    request[6] = crc & 0xFF;        // CRC low byte first
    request[7] = (crc >> 8) & 0xFF; // CRC high byte

    // Send request
    _serial.write(request, 8);
    _serial.flush();
}

bool DPS5015::checkWriteResponse() {
    // Read exactly 8 bytes for write response
    uint8_t response[8];
    for (size_t i = 0; i < 8; i++) {
        unsigned long start = millis();
        while (!_serial.available()) {
            if (millis() - start > MODBUS_BYTE_TIMEOUT_MS) {
                return false; // Byte timeout
            }
        }
        response[i] = _serial.read();
    }

    // Validate response header

    if (response[0] != _slave_address || response[1] != 0x06) {
        return false;
    }

    // Verify CRC
    uint16_t received_crc = response[6] | (response[7] << 8);
    uint16_t calculated_crc = calculateCRC(response, 6);
    if (received_crc != calculated_crc) {
        return false;
    }

    return true;
}
