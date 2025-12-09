#include "DPS5015.h"
#include "config.h"

DPS5015::DPS5015(Logger &logger, const char *label, HardwareSerial &serial,
                 uint8_t slaveAddress)
    : _logger(logger), _serial(serial), _label(label),
      _slave_address(slaveAddress), _initialized(false), _connected(false),
      _ever_connected(false), _in_error_state(false), _last_update_time(0),
      _set_voltage(0.0f), _set_current(0.0f), _output_voltage(0.0f),
      _output_current(0.0f), _output_power(0.0f), _input_voltage(0.0f),
      _output_on(false), _cc_mode(false) {}

void DPS5015::begin(int rxPin, int txPin, unsigned long baud) {
    if (_initialized)
        return; // prevent re-initialization

    // Initialize Serial1 for Modbus communication
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    delay(100); // allow serial to stabilize

    // Try to read initial values to check if device is connected
    uint16_t buffer[10];
    if (readRegisters(REG_SET_VOLTAGE, 10, buffer)) {
        _connected = true;
        _ever_connected = true;
        _set_voltage = buffer[0] / 100.0f;
        _set_current = buffer[1] / 1000.0f;
        _output_voltage = buffer[2] / 100.0f;
        _output_current = buffer[3] / 1000.0f;
        _output_power = buffer[4] / 100.0f;
        _input_voltage = buffer[5] / 100.0f;
        _cc_mode = (buffer[8] == 1);
        _output_on = (buffer[9] == 1);

        // Register display lines
        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s V:", _label);
        _logger.registerLine(_label, labelBuf, "V", _output_voltage);

        char currentId[16];
        snprintf(currentId, sizeof(currentId), "%s_I", _label);
        snprintf(labelBuf, sizeof(labelBuf), "%s I:", _label);
        _logger.registerLine(currentId, labelBuf, "A", _output_current);

        _logger.log("DPS5015 initialized.");
    } else {
        _connected = false;
        _in_error_state = true;

        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s:", _label);
        _logger.registerTextLine(_label, labelBuf, "NO CONN");
        _logger.log("DPS5015 not found");
    }

    _initialized = true;
}

void DPS5015::update() {
    if (!_initialized)
        return;

    unsigned long current_time = millis();
    if (current_time - _last_update_time < DPS5015_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_time = current_time;

    // Read all relevant registers at once (0x0000 to 0x0009)
    uint16_t buffer[10];
    if (!readRegisters(REG_SET_VOLTAGE, 10, buffer)) {
        // If never connected, keep trying silently
        if (!_ever_connected) {
            return;
        }

        // Only log error if we were previously connected
        if (!_in_error_state) {
            _in_error_state = true;
            _connected = false;
            _logger.log("DPS5015 comm error");
            _logger.updateLineText(_label, "ERROR");

            char currentId[16];
            snprintf(currentId, sizeof(currentId), "%s_I", _label);
            _logger.updateLineText(currentId, "ERROR");
        }
        return;
    }

    // Parse register values
    _set_voltage = buffer[0] / 100.0f;
    _set_current = buffer[1] / 1000.0f;
    _output_voltage = buffer[2] / 100.0f;
    _output_current = buffer[3] / 1000.0f;
    _output_power = buffer[4] / 100.0f;
    _input_voltage = buffer[5] / 100.0f;
    _cc_mode = (buffer[8] == 1);
    _output_on = (buffer[9] == 1);

    // First successful connection after being disconnected at startup
    if (!_ever_connected) {
        _ever_connected = true;
        _connected = true;
        _in_error_state = false;

        // Re-register display lines (replace "NO CONN" with actual values)
        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s V:", _label);
        _logger.registerLine(_label, labelBuf, "V", _output_voltage);

        char currentId[16];
        snprintf(currentId, sizeof(currentId), "%s_I", _label);
        snprintf(labelBuf, sizeof(labelBuf), "%s I:", _label);
        _logger.registerLine(currentId, labelBuf, "A", _output_current);

        _logger.log("DPS5015 connected.");
        return;
    }

    // Recover from error state
    if (_in_error_state) {
        _in_error_state = false;
        _connected = true;
        _logger.log("DPS5015 recovered");

        // Re-register display lines
        char labelBuf[16];
        snprintf(labelBuf, sizeof(labelBuf), "%s V:", _label);
        _logger.registerLine(_label, labelBuf, "V", _output_voltage);

        char currentId[16];
        snprintf(currentId, sizeof(currentId), "%s_I", _label);
        snprintf(labelBuf, sizeof(labelBuf), "%s I:", _label);
        _logger.registerLine(currentId, labelBuf, "A", _output_current);
    }

    // Update display
    _logger.updateLine(_label, _output_voltage);

    char currentId[16];
    snprintf(currentId, sizeof(currentId), "%s_I", _label);
    _logger.updateLine(currentId, _output_current);
}

bool DPS5015::setVoltage(float voltage) {
    if (!_connected)
        return false;

    uint16_t value = static_cast<uint16_t>(voltage * 100.0f);
    if (writeRegister(REG_SET_VOLTAGE, value)) {
        _set_voltage = voltage;
        return true;
    }
    return false;
}

bool DPS5015::setCurrent(float current) {
    if (!_connected)
        return false;

    uint16_t value = static_cast<uint16_t>(current * 1000.0f);
    if (writeRegister(REG_SET_CURRENT, value)) {
        _set_current = current;
        return true;
    }
    return false;
}

bool DPS5015::setOutput(bool on) {
    if (!_connected)
        return false;

    if (writeRegister(REG_OUTPUT, on ? 1 : 0)) {
        _output_on = on;
        return true;
    }
    return false;
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

bool DPS5015::readRegisters(uint16_t startReg, uint16_t count,
                            uint16_t *buffer) {
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

    // Wait for response
    unsigned long start_time = millis();
    size_t expected_length =
        5 + (count * 2); // addr + func + byte_count + data + crc

    while (_serial.available() < (int)expected_length) {
        if (millis() - start_time > RESPONSE_TIMEOUT_MS) {
            _logger.log("DPS: timeout", true);
            return false; // Timeout
        }
        delay(1);
    }

    // Read response
    uint8_t response[64];
    size_t bytes_read = 0;
    while (_serial.available() && bytes_read < sizeof(response)) {
        response[bytes_read++] = _serial.read();
    }

    // Validate response
    if (bytes_read < expected_length) {
        return false;
    }

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
        response[bytes_read - 2] | (response[bytes_read - 1] << 8);
    uint16_t calculated_crc = calculateCRC(response, bytes_read - 2);
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

bool DPS5015::writeRegister(uint16_t reg, uint16_t value) {
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

    // Wait for response (echo of request)
    unsigned long start_time = millis();
    size_t expected_length = 8;

    while (_serial.available() < (int)expected_length) {
        if (millis() - start_time > RESPONSE_TIMEOUT_MS) {
            return false; // Timeout
        }
        delay(1);
    }

    // Read response
    uint8_t response[8];
    size_t bytes_read = 0;
    while (_serial.available() && bytes_read < sizeof(response)) {
        response[bytes_read++] = _serial.read();
    }

    // Validate response (should be echo of request)
    if (bytes_read < expected_length) {
        return false;
    }

    // Check that response matches request
    for (size_t i = 0; i < 6; i++) {
        if (response[i] != request[i]) {
            return false;
        }
    }

    // Verify CRC
    uint16_t received_crc = response[6] | (response[7] << 8);
    uint16_t calculated_crc = calculateCRC(response, 6);
    if (received_crc != calculated_crc) {
        return false;
    }

    return true;
}
