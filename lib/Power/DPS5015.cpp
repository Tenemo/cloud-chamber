#include "DPS5015.h"
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
      _consecutive_errors(0), _last_command_time(0),
      _reads_since_command(0), _pending_writes(0),
      _pending_config{0.0f, 0.0f, false, false} {}

void DPS5015::begin(int rxPin, int txPin, unsigned long baud) {
    if (_initialized)
        return;

    // Use config baud rate if default (0) passed, otherwise use specified
    unsigned long actual_baud = (baud == 0) ? Modbus::BAUD_RATE : baud;

    // Initialize Serial for Modbus communication
    _serial.begin(actual_baud, SERIAL_8N1, rxPin, txPin);
    delay(100); // Allow UART hardware to stabilize after configuration

    _initialized = true;
    _last_update_time = 0; // Force immediate first read attempt
}

void DPS5015::update() {
    if (!_initialized)
        return;

    unsigned long current_time = millis();

    // ---------------------------------------------------------------------
    // If waiting for a response, try to consume it or timeout
    // ---------------------------------------------------------------------
    if (_state == ModbusState::WAITING_RESPONSE) {
        if (current_time - _request_start_time > RESPONSE_TIMEOUT_MS) {
            handleCommError();
            _state = ModbusState::IDLE;
            _last_update_time = current_time;
            return;
        }

        if (_serial.available() >= (int)_expected_response_length) {
            if (_active_txn.type == TxnType::ReadHolding) {
                uint16_t buffer[16]; // enough for 10 registers
                if (checkReadResponse(_active_txn.value, buffer)) {
                    handleReadComplete(buffer);
                } else {
                    handleCommError();
                }
            } else { // Write
                if (!checkWriteResponse()) {
                    handleCommError();
                }
            }
            _state = ModbusState::IDLE;
            _last_update_time = current_time;
        }
        return;
    }

    // ---------------------------------------------------------------------
    // Choose next transaction: emergency writes > normal writes > polling
    // ---------------------------------------------------------------------
    Txn next{};
    if (popNextTxn(next)) {
        if (sendTxn(next)) {
            _active_txn = next;
            _state = ModbusState::WAITING_RESPONSE;
            _request_start_time = current_time;
            _expected_response_length =
                (next.type == TxnType::ReadHolding)
                    ? static_cast<size_t>(5 + next.value * 2)
                    : 8;
        }
        return;
    }

    // No pending writes - throttle polling to update interval
        if (current_time - _last_update_time <
            Intervals::DPS5015_UPDATE_INTERVAL_MS) {
            return;
        }

    // Default polling read of 10 registers starting at 0x0000
    next = {TxnType::ReadHolding, REG_SET_VOLTAGE, 10};
    if (sendTxn(next)) {
        _active_txn = next;
        _state = ModbusState::WAITING_RESPONSE;
        _request_start_time = current_time;
        _expected_response_length = 5 + (10 * 2);
    }
}

void DPS5015::handleReadComplete(uint16_t *buffer) {
    // Reset consecutive error count on successful read
    _consecutive_errors = 0;

    // Increment read counter for transaction-based grace period
    if (_reads_since_command < READS_REQUIRED_FOR_SETTLE + 1) {
        _reads_since_command++;
    }

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

        // Initialize commanded values from actual DPS state to prevent
        // false manual override detection on startup
        _commanded_voltage = _set_voltage;
        _commanded_current = _set_current;
        _commanded_output = _output_on;

        _logger.log("DPS5015 connected.");
        applyPendingConfig();
        return;
    }

    // Recover from error state
    if (_in_error_state) {
        _in_error_state = false;
        _currently_online = true;
        _logger.log("DPS5015 recovered");

        // Re-apply pending config (ghost state) to restore intended state
        // This handles the case where DPS reset to default while disconnected
        applyPendingConfig();
    }
}

void DPS5015::handleCommError() {
    // Increment consecutive error count
    _consecutive_errors++;

    // If never connected, keep trying silently
    if (!_ever_seen) {
        return;
    }

    // Only transition to error state after Modbus::MAX_RETRIES consecutive
    // failures
    if (_consecutive_errors >= Modbus::MAX_RETRIES && !_in_error_state) {
        _in_error_state = true;
        _currently_online = false;
        _logger.log("DPS5015 comm error");
    }
}

bool DPS5015::queueWrite(uint16_t reg, uint16_t value) {
    uint8_t next = (_normal_head + 1) % NORMAL_QUEUE_SIZE;
    if (next == _normal_tail)
        return false; // full

    _normal_q[_normal_head] = {TxnType::WriteSingle, reg, value};
    _normal_head = next;
    _normal_count++;
    _pending_writes++;
    return true;
}

bool DPS5015::queueEmergencyWrite(uint16_t reg, uint16_t value) {
    uint8_t next = (_emerg_head + 1) % EMERGENCY_QUEUE_SIZE;
    if (next == _emerg_tail)
        return false; // full

    _emergency_q[_emerg_head] = {TxnType::WriteSingle, reg, value};
    _emerg_head = next;
    _emerg_count++;
    _pending_writes++;
    return true;
}

bool DPS5015::popNextTxn(Txn &txn) {
    // Emergency queue has priority
    if (_emerg_count > 0) {
        txn = _emergency_q[_emerg_tail];
        _emerg_tail = (_emerg_tail + 1) % EMERGENCY_QUEUE_SIZE;
        _emerg_count--;
        if (_pending_writes > 0)
            _pending_writes--;
        return true;
    }

    if (_normal_count > 0) {
        txn = _normal_q[_normal_tail];
        _normal_tail = (_normal_tail + 1) % NORMAL_QUEUE_SIZE;
        _normal_count--;
        if (_pending_writes > 0)
            _pending_writes--;
        return true;
    }

    return false;
}

bool DPS5015::sendTxn(const Txn &txn) {
    switch (txn.type) {
    case TxnType::ReadHolding:
        sendReadRequest(txn.reg, txn.value);
        return true;
    case TxnType::WriteSingle:
        sendWriteRequest(txn.reg, txn.value);
        return true;
    default:
        return false;
    }
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
        _reads_since_command = 0; // Reset for transaction-based grace
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
        _reads_since_command = 0; // Reset for transaction-based grace
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
        _reads_since_command = 0; // Reset for transaction-based grace
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

    // Always start grace period when configuration is requested,
    // even if PSU is temporarily offline. This prevents false
    // manual override detection during state transitions.
    _last_command_time = millis();
    _reads_since_command = 0; // Reset for transaction-based grace

    // If already connected, apply immediately
    if (_currently_online) {
        applyPendingConfig();
    }
}

bool DPS5015::setCurrentImmediate(float current) {
    // Emergency, high-priority write using emergency queue (non-blocking)
    if (!_initialized)
        return false;

    float clamped = current;
    uint16_t value = clampAndConvert(clamped, 100.0f, 15.0f);
    if (queueEmergencyWrite(REG_SET_CURRENT, value)) {
        _commanded_current = clamped;
        _last_command_time = millis();
        _reads_since_command = 0;
        return true;
    }
    return false;
}

bool DPS5015::disableOutput() {
    if (!_initialized)
        return false;

    if (queueEmergencyWrite(REG_OUTPUT, 0)) {
        _commanded_output = false;
        _last_command_time = millis();
        _reads_since_command = 0;
        return true;
    }
    return false;
}

bool DPS5015::isInGracePeriod() const {
    // Transaction-based grace period: require N successful Modbus reads
    // after last command before allowing override detection.
    // This is more reliable than time-based because it guarantees
    // fresh data has been read from the DPS.
    return _reads_since_command < READS_REQUIRED_FOR_SETTLE;
}

bool DPS5015::isSettled() const {
    // Returns true if DPS state matches what we last commanded
    // Used to verify DPS has processed our commands before sending new ones

    // If we have pending writes or in grace period, not yet settled
    if (_pending_writes > 0 || isInGracePeriod())
        return false;

    // Check if current matches commanded value (within tolerance)
    float current_diff = fabs(_set_current - _commanded_current);
    return current_diff < Tuning::MANUAL_OVERRIDE_CURRENT_TOLERANCE_A;
}

bool DPS5015::hasCurrentMismatch() const {
    // Returns true if DPS current doesn't match commanded current
    float current_diff = fabs(_set_current - _commanded_current);
    return current_diff > Tuning::MANUAL_OVERRIDE_CURRENT_TOLERANCE_A;
}

bool DPS5015::hasVoltageMismatch() const {
    // Returns true if DPS voltage doesn't match commanded voltage
    float voltage_diff = fabs(_set_voltage - _commanded_voltage);
    return voltage_diff > Tuning::MANUAL_OVERRIDE_VOLTAGE_TOLERANCE_V;
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

    // Unlock first to allow changes
    queueWrite(REG_LOCK, 0);

    // Set hardware protection limits (preset registers 0x52-0x54)
    // These act as failsafes against software/Modbus errors
    // Scale: 0.01V for OVP, 0.01A for OCP, 0.01W for OPP
    queueWrite(REG_OVP, static_cast<uint16_t>(Limits::DPS_OVP_LIMIT * 100.0f));
    queueWrite(REG_OCP, static_cast<uint16_t>(Limits::DPS_OCP_LIMIT * 100.0f));

    // Track commanded values
    _commanded_voltage = _pending_config.voltage;
    _commanded_current = _pending_config.current;
    _commanded_output = _pending_config.output_on;

    // Queue voltage, current, and output settings
    queueWrite(REG_SET_VOLTAGE,
               static_cast<uint16_t>(_pending_config.voltage * 100.0f));
    queueWrite(REG_SET_CURRENT,
               static_cast<uint16_t>(_pending_config.current * 100.0f));
    queueWrite(REG_OUTPUT, _pending_config.output_on ? 1 : 0);

    // Start grace period - prevents false manual override detection
    // while queued writes are being processed
    _last_command_time = millis();
    _reads_since_command = 0; // Reset for transaction-based grace

    // Clear flag so older pending values can't overwrite a newer configure()
    _pending_config.has_config = false;

    // New configure() calls will set has_config again; clearing here prevents
    // stale configs from overwriting newer ones after multiple configure() calls.
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

    // Non-blocking: if not enough data yet, let caller retry next loop
    if (_serial.available() < static_cast<int>(expected_length)) {
        return false;
    }

    uint8_t response[64];
    for (size_t i = 0; i < expected_length && i < sizeof(response); i++) {
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
    // Non-blocking: require full response to be ready
    if (_serial.available() < 8)
        return false;

    uint8_t response[8];
    for (size_t i = 0; i < 8; i++) {
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
