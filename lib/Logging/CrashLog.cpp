/**
 * @file CrashLog.cpp
 * @brief Stubbed crash logging: serial only, no NVM/PSRAM.
 */

#include "CrashLog.h"
#include <esp_system.h>

// Static members
bool CrashLog::_initialized = false;
const char *CrashLog::CRASH_LOG_PATH = "/crash.log";
const size_t CrashLog::MAX_LOG_SIZE = 0;
const size_t CrashLog::MAX_ENTRY_SIZE = 0;

bool CrashLog::begin() {
    _initialized = true;
    Serial.println("CrashLog disabled: serial-only (no SPIFFS).");
    // Still print reset reason for visibility
    Serial.printf("Reset reason: %s\n", getResetReasonString());
    return true;
}

void CrashLog::logResetReason() {
    // Serial-only; nothing persistent
    Serial.printf("Reset reason: %s\n", getResetReasonString());
}

const char *CrashLog::getResetReasonString() {
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "Power-on";
    case ESP_RST_EXT:
        return "External reset";
    case ESP_RST_SW:
        return "Software reset";
    case ESP_RST_PANIC:
        return "Exception/panic";
    case ESP_RST_INT_WDT:
        return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:
        return "Task watchdog";
    case ESP_RST_WDT:
        return "Other watchdog";
    case ESP_RST_DEEPSLEEP:
        return "Deep sleep wake";
    case ESP_RST_BROWNOUT:
        return "Brownout";
    case ESP_RST_SDIO:
        return "SDIO";
    default:
        return "Unknown";
    }
}

void CrashLog::logCritical(const char *message) {
    logCritical("EVENT", message);
}

void CrashLog::logCritical(const char *category, const char *message) {
    Serial.printf("CRASH[%s]: %s\n", category, message);
}

void CrashLog::dumpToSerial() {
    Serial.println("=== CRASH LOG DISABLED (serial only) ===");
}

void CrashLog::clear() {
    // Nothing to clear
}
