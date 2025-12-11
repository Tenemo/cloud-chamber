/**
 * @file CrashLog.cpp
 * @brief Implementation of SPIFFS-based crash logging
 */

#include "CrashLog.h"
#include <SPIFFS.h>
#include <esp_system.h>

// Static member definitions
bool CrashLog::_initialized = false;
const char *CrashLog::CRASH_LOG_PATH = "/crash.log";
const size_t CrashLog::MAX_LOG_SIZE = 8192;  // 8KB max log file
const size_t CrashLog::MAX_ENTRY_SIZE = 128; // Max chars per entry

bool CrashLog::begin() {
    if (_initialized) {
        return true;
    }

    if (!SPIFFS.begin(true)) { // true = format if mount fails
        Serial.println("CrashLog: SPIFFS mount failed!");
        return false;
    }

    _initialized = true;

    // Log the reset reason on startup
    logResetReason();

    Serial.println("CrashLog initialized.");

    // Check if there are existing crash logs
    if (SPIFFS.exists(CRASH_LOG_PATH)) {
        File f = SPIFFS.open(CRASH_LOG_PATH, "r");
        if (f) {
            size_t size = f.size();
            f.close();
            if (size > 0) {
                Serial.printf("CrashLog: %d bytes of crash data available\n",
                              (int)size);
            }
        }
    }

    return true;
}

void CrashLog::logResetReason() {
    if (!_initialized) {
        return;
    }

    esp_reset_reason_t reason = esp_reset_reason();

    // Only log abnormal resets
    switch (reason) {
    case ESP_RST_PANIC:
        logCritical("RESET", "Software panic");
        break;
    case ESP_RST_INT_WDT:
        logCritical("RESET", "Interrupt watchdog");
        break;
    case ESP_RST_TASK_WDT:
        logCritical("RESET", "Task watchdog");
        break;
    case ESP_RST_WDT:
        logCritical("RESET", "Other watchdog");
        break;
    case ESP_RST_BROWNOUT:
        logCritical("RESET", "Brownout");
        break;
    case ESP_RST_SDIO:
        logCritical("RESET", "SDIO reset");
        break;
    default:
        // Normal resets (power-on, software reset, deep sleep) - don't log
        break;
    }
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

void CrashLog::rotateIfNeeded() {
    if (!_initialized || !SPIFFS.exists(CRASH_LOG_PATH)) {
        return;
    }

    File f = SPIFFS.open(CRASH_LOG_PATH, "r");
    if (!f) {
        return;
    }

    size_t size = f.size();
    f.close();

    if (size > MAX_LOG_SIZE) {
        // Read last half of file
        f = SPIFFS.open(CRASH_LOG_PATH, "r");
        if (!f) {
            return;
        }

        // Skip first half
        f.seek(size / 2);

        // Read to end
        String remaining;
        while (f.available()) {
            remaining += (char)f.read();
        }
        f.close();

        // Find first complete line
        int newlinePos = remaining.indexOf('\n');
        if (newlinePos > 0) {
            remaining = remaining.substring(newlinePos + 1);
        }

        // Write truncated content
        f = SPIFFS.open(CRASH_LOG_PATH, "w");
        if (f) {
            f.print("[LOG ROTATED]\n");
            f.print(remaining);
            f.close();
        }
    }
}

void CrashLog::logCritical(const char *message) {
    logCritical("EVENT", message);
}

void CrashLog::logCritical(const char *category, const char *message) {
    if (!_initialized) {
        // Fallback to serial if SPIFFS not available
        Serial.printf("CRASH[%s]: %s\n", category, message);
        return;
    }

    rotateIfNeeded();

    File f = SPIFFS.open(CRASH_LOG_PATH, "a");
    if (!f) {
        Serial.println("CrashLog: Failed to open file for writing");
        return;
    }

    // Format: [timestamp_ms] CATEGORY: message
    unsigned long timestamp = millis();
    char entry[MAX_ENTRY_SIZE];
    snprintf(entry, sizeof(entry), "[%lu] %s: %s\n", timestamp, category,
             message);

    f.print(entry);
    f.close();

    // Also output to serial for immediate visibility
    Serial.print("CRASH: ");
    Serial.print(entry);
}

void CrashLog::dumpToSerial() {
    Serial.println("=== BEGIN CRASH LOG DUMP ===");

    if (!_initialized) {
        Serial.println("(SPIFFS not initialized)");
        Serial.println("=== END CRASH LOG DUMP ===");
        return;
    }

    Serial.printf("Last reset reason: %s\n", getResetReasonString());

    if (!SPIFFS.exists(CRASH_LOG_PATH)) {
        Serial.println("(No crash log file)");
        Serial.println("=== END CRASH LOG DUMP ===");
        return;
    }

    File f = SPIFFS.open(CRASH_LOG_PATH, "r");
    if (!f) {
        Serial.println("(Failed to open crash log)");
        Serial.println("=== END CRASH LOG DUMP ===");
        return;
    }

    Serial.printf("Log size: %d bytes\n", (int)f.size());
    Serial.println("---");

    while (f.available()) {
        Serial.write(f.read());
    }

    f.close();
    Serial.println("---");
    Serial.println("=== END CRASH LOG DUMP ===");
}

void CrashLog::clear() {
    if (!_initialized) {
        return;
    }

    if (SPIFFS.exists(CRASH_LOG_PATH)) {
        SPIFFS.remove(CRASH_LOG_PATH);
        Serial.println("CrashLog: Cleared");
    }
}
