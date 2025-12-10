/**
 * @file CrashLog.h
 * @brief SPIFFS-based persistent crash and critical event logging
 *
 * Saves critical events to flash for post-mortem diagnostics.
 * Unlike PSRAM, this data survives power cycles and resets.
 *
 * WHEN TO USE THIS vs Logger:
 * ---------------------------
 * Use CrashLog ONLY for events where you need to know what happened
 * after a power cycle or reset:
 * - Thermal faults (HOT_SIDE_FAULT)
 * - Emergency shutdowns (startEmergencyShutdown, hardShutdown)
 * - Sensor failures that may have caused a reset
 * - Self-test failures
 * - Watchdog or brownout reset detection
 * - Any safety-critical state transition
 *
 * Use Logger for everything else:
 * - Normal state transitions
 * - Debug information
 * - Optimization decisions
 * - Display updates
 *
 * CrashLog writes to SPIFFS flash which has limited write cycles.
 * Logger writes to PSRAM which is volatile but unlimited writes.
 *
 * USAGE:
 * ------
 * 1. Initialize once during setup:
 *    CrashLog::begin();
 *
 * 2. Log critical events (faults, shutdowns, etc.):
 *    CrashLog::logCritical("THERMAL_FAULT: HOT>70C");
 *
 * 3. Dump logs via serial command or after restart:
 *    CrashLog::dumpToSerial();
 *
 * 4. Clear logs after reviewing:
 *    CrashLog::clear();
 *
 * STORAGE:
 * --------
 * - Uses SPIFFS partition for persistence
 * - Circular file rotation when full
 * - Each entry includes timestamp (millis) and message
 * - Maximum ~50 critical events stored
 */

#ifndef CRASH_LOG_H
#define CRASH_LOG_H

#include <Arduino.h>

/**
 * @brief SPIFFS-backed persistent crash log
 *
 * Static class for global access to crash logging functionality.
 * All methods are thread-safe (single-writer pattern).
 */
class CrashLog {
  public:
    /**
     * @brief Initialize SPIFFS and load existing logs
     * @return true if SPIFFS mounted successfully
     */
    static bool begin();

    /**
     * @brief Log a critical event to SPIFFS
     * @param message The event description (max 120 chars)
     *
     * Call this for:
     * - Thermal faults
     * - Emergency shutdowns
     * - Sensor failures
     * - Watchdog resets (if detectable)
     * - Any safety-critical state transition
     */
    static void logCritical(const char *message);

    /**
     * @brief Log with additional context
     * @param category Event category (e.g., "FAULT", "SHUTDOWN")
     * @param message Event details
     */
    static void logCritical(const char *category, const char *message);

    /**
     * @brief Dump all crash logs to Serial
     */
    static void dumpToSerial();

    /**
     * @brief Clear all crash logs
     * @note Reserved for future serial command interface
     */
    static void clear();

    /**
     * @brief Check if SPIFFS is available
     */
    static bool isAvailable() { return _initialized; }

    /**
     * @brief Log the reason for the last reset
     * Call this during startup to record watchdog/brownout resets.
     */
    static void logResetReason();

  private:
    static bool _initialized;
    static const char *CRASH_LOG_PATH;
    static const size_t MAX_LOG_SIZE;
    static const size_t MAX_ENTRY_SIZE;

    /**
     * @brief Rotate log file if it exceeds maximum size
     */
    static void rotateIfNeeded();

    /**
     * @brief Get human-readable reset reason
     */
    static const char *getResetReasonString();
};

#endif // CRASH_LOG_H
