/**
 * @file CrashLog.h
 * @brief Crash logging stub (serial-only, no persistence)
 *
 * USAGE:
 * ------
 * Functions remain for compatibility but now only print to Serial.
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
     * @brief Get human-readable reset reason
     */
    static const char *getResetReasonString();
};

#endif // CRASH_LOG_H
