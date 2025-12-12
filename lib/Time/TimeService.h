/**
 * @file TimeService.h
 * @brief One-shot WiFi + NTP time synchronization and ISO8601 timestamps
 *
 * TimeService provides:
 * - A bounded, boot-time attempt to sync wall clock time via WiFi + NTP
 * - A validity flag indicating whether wall time is available
 * - Formatting of current wall time as an ISO8601 UTC string
 *
 * If sync fails or WiFi is not available, wall time remains invalid and
 * callers should fall back to existing log formats.
 */

#ifndef TIME_SERVICE_H
#define TIME_SERVICE_H

#include <Arduino.h>

using TimeLogCallback = void (*)(const char *message, bool serialOnly);

namespace TimeService {

/**
 * @brief Attempt to sync time using home WiFi + NTP.
 *
 * Uses WIFI_SSID / WIFI_PASSWORD from env.h when available.
 * Bounded by timeouts in config.h.
 *
 * @return true if wall time became valid, false otherwise.
 */
/**
 * @param logCb Optional logging sink. If provided, this function will log
 *        "Time sync OK" or "Time sync skipped/failed" itself via the callback.
 *        If nullptr, it will remain silent.
 */
bool trySyncFromWifi(TimeLogCallback logCb = nullptr);

/**
 * @brief Whether wall-clock time is valid.
 */
bool isWallTimeValid();

/**
 * @brief Get current time as ISO8601 local string (YYYY-MM-DDTHH:MM:SS±HH:MM).
 *
 * Uses TIME_TZ_STRING / offsets from config.h. Returns nullptr if wall time
 * isn't valid yet.
 */
const char *getIsoTimestamp();

} // namespace TimeService

#endif // TIME_SERVICE_H
