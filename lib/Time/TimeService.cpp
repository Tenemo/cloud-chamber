/**
 * @file TimeService.cpp
 * @brief Implementation of one-shot WiFi + NTP time synchronization
 */

#include "TimeService.h"
#include "config.h"

#include <WiFi.h>
#include <time.h>

#if __has_include("env.h")
#include "env.h"
#define TIME_SERVICE_HAS_WIFI_CREDS 1
#else
#define TIME_SERVICE_HAS_WIFI_CREDS 0
#endif

namespace {
// Consider wall time valid if epoch is after 2024-01-01.
constexpr time_t VALID_EPOCH_THRESHOLD = 1704067200;

bool wall_time_valid = false;

bool waitForConnection(unsigned long timeout_ms) {
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
        delay(200);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool waitForTimeValid(unsigned long timeout_ms) {
    unsigned long start = millis();
    while ((millis() - start) < timeout_ms) {
        time_t now = time(nullptr);
        if (now > VALID_EPOCH_THRESHOLD) {
            wall_time_valid = true;
            return true;
        }
        delay(200);
    }
    return false;
}

} // namespace

namespace TimeService {

bool trySyncFromWifi(TimeLogCallback logCb) {
    auto log = [logCb](const char *msg) {
        if (logCb) {
            logCb(msg, false);
        }
    };

    log("Wi-Fi time sync...");

#if !TIME_SERVICE_HAS_WIFI_CREDS
    log("Wi-Fi time sync skipped (no env.h)");
    return false;
#else
    if (WIFI_SSID == nullptr || WIFI_SSID[0] == '\0') {
        log("Wi-Fi time sync skipped (SSID empty)");
        return false;
    }

    wall_time_valid = false;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true); // ensure clean start

    // Quick pre-flight scan to confirm SSID is visible (reduces silent NO_AP)
    int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
    bool ssid_found = false;
    int ssid_rssi = -127;
    int ssid_channel = -1;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == WIFI_SSID) {
            ssid_found = true;
            ssid_rssi = WiFi.RSSI(i);
            ssid_channel = WiFi.channel(i);
            break;
        }
    }
    WiFi.scanDelete();

    if (!ssid_found) {
        log("Wi-Fi time sync skipped (SSID not found in scan)");
        WiFi.mode(WIFI_OFF);
        return false;
    } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "Wi-Fi SSID visible (RSSI %ddBm ch%d)",
                 ssid_rssi, ssid_channel);
        log(buf);
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    if (!waitForConnection(WIFI_CONNECT_TIMEOUT_MS)) {
        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        log("Wi-Fi time sync failed (connect timeout)");
        return false;
    }

    // Sync via NTP and set timezone (rules configurable in config.h)
    if (TIME_TZ_STRING != nullptr && TIME_TZ_STRING[0] != '\0') {
        configTzTime(TIME_TZ_STRING, NTP_SERVER_1, NTP_SERVER_2);
    } else {
        configTime(TIME_GMT_OFFSET_SEC, TIME_DAYLIGHT_OFFSET_SEC, NTP_SERVER_1,
                   NTP_SERVER_2);
    }

    bool ok = waitForTimeValid(NTP_SYNC_TIMEOUT_MS);

    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);

    log(ok ? "Time sync OK" : "Time sync failed (NTP timeout)");
    return ok;
#endif
}

bool isWallTimeValid() { return wall_time_valid; }

const char *getIsoTimestamp() {
    if (!wall_time_valid) {
        return nullptr;
    }

    static char buf[24];
    time_t now = time(nullptr);
    struct tm tm_local;
    localtime_r(&now, &tm_local);

    // Format as YYYY-MM-DDTHH:MM:SS (no timezone suffix, local time)
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_local);
    return buf;
}

} // namespace TimeService
