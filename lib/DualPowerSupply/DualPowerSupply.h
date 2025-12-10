/**
 * @file DualPowerSupply.h
 * @brief Symmetric dual DPS5015 power supply controller
 *
 * This wrapper enforces symmetric control of two DPS5015 units.
 * All operations apply to both PSUs together - if one fails, both are affected.
 *
 * RESPONSIBILITIES:
 * - Symmetric current/voltage control (both channels always together)
 * - Manual override detection (consolidated from DPS5015 mismatch data)
 * - Fail-one-kill-both shutdown logic
 * - Emergency shutdown with controlled ramp-down
 * - Connection state tracking
 *
 * USAGE:
 * ------
 * DualPowerSupply dps(logger, psu0, psu1);
 * dps.begin();
 *
 * // In update loop:
 * dps.update();
 *
 * // Control:
 * dps.setSymmetricCurrent(5.0f);  // Sets both to 5A
 * dps.enableOutput();
 * dps.disableOutput();
 *
 * // Status:
 * if (dps.areBothConnected()) { ... }
 * if (dps.isManualOverride()) { ... }
 */

#ifndef DUAL_POWER_SUPPLY_H
#define DUAL_POWER_SUPPLY_H

#include "DPS5015.h"
#include "Logger.h"
#include "config.h"
#include <Arduino.h>

/**
 * @brief Result of manual override check
 */
enum class OverrideStatus {
    NONE,     // No override detected
    DETECTED, // Override detected on one or both channels
    PENDING   // Mismatch detected but waiting for confirmation
};

/**
 * @brief Result of self-test execution
 */
enum class SelfTestResult {
    IN_PROGRESS, // Test still running, call again
    PASSED,      // Both PSUs verified working
    FAILED       // Test failed (see logs for details)
};

class DualPowerSupply {
  public:
    DualPowerSupply(Logger &logger, DPS5015 &psu0, DPS5015 &psu1);

    void begin();
    void update();

    // =========================================================================
    // Symmetric Control
    // =========================================================================

    /**
     * @brief Set current on both channels symmetrically
     * @param current Target current (will be clamped to valid range)
     * @return true if both commands were queued successfully
     */
    bool setSymmetricCurrent(float current);

    /**
     * @brief Set voltage on both channels symmetrically
     * @param voltage Target voltage
     * @return true if both commands were queued successfully
     */
    bool setSymmetricVoltage(float voltage);

    /**
     * @brief Enable output on both channels
     * @return true if both commands were queued successfully
     */
    bool enableOutput();

    /**
     * @brief Disable output on both channels (blocking, for emergency use)
     * @return true if both channels were disabled successfully
     */
    bool disableOutput();

    /**
     * @brief Configure both PSUs with voltage, current, and output state
     */
    void configure(float voltage, float current, bool outputOn = true);

    // =========================================================================
    // Emergency Shutdown
    // =========================================================================

    /**
     * @brief Start non-blocking emergency shutdown with controlled ramp-down
     */
    void startEmergencyShutdown();

    /**
     * @brief Update emergency shutdown state (call in update loop)
     * @return true if shutdown is still in progress
     */
    bool updateEmergencyShutdown();

    /**
     * @brief Check if shutdown is in progress
     */
    bool isShutdownInProgress() const { return _shutdown_in_progress; }

    /**
     * @brief Immediate hard shutdown (blocking, for critical faults)
     * @return true if both channels were disabled successfully
     */
    bool hardShutdown();

    // =========================================================================
    // Status Queries
    // =========================================================================

    /**
     * @brief Check if both PSUs are connected
     */
    bool areBothConnected() const;

    /**
     * @brief Check if either PSU is connected
     */
    bool isEitherConnected() const;

    /**
     * @brief Check if exactly one PSU is connected (asymmetric failure)
     */
    bool isAsymmetricFailure() const;

    /**
     * @brief Check for manual override (human changed DPS settings)
     *
     * Detects mismatches in current, voltage, or output state between
     * what we commanded and what the DPS reports. Requires consecutive
     * mismatches to avoid false positives from transient states.
     */
    OverrideStatus checkManualOverride() const;

    /**
     * @brief Check if either PSU has its output enabled
     */
    bool isOutputOn() const;

    /**
     * @brief Check if both PSUs have settled (match commanded values)
     *
     * Use this before issuing new setpoints to verify PSUs are ready.
     * A PSU is "settled" when it has processed our last command and
     * its reported values match what we commanded.
     *
     * @return true if both PSUs are settled and ready for new commands
     */
    bool areBothSettled() const;

    /**
     * @brief Get current target (what we commanded)
     */
    float getTargetCurrent() const { return _target_current; }

    /**
     * @brief Get current target voltage
     */
    float getTargetVoltage() const { return _target_voltage; }

    /**
     * @brief Get average actual output current
     */
    float getAverageOutputCurrent() const;

    /**
     * @brief Get total power (sum of both channels)
     */
    float getTotalPower() const;

    /**
     * @brief Get actual current from specific channel
     */
    float getOutputCurrent(size_t channel) const;

    /**
     * @brief Get set current from specific channel (what DPS reports)
     */
    float getSetCurrent(size_t channel) const;

    /**
     * @brief Check if specific PSU is connected
     */
    bool isConnected(size_t channel) const;

    /**
     * @brief Check if specific PSU has output on
     */
    bool isOutputOn(size_t channel) const;

    /**
     * @brief Get raw PSU reference for direct access (e.g., self-test)
     * @param channel 0 or 1
     * @return Reference to the requested PSU
     */
    DPS5015 &getPsu(size_t channel) { return (channel == 0) ? _psu0 : _psu1; }

    // =========================================================================
    // Channel Imbalance Detection
    // =========================================================================

    /**
     * @brief Check for significant current imbalance between channels
     * @return Current difference in amps (0 if not applicable)
     */
    float getCurrentImbalance() const;

    /**
     * @brief Check for significant power imbalance between channels
     * @return Power difference in watts (0 if not applicable)
     */
    float getPowerImbalance() const;

    /**
     * @brief Log imbalance warning if significant, with rate limiting
     *
     * Checks current and power imbalance between channels and logs a warning
     * if thresholds are exceeded. Rate-limited to avoid log spam.
     *
     * @param current_threshold Current imbalance threshold in amps
     * @param power_threshold Power imbalance threshold in watts
     * @param interval_ms Minimum interval between log messages
     */
    void checkAndLogImbalance(float current_threshold, float power_threshold,
                              unsigned long interval_ms);

    // =========================================================================
    // Hot Reset Detection
    // =========================================================================

    /**
     * @brief Detect if DPS was already running on boot (hot reset)
     *
     * Checks if the DPS has output enabled with significant current flow.
     * Use this during initialization to detect MCU reset while TEC was active.
     *
     * @param min_threshold Minimum current (A) to consider "running"
     * @return Adopted current if hot reset detected, 0.0 if not
     */
    float detectHotReset(float min_threshold);

    // =========================================================================
    // Self-Test
    // =========================================================================

    /**
     * @brief Run non-blocking self-test of both PSUs
     *
     * Tests voltage setting, output enable/disable for both channels.
     * Call repeatedly until it returns PASSED or FAILED.
     *
     * @return IN_PROGRESS while testing, PASSED or FAILED when complete
     */
    SelfTestResult runSelfTest();

    /**
     * @brief Reset self-test state (call before starting a new test)
     */
    void resetSelfTest();

  private:
    Logger &_logger;
    DPS5015 &_psu0;
    DPS5015 &_psu1;

    // Target setpoints (commanded values)
    float _target_current;
    float _target_voltage;
    bool _target_output;

    // Emergency shutdown state
    bool _shutdown_in_progress;
    float _shutdown_current;
    unsigned long _last_shutdown_step_time;

    // Manual override tracking
    mutable int _consecutive_mismatches;

    // Imbalance logging rate limiting
    unsigned long _last_imbalance_log_time = 0;

    // Self-test state
    int _selftest_phase = 0;
    unsigned long _selftest_phase_start = 0;
    bool _selftest_passed[2] = {false, false};

    // Override confirmation count
    static constexpr int OVERRIDE_CONFIRM_COUNT =
        MANUAL_OVERRIDE_MISMATCH_COUNT;
};

#endif // DUAL_POWER_SUPPLY_H
