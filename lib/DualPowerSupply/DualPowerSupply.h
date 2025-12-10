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
     */
    OverrideStatus checkManualOverride() const;

    /**
     * @brief Check if either PSU has its output enabled
     */
    bool isOutputOn() const;

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
     * @brief Pre-write validation: check if DPS state allows safe write
     * @param expected_current The current we're about to command
     * @return true if safe to proceed
     */
    bool validateBeforeWrite(float expected_current) const;

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

    // Constants (could be moved to config if needed)
    static constexpr float EMERGENCY_RAMP_RATE_A_PER_SEC = 2.0f;
    static constexpr unsigned long SHUTDOWN_STEP_MS = 500;
    static constexpr int OVERRIDE_CONFIRM_COUNT = 3;
};

#endif // DUAL_POWER_SUPPLY_H
