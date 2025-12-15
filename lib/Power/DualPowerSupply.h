/**
 * @file DualPowerSupply.h
 * @brief Symmetric dual DPS5015 power supply controller
 *
 * This wrapper owns and manages two DPS5015 units, enforcing symmetric control.
 * All operations apply to both PSUs together - if one fails, both are affected.
 *
 * RESPONSIBILITIES:
 * - Owning and initializing both DPS5015 instances
 * - Symmetric current/voltage control (both channels always together)
 * - Manual override detection (consolidated from DPS5015 mismatch data)
 * - Provides status signals used by controller for fail-one-kill-both policy
 * - Emergency shutdown with controlled ramp-down
 * - Connection state tracking
 *
 * USAGE:
 * ------
 * DualPowerSupply dps(logger);
 * dps.begin();  // Initializes both PSUs with correct pins
 *
 * // In update loop:
 * dps.update();  // Updates both PSUs
 *
 * // Control:
 * dps.setSymmetricCurrent(5.0f);  // Sets both to 5A
 * dps.enableOutput();
 * dps.disableOutput();
 *
 * // Status:
 * if (dps.areBothConnected()) { ... }
 * if (dps.checkOverrideDetail().cause != OverrideCause::NONE) { ... }
 */

#ifndef DUAL_POWER_SUPPLY_H
#define DUAL_POWER_SUPPLY_H

#include "DPS5015.h"
#include "Logger.h"
#include "Counter.h"
#include "ThermalConstants.h"
#include "config.h"

/**
 * @brief Result of self-test execution
 */
enum class SelfTestResult {
    IN_PROGRESS, // Test still running, call again
    PASSED,      // Both PSUs verified working
    FAILED       // Test failed (see logs for details)
};

enum class OverrideCause : uint8_t {
    NONE = 0,
    HUMAN_OVERRIDE,
    CONTROL_MISMATCH
};

struct OverrideInfo {
    OverrideCause cause = OverrideCause::NONE;
    char reason[64] = "";
};

class DualPowerSupply {
  public:
    explicit DualPowerSupply(Logger &logger);

    /**
     * @brief Initialize both PSUs with their hardware pins
     *
     * Calls begin() on both DPS5015 instances with the correct
     * RX/TX pins from config.h.
     */
    void begin();

    /**
     * @brief Update both PSUs (call in main loop)
     *
     * Calls update() on both DPS5015 instances to process
     * Modbus communication.
     */
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
     *
     * Note: This method has side effects (updates consecutive mismatch counter)
     * so it is intentionally non-const.
     */
    OverrideInfo checkOverrideDetail();

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
     * @brief Get actual voltage from specific channel
     */
    float getOutputVoltage(size_t channel) const;

    /**
     * @brief Get actual power from specific channel
     */
    float getOutputPower(size_t channel) const;

    /**
     * @brief Check if specific PSU is connected
     */
    bool isConnected(size_t channel) const;

    /**
     * @brief Check if specific PSU has output on
     */
    bool isOutputOn(size_t channel) const;

    /**
     * @brief Reset manual override counter
     *
     * Call this immediately after intentionally changing current to prevent
     * false override detection. The counter tracks consecutive mismatches
     * between commanded and actual values - an intentional change will cause
     * a temporary mismatch that should not trigger override detection.
     */
    void resetOverrideCounter() { _consecutive_mismatches.reset(); }

    /**
     * @brief Check if any PSU has a mismatch between commanded and actual
     * values
     *
     * Used for immediate "pre-flight" checks before automated adjustments.
     * This is a raw check that ignores grace periods - caller must ensure
     * PSUs are settled before calling this.
     *
     * @return true if any mismatch detected on any connected PSU
     */
    bool hasAnyMismatch() const;

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
    DPS5015 _psu_storage[2]; // Storage for PSU objects
    DPS5015 *_psus[2];       // Array of pointers for uniform access

    // Target setpoints (commanded values)
    float _target_current;
    bool _target_output;

    // Timing for override classification
    unsigned long _last_command_ms = 0;
    unsigned long _last_measure_ms = 0;
    uint16_t _last_measure_mA = 0;

    // Emergency shutdown state
    bool _shutdown_in_progress;
    float _shutdown_current;
    unsigned long _last_shutdown_step_time;

    // Manual override tracking
    Counter _consecutive_mismatches;

    // Imbalance logging rate limiting
    unsigned long _last_imbalance_log_time = 0;
    unsigned long _last_setpoint_change_time = 0; // For imbalance suppression
    static constexpr unsigned long IMBALANCE_SETTLE_MS = 3000; // 3s settle time

    // Self-test state
    int _selftest_phase = 0;
    unsigned long _selftest_phase_start = 0;
    bool _selftest_passed[2] = {false, false};

    // Override confirmation count
    static constexpr int OVERRIDE_CONFIRM_COUNT =
        Tuning::MANUAL_OVERRIDE_MISMATCH_COUNT;

    // Self-test phases
    enum class SelfTestPhase {
        START = 0,
        WAIT_WRITES = 1, // Wait for pending writes to complete
        CHECK_SETTINGS = 2,
        ENABLE_PSU0 = 3,
        VERIFY_PSU0 = 4,
        ENABLE_PSU1 = 5,
        VERIFY_PSU1 = 6,
        COMPLETE = 7
    };

    // Self-test configuration
    // ST_SETTLE_MS must exceed DPS5015_UPDATE_INTERVAL_MS to ensure
    // at least one Modbus read cycle has occurred before checking values
    static constexpr unsigned long ST_SETTLE_MS = 750;
    static constexpr unsigned long ST_TIMEOUT_MS = 3000;
    static constexpr float ST_VOLTAGE =
        Limits::TEC_VOLTAGE_SETPOINT;         // Use actual TEC voltage for test
    static constexpr float ST_CURRENT = 0.1f; // Safe test current (no load)
    static constexpr float ST_VOLTAGE_TOLERANCE = 0.5f;
};

#endif // DUAL_POWER_SUPPLY_H
