/**
 * @file ThermalHistory.h
 * @brief Circular buffer for thermal data with trend analysis
 *
 * Manages a rolling history of temperature samples and provides
 * rate-of-change calculations using linear regression.
 */

#ifndef THERMAL_HISTORY_H
#define THERMAL_HISTORY_H

#include "config.h"
#include <Arduino.h>

// Trend classification for analysis
enum class ThermalTrend {
    COOLING,  // Sustained negative temperature slope
    WARMING,  // Sustained positive slope
    STABLE,   // Near-zero slope with low variance
    ANOMALOUS // Insufficient history for classification
};

// History sample record
struct ThermalSample {
    unsigned long timestamp_ms;
    float cold_plate_temp;
    float hot_plate_temp;
    float ambient_temp;
    float current_setpoint_ch1;
    float current_setpoint_ch2;
    float actual_current_ch1;
    float actual_current_ch2;
    float power_ch1;
    float power_ch2;
};

/**
 * @brief Circular buffer for thermal history with trend analysis
 *
 * Records temperature samples at regular intervals and provides
 * linear regression-based rate calculations for control decisions.
 */
class ThermalHistory {
  public:
    ThermalHistory();

    /**
     * @brief Record a new sample to the history buffer
     * @param sample The thermal sample to record
     */
    void recordSample(const ThermalSample &sample);

    /**
     * @brief Get the cold plate cooling rate
     * @return Rate in K/min (negative = cooling, positive = warming)
     */
    float getColdPlateRate() const;

    /**
     * @brief Get the hot plate rate of change
     * @return Rate in K/min, or RATE_INSUFFICIENT_HISTORY if not enough data
     */
    float getHotPlateRate() const;

    /**
     * @brief Analyze the current thermal trend
     * @return ThermalTrend classification
     */
    ThermalTrend analyzeTrend() const;

    /**
     * @brief Check if enough history exists for reliable rate calculation
     * @param min_samples Minimum samples required
     * @return true if sufficient history exists
     */
    bool hasMinimumHistory(size_t min_samples) const;

    /**
     * @brief Get the number of samples in the buffer
     */
    size_t getCount() const { return _count; }

    /**
     * @brief Get a sample from the buffer by relative index
     * @param samples_ago 0 = most recent, 1 = second most recent, etc.
     * @return Pointer to sample or nullptr if index out of range
     */
    const ThermalSample *getSample(size_t samples_ago) const;

    /**
     * @brief Clear all history data
     */
    void clear();

    // Sentinel value returned when insufficient history for rate calculation
    static constexpr float RATE_INSUFFICIENT_HISTORY = 99.0f;

  private:
    ThermalSample _buffer[HISTORY_BUFFER_SIZE];
    size_t _head;  // Next write position
    size_t _count; // Number of valid samples

    /**
     * @brief Calculate slope using linear regression
     * @param use_hot_plate true for hot plate, false for cold plate
     * @param window_samples Number of samples to use
     * @return Slope in K/min
     */
    float calculateSlopeKPerMin(bool use_hot_plate,
                                size_t window_samples) const;
};

#endif // THERMAL_HISTORY_H
