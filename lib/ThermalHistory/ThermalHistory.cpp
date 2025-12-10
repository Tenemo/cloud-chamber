/**
 * @file ThermalHistory.cpp
 * @brief Implementation of thermal history buffer and trend analysis
 */

#include "ThermalHistory.h"
#include <cmath>

ThermalHistory::ThermalHistory() : _head(0), _count(0) {}

void ThermalHistory::recordSample(const ThermalSample &sample) {
    _buffer[_head] = sample;
    _head = (_head + 1) % HISTORY_BUFFER_SIZE;
    if (_count < HISTORY_BUFFER_SIZE) {
        _count++;
    }
}

bool ThermalHistory::hasMinimumHistory(size_t min_samples) const {
    return _count >= min_samples;
}

const ThermalSample *ThermalHistory::getSample(size_t samples_ago) const {
    if (samples_ago >= _count) {
        return nullptr;
    }
    size_t idx =
        (_head + HISTORY_BUFFER_SIZE - 1 - samples_ago) % HISTORY_BUFFER_SIZE;
    return &_buffer[idx];
}

void ThermalHistory::clear() {
    _head = 0;
    _count = 0;
}

float ThermalHistory::calculateSlopeKPerMin(bool use_hot_plate,
                                            size_t window_samples) const {
    if (_count < window_samples) {
        return RATE_INSUFFICIENT_HISTORY;
    }

    // Linear regression: y = mx + b
    // We compute slope m using least squares
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    size_t n = window_samples;

    for (size_t i = 0; i < n; i++) {
        size_t idx =
            (_head + HISTORY_BUFFER_SIZE - n + i) % HISTORY_BUFFER_SIZE;
        float x = static_cast<float>(i);
        float y = use_hot_plate ? _buffer[idx].hot_plate_temp
                                : _buffer[idx].cold_plate_temp;

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (fabs(denom) < 0.0001f) {
        return 0.0f;
    }

    // Slope in degrees per sample
    float slope = (n * sum_xy - sum_x * sum_y) / denom;

    // Convert to K/min (samples are 1 second apart per
    // HISTORY_SAMPLE_INTERVAL_MS)
    return slope * 60.0f;
}

float ThermalHistory::getColdPlateRate() const {
    return calculateSlopeKPerMin(false, COOLING_RATE_WINDOW_SAMPLES);
}

float ThermalHistory::getHotPlateRate() const {
    return calculateSlopeKPerMin(true, HOT_SIDE_RATE_SAMPLE_WINDOW_SAMPLES);
}
