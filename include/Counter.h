/**
 * @file Counter.h
 * @brief Tiny saturation counter helper used across modules
 */

#ifndef SIMPLE_COUNTER_H
#define SIMPLE_COUNTER_H

#include <cstdint>

struct Counter {
    uint16_t v = 0;
    void reset() { v = 0; }
    void inc() {
        if (v < UINT16_MAX)
            ++v;
    }
    bool atLeast(uint16_t n) const { return v >= n; }
    uint16_t value() const { return v; }

    // Convenience operator to mimic simple integral use in legacy spots
    Counter &operator=(int rhs) {
        v = static_cast<uint16_t>(rhs);
        return *this;
    }
};

#endif // SIMPLE_COUNTER_H
