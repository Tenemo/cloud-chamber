/**
 * @file Benchmark.h
 * @brief Fixed-current benchmark sequencing (multi-run)
 *
 * Runs a sequence of fixed-current profiles:
 *  - Ramp up to target current
 *  - Hold for a fixed duration
 *  - Ramp down to 0A and turn outputs OFF
 *  - Wait for cold plate to warm back up (e.g. 20C) before the next run
 *
 * This logic is kept separate from ThermalController optimization.
 */

#ifndef BENCHMARK_H
#define BENCHMARK_H

#include "DualPowerSupply.h"
#include "Logger.h"
#include "ThermalConstants.h"
#include "ThermalOptimizer.h" // ThermalSnapshot
#include "ThermalTypes.h"
#include <cstddef>

class Benchmark {
  public:
    explicit Benchmark(Logger &logger);

    /**
     * @brief Enable and configure benchmark sequence
     *
     * @param currents_a Pointer to array of target currents (A per channel).
     *        A value of 0.0f is allowed and represents a "0A run".
     * @param count Number of entries in currents_a
     * @param hold_ms Hold duration at each target current (ms)
     * @param warmup_temp_c Cold plate temperature that must be reached before
     *        starting the next run (C)
     */
    void begin(const float *currents_a, size_t count, unsigned long hold_ms,
               float warmup_temp_c = 20.0f);

    bool isEnabled() const { return _enabled; }

    struct StepResult {
        bool should_transition = false;
        ThermalState next_state = ThermalState::STEADY_STATE;
        const char *reason = nullptr;
    };

    /**
     * @brief Run one benchmark update step.
     *
     * Called by ThermalController from its RAMP_UP and STEADY_STATE handlers.
     *
     * @param controller_state Current ThermalController state
     * @param snapshot Current system snapshot
     * @param dps Dual power supply reference
     * @param last_adjustment_time Controller-owned adjustment timestamp (ms)
     */
    StepResult update(ThermalState controller_state,
                      const ThermalSnapshot &snapshot, DualPowerSupply &dps,
                      unsigned long &last_adjustment_time);

  private:
    enum class Phase {
        RAMP_UP,
        HOLD,
        RAMP_DOWN,
        WAIT_WARMUP,
        DONE,
    };

    Logger &_logger;

    const float *_currents_a = nullptr;
    size_t _count = 0;
    size_t _index = 0;
    float _target_a = 0.0f;

    unsigned long _hold_ms = 0;
    float _warmup_temp_c = 20.0f;

    bool _enabled = false;
    Phase _phase = Phase::DONE;

    // Per-run tracking
    unsigned long _hold_start_time = 0;
    unsigned long _rampdown_last_step_time = 0;
    bool _run_started_logged = false;
    bool _idle_off_configured = false;

    char _reason_buf[64] = "";

    void startRun(size_t index);
    void markDone();
};

#endif // BENCHMARK_H
