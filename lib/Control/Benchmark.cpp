/**
 * @file Benchmark.cpp
 * @brief Fixed-current benchmark sequencing implementation
 */

#include "Benchmark.h"
#include <cmath>
#include <cstdio>

Benchmark::Benchmark(Logger &logger) : _logger(logger) {}

void Benchmark::begin(const float *currents_a, size_t count,
                      unsigned long hold_ms, float warmup_temp_c) {
    _currents_a = currents_a;
    _count = count;
    _hold_ms = hold_ms;
    _warmup_temp_c = warmup_temp_c;

    _enabled = (_currents_a != nullptr) && (_count > 0);
    _idle_off_configured = false;
    _reason_buf[0] = '\0';

    if (!_enabled) {
        _phase = Phase::DONE;
        _index = 0;
        _target_a = 0.0f;
        _hold_start_time = 0;
        _rampdown_last_step_time = 0;
        _run_started_logged = false;
        return;
    }

    const float hold_minutes = static_cast<float>(_hold_ms) / 60000.0f;
    _logger.logf(true, "BENCH: %u runs, hold %.1f min, warm >= %.1fC",
                 static_cast<unsigned>(_count), hold_minutes,
                 static_cast<double>(_warmup_temp_c));

    startRun(0);
}

void Benchmark::startRun(size_t index) {
    _index = index;
    // Allow explicit "0A run" targets. For all non-zero values we still clamp
    // to the normal operational range (MIN..MAX).
    _target_a = _currents_a[_index];
    if (_target_a <= 0.0f) {
        _target_a = 0.0f;
    } else {
        _target_a = Clamp::current(_target_a);
    }
    _phase = Phase::RAMP_UP;

    _hold_start_time = 0;
    _rampdown_last_step_time = 0;
    _run_started_logged = false;
    _idle_off_configured = false;
}

void Benchmark::markDone() {
    _phase = Phase::DONE;
    _target_a = 0.0f;
    _hold_start_time = 0;
    _rampdown_last_step_time = 0;
    _run_started_logged = false;
}

Benchmark::StepResult
Benchmark::update(ThermalState controller_state, const ThermalSnapshot &snapshot,
                  DualPowerSupply &dps, unsigned long &last_adjustment_time) {
    StepResult result{};
    if (!_enabled) {
        return result;
    }

    // Defensive: if configuration is invalid, stop.
    if (_currents_a == nullptr || _count == 0) {
        markDone();
        return result;
    }

    // If we ran out of targets, keep outputs at 0A indefinitely.
    if (_index >= _count) {
        markDone();
    }

    constexpr float TARGET_EPS_A = 0.05f;
    constexpr unsigned long RAMPDOWN_STEP_MS = 1000;
    constexpr float RAMPDOWN_RATE_A_PER_SEC = 0.2f;

    auto psusReady = [&]() -> bool {
        return !dps.isShutdownInProgress() && dps.areBothSettled();
    };

    auto requestManualOverride = [&]() -> StepResult {
        _logger.log("TC: Pre-flight mismatch detected!");
        StepResult out{};
        out.should_transition = true;
        out.next_state = ThermalState::MANUAL_OVERRIDE;
        out.reason = "Pre-flight check";
        return out;
    };

    auto ensureOutputsOffOnce = [&]() {
        if (_idle_off_configured)
            return;
        dps.configure(Limits::TEC_VOLTAGE_SETPOINT, 0.0f, false);
        _idle_off_configured = true;
    };

    if (_phase == Phase::DONE) {
        ensureOutputsOffOnce();
        return result;
    }

    // Current target for active run (per-channel)
    const float target = _target_a;
    const bool allow_zero_target = (target <= 0.0f);

    switch (_phase) {
    case Phase::RAMP_UP:
    {
        // Ensure we are in controller RAMP_UP for ramping. If not, request it.
        if (controller_state != ThermalState::RAMP_UP) {
            result.should_transition = true;
            result.next_state = ThermalState::RAMP_UP;
            result.reason = "Benchmark ramp";
            return result;
        }

        if (!_run_started_logged) {
            _logger.logf(true, "BENCH: Run %u/%u target %.2fA",
                         static_cast<unsigned>(_index + 1),
                         static_cast<unsigned>(_count), target);
            _run_started_logged = true;
        }

        // If outputs are OFF (e.g., after prior run), re-enable at 0A and wait
        // for the PSUs to settle before ramping.
        if (!dps.isOutputOn()) {
            dps.configure(Limits::TEC_VOLTAGE_SETPOINT, 0.0f, true);
            last_adjustment_time = 0;
            return result;
        }

        const float current = snapshot.current_setpoint;
        const float delta = target - current;

        if (fabs(delta) <= TARGET_EPS_A) {
            _phase = Phase::HOLD;
            _hold_start_time = 0;
            _idle_off_configured = false;
            result.should_transition = true;
            result.next_state = ThermalState::STEADY_STATE;
            result.reason = "Bench target reached";
            return result;
        }

        if (!psusReady()) {
            return result;
        }

        if ((snapshot.now - last_adjustment_time) <
            Timing::RAMP_ADJUSTMENT_INTERVAL_MS) {
            return result;
        }

        if (dps.hasAnyMismatch()) {
            return requestManualOverride();
        }

        const float step = (fabs(delta) >= Tuning::COARSE_STEP_A)
                               ? Tuning::COARSE_STEP_A
                               : Tuning::FINE_STEP_A;
        float new_current = current + ((delta > 0.0f) ? step : -step);
        if ((delta > 0.0f && new_current > target) ||
            (delta < 0.0f && new_current < target)) {
            new_current = target;
        }
        if (allow_zero_target) {
            new_current = fmaxf(0.0f, fminf(new_current, Limits::MAX_CURRENT_PER_CHANNEL));
            dps.setSymmetricCurrentAllowZero(new_current);
        } else {
            new_current = Clamp::current(new_current);
            dps.setSymmetricCurrent(new_current);
        }
        dps.resetOverrideCounter();
        last_adjustment_time = snapshot.now;
        _logger.logf(true, "BENCH: Ramp %.2fA", new_current);
        return result;
    }

    case Phase::HOLD:
    {
        // Hold phase runs in STEADY_STATE.
        if (controller_state != ThermalState::STEADY_STATE) {
            result.should_transition = true;
            result.next_state = ThermalState::STEADY_STATE;
            result.reason = "Benchmark hold";
            return result;
        }

        const bool ready = psusReady();

        // Keep PSU target pinned to the requested current (best-effort).
        if (ready &&
            fabs(snapshot.current_setpoint - target) > TARGET_EPS_A &&
            (snapshot.now - last_adjustment_time) >=
                Timing::RAMP_ADJUSTMENT_INTERVAL_MS) {
            if (dps.hasAnyMismatch()) {
                return requestManualOverride();
            }
            if (allow_zero_target) {
                dps.setSymmetricCurrentAllowZero(target);
            } else {
                dps.setSymmetricCurrent(target);
            }
            dps.resetOverrideCounter();
            last_adjustment_time = snapshot.now;
            _logger.logf(true, "BENCH: Hold %.2fA", target);
        }

        // Start hold timer once we're at the requested current.
        if (_hold_ms > 0) {
            if (_hold_start_time == 0) {
                if (fabs(snapshot.current_setpoint - target) <= TARGET_EPS_A) {
                    _hold_start_time = snapshot.now;
                    const float hold_minutes =
                        static_cast<float>(_hold_ms) / 60000.0f;
                    _logger.logf(true, "BENCH: Hold start %.1f min", hold_minutes);
                }
                return result;
            }

            if ((snapshot.now - _hold_start_time) < _hold_ms) {
                return result;
            }
        }

        const float hold_minutes = static_cast<float>(_hold_ms) / 60000.0f;
        _logger.logf(true, "BENCH: Hold complete %.1f min", hold_minutes);
        _logger.log("BENCH: Hold done, ramping down", true);
        _phase = Phase::RAMP_DOWN;
        _rampdown_last_step_time = snapshot.now;
        return result;
    }

    case Phase::RAMP_DOWN:
    {
        if (controller_state != ThermalState::STEADY_STATE) {
            result.should_transition = true;
            result.next_state = ThermalState::STEADY_STATE;
            result.reason = "Benchmark rampdown";
            return result;
        }

        if (!psusReady()) {
            return result;
        }

        if ((snapshot.now - _rampdown_last_step_time) < RAMPDOWN_STEP_MS) {
            return result;
        }

        if (dps.hasAnyMismatch()) {
            return requestManualOverride();
        }

        const float step_a =
            RAMPDOWN_RATE_A_PER_SEC * (RAMPDOWN_STEP_MS / 1000.0f);
        float new_current = snapshot.current_setpoint - step_a;

        if (new_current <= 0.0f) {
            dps.configure(Limits::TEC_VOLTAGE_SETPOINT, 0.0f, false);
            _idle_off_configured = true;
            _logger.log("BENCH: Rampdown complete, outputs OFF", true);

            // Decide next action.
            if ((_index + 1) >= _count) {
                _logger.log("BENCH: Complete (holding 0.0A)", true);
                markDone();
                return result;
            }

            _phase = Phase::WAIT_WARMUP;
            _hold_start_time = 0;
            _rampdown_last_step_time = 0;
            _run_started_logged = false;
            _logger.logf(true, "BENCH: Waiting for cold plate >= %.1fC",
                         static_cast<double>(_warmup_temp_c));
            return result;
        }

        dps.setSymmetricCurrentAllowZero(new_current);
        _rampdown_last_step_time = snapshot.now;
        return result;
    }

    case Phase::WAIT_WARMUP:
    {
        if (controller_state != ThermalState::STEADY_STATE) {
            result.should_transition = true;
            result.next_state = ThermalState::STEADY_STATE;
            result.reason = "Benchmark warmup";
            return result;
        }

        ensureOutputsOffOnce();

        if (snapshot.cold_temp < _warmup_temp_c) {
            return result;
        }

        const size_t next_index = _index + 1;
        if (next_index >= _count) {
            _logger.log("BENCH: Complete (holding 0.0A)", true);
            markDone();
            return result;
        }

        startRun(next_index);
        last_adjustment_time = 0;

        snprintf(_reason_buf, sizeof(_reason_buf), "Bench next %.2fA",
                 static_cast<double>(_target_a));

        result.should_transition = true;
        result.next_state = ThermalState::RAMP_UP;
        result.reason = _reason_buf;
        return result;
    }

    default:
        break;
    }

    return result;
}
