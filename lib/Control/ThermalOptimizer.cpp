/**
 * @file ThermalOptimizer.cpp
 * @brief Implementation of hill-climbing current optimizer
 */

#include "ThermalOptimizer.h"
#include <cmath>

namespace {
constexpr float TEMP_IMPROVEMENT_THRESHOLD_C = 0.1f;
}

ThermalOptimizer::ThermalOptimizer(Logger &logger)
    : _logger(logger), _optimal_current(0.0f), _temp_at_optimal(100.0f),
      _baseline_current(0.0f), _baseline_temp(100.0f), _baseline_rate(0.0f),
      _awaiting_evaluation(false), _eval_start_time(0), _probe_direction(1),
      _consecutive_bounces(), _consecutive_degraded(), _converged(false),
      _stabilization_until(0) {
    _log_buffer[0] = '\0';
}

void ThermalOptimizer::reset() {
    _optimal_current = 0.0f;
    _temp_at_optimal = 100.0f;
    _baseline_current = 0.0f;
    _baseline_temp = 100.0f;
    _baseline_rate = 0.0f;
    _awaiting_evaluation = false;
    _eval_start_time = 0;
    _probe_direction = 1;
    _consecutive_bounces.reset();
    _consecutive_degraded.reset();
    _converged = false;
    _stabilization_until = 0;
}

void ThermalOptimizer::seedWithCurrent(float current, float temp) {
    _optimal_current = current;
    _temp_at_optimal = temp;
    _baseline_current = current;
    _baseline_temp = temp;

    // Start probing upward - we likely haven't found optimum yet
    _probe_direction = 1;

    // After hot reset, wait for system to stabilize before optimizing
    _stabilization_until = millis() + Timing::HOT_RESET_STABILIZATION_MS;
}

// =============================================================================
// Main Update Entry Point
// =============================================================================

ThermalOptimizationDecision ThermalOptimizer::update(
    const ThermalSnapshot &snapshot, ThermalControlPhase phase,
    unsigned long last_adjustment_time, unsigned long adjustment_interval_ms,
    bool psus_ready) {

    ThermalOptimizationDecision no_change = {false, 0.0f,  false,
                                             false, false, nullptr};

    // Wait for stabilization period after hot reset
    if (_stabilization_until > 0 && snapshot.now < _stabilization_until) {
        return no_change;
    }
    _stabilization_until = 0; // Clear once past

    // First, check for pending evaluation
    if (_awaiting_evaluation) {
        return processEvaluation(snapshot, phase);
    }

    // Check if enough time has passed since last adjustment
    if ((snapshot.now - last_adjustment_time) < adjustment_interval_ms) {
        return no_change;
    }

    // Check if PSUs are ready
    if (!psus_ready) {
        return no_change;
    }

    // Dispatch based on phase
    if (phase == ThermalControlPhase::RAMP_UP) {
        return attemptRampStep(snapshot);
    } else {
        return attemptSteadyProbe(snapshot);
    }
}

// =============================================================================
// Ramp Termination Check
// =============================================================================

bool ThermalOptimizer::shouldExitRamp(const ThermalSnapshot &snapshot,
                                      bool has_enough_history) const {
    // Never exit during stabilization period (e.g., after hot reset)
    // The system needs time to establish actual thermal behavior
    if (_stabilization_until > 0 && snapshot.now < _stabilization_until) {
        return false;
    }

    // At maximum current
    if (snapshot.current_setpoint >= Limits::MAX_CURRENT_PER_CHANNEL)
        return true;

    // Hot side limiting
    if (snapshot.hot_in_alarm)
        return true;

    // Cooling stalled - but only check if:
    // 1. We have enough temperature history
    // 2. We've ramped to at least MIN_CURRENT_FOR_STALL_CHECK_A
    // This prevents false "stall" detection at low currents when TECs
    // haven't started producing significant cooling yet
    if (has_enough_history &&
        snapshot.current_setpoint >= Tuning::MIN_CURRENT_FOR_STALL_CHECK_A) {
        if (fabs(snapshot.cooling_rate) < Tuning::COOLING_STALL_THRESHOLD_C)
            return true;
    }

    return false;
}

ThermalEvaluationResult
ThermalOptimizer::evaluateEffect(const ThermalSnapshot &snapshot) {
    // Minimum delay before evaluation
    if (snapshot.now - _eval_start_time < Timing::CURRENT_EVALUATION_DELAY_MS)
        return ThermalEvaluationResult::WAITING;

    // Wait for hot-side stabilization (or timeout)
    bool timeout =
        (snapshot.now - _eval_start_time >
         Timing::HOT_SIDE_STABILIZATION_MAX_WAIT_MS);
    if (!snapshot.hot_side_stable && !timeout)
        return ThermalEvaluationResult::WAITING;

    // Compare against baseline
    float rate_delta = snapshot.cooling_rate - _baseline_rate;

    bool temp_improved =
        (snapshot.cold_temp < _baseline_temp - TEMP_IMPROVEMENT_THRESHOLD_C);
    bool temp_worsened =
        (snapshot.cold_temp >
         _baseline_temp + Tuning::OVERCURRENT_WARMING_THRESHOLD_C);
    bool rate_degraded =
        (rate_delta > Tuning::COOLING_RATE_DEGRADATION_THRESHOLD);

    if (temp_improved)
        return ThermalEvaluationResult::IMPROVED;
    if (temp_worsened || rate_degraded)
        return ThermalEvaluationResult::DEGRADED;
    return ThermalEvaluationResult::UNCHANGED;
}

// =============================================================================
// Evaluation Logic
// =============================================================================

ThermalOptimizationDecision
ThermalOptimizer::processEvaluation(const ThermalSnapshot &snapshot,
                                    ThermalControlPhase phase) {

    ThermalOptimizationDecision decision = {false, 0.0f,  false,
                                            false, false, nullptr};

    // Use phase-specific evaluation logic
    ThermalEvaluationResult result = evaluateEffect(snapshot);
    if (result == ThermalEvaluationResult::WAITING) {
        return decision;
    }

    decision.evaluation_complete = true;
    _awaiting_evaluation = false;

    if (result == ThermalEvaluationResult::IMPROVED) {
        // Accept new current as session best
        _optimal_current = snapshot.current_setpoint;
        _temp_at_optimal = snapshot.cold_temp;
        _consecutive_bounces.reset();
        _consecutive_degraded.reset(); // Reset degraded counter on success
        _converged = false;
        // Direction stays the same - continue probing in this direction

        snprintf(_log_buffer, sizeof(_log_buffer), "TC: Opt %.2fA=%.2fC",
                 snapshot.current_setpoint, snapshot.cold_temp);
        decision.log_message = _log_buffer;

    } else if (result == ThermalEvaluationResult::DEGRADED) {
        _consecutive_degraded.inc();

        // During RAMP_UP: require multiple consecutive degraded AND minimum
        // current before actually reverting. This prevents premature exit.
        bool should_revert = false;
        if (phase == ThermalControlPhase::RAMP_UP) {
            // Only revert if:
            // 1. We've reached minimum ramp current, AND
            // 2. We've seen multiple consecutive degraded evaluations
            if (snapshot.current_setpoint >=
                    Tuning::MIN_RAMP_CURRENT_BEFORE_EXIT_A &&
                _consecutive_degraded.atLeast(
                    Tuning::CONSECUTIVE_DEGRADED_FOR_REVERT)) {
                should_revert = true;
            } else {
                // Not ready to revert yet - log but continue ramping
                snprintf(_log_buffer, sizeof(_log_buffer),
                         "TC: Degraded #%d at %.2fA (need %d @ %.2fA+)",
                         _consecutive_degraded, snapshot.current_setpoint,
                         Tuning::CONSECUTIVE_DEGRADED_FOR_REVERT,
                         Tuning::MIN_RAMP_CURRENT_BEFORE_EXIT_A);
                decision.log_message = _log_buffer;
                return decision;
            }
        } else {
            // STEADY_STATE: single degraded is enough to revert
            should_revert = true;
        }

        if (should_revert) {
            // Revert to baseline (local), not global best
            float revert_current = _baseline_current;
            if (revert_current < Limits::MIN_CURRENT_PER_CHANNEL) {
                revert_current = _optimal_current;
            }
            revert_current = Clamp::current(revert_current);

            decision.change_current = true;
            decision.new_current = revert_current;

            _optimal_current = revert_current;
            _temp_at_optimal = _baseline_temp;
            _consecutive_degraded.reset(); // Reset after revert

            // Bounce: flip direction and track
            _probe_direction *= -1;
            _consecutive_bounces.inc();

            // Check for convergence (tried both directions)
            if (_consecutive_bounces.atLeast(2)) {
                _converged = true;
                decision.converged = true;

                snprintf(_log_buffer, sizeof(_log_buffer),
                         "TC: Converged at %.2fA (%.2fC)", _optimal_current,
                         _temp_at_optimal);
                decision.log_message = _log_buffer;
            } else {
                snprintf(_log_buffer, sizeof(_log_buffer),
                         "TC: Revert %.2fA (bounce #%d)", revert_current,
                         static_cast<int>(_consecutive_bounces.value()));
                decision.log_message = _log_buffer;
            }

            // Signal transition to steady state if bounced during ramp
            if (phase == ThermalControlPhase::RAMP_UP) {
                decision.should_transition = true;
            }
        }
    } else {
        // UNCHANGED: Accept current setpoint as new best, flip direction to
        // explore
        _optimal_current = snapshot.current_setpoint;
        _temp_at_optimal = snapshot.cold_temp;
        _probe_direction *= -1; // Try other direction next time
    }

    return decision;
}

// =============================================================================
// Step Size Calculation
// =============================================================================

float ThermalOptimizer::calculateStepSize(
    const ThermalSnapshot &snapshot) const {
    float current = snapshot.current_setpoint;
    float rate_mag = fabs(snapshot.cooling_rate);

    // Hot-side warning caps aggressiveness
    if (snapshot.hot_in_warning) {
        return Tuning::FINE_STEP_A;
    }

    // If we've bounced multiple times, we're near optimum
    if (_consecutive_bounces.atLeast(2)) {
        return Tuning::FINE_STEP_A;
    }

    // Rate-based guidance first (when rate data is meaningful)
    if (rate_mag > Tuning::STEP_COARSE_RATE_THRESHOLD) {
        return Tuning::COARSE_STEP_A; // Still far from equilibrium
    }

    // Fallback to current-based sizing
    if (current < Tuning::RAMP_COARSE_BELOW_A) {
        return Tuning::COARSE_STEP_A; // Approach quickly
    }
    return Tuning::FINE_STEP_A; // Scan gently
}

// =============================================================================
// RAMP_UP Mode
// =============================================================================

ThermalOptimizationDecision
ThermalOptimizer::attemptRampStep(const ThermalSnapshot &snapshot) {

    ThermalOptimizationDecision decision = {false, 0.0f,  false,
                                            false, false, nullptr};

    float current = snapshot.current_setpoint;
    float step = calculateStepSize(snapshot);

    // Record baseline before step
    _baseline_current = current;
    _baseline_temp = snapshot.cold_temp;
    _baseline_rate = snapshot.cooling_rate;

    // Increase current (ramp is always upward)
    float new_current = Clamp::current(current + step);

    decision.change_current = true;
    decision.new_current = new_current;

    _awaiting_evaluation = true;
    _eval_start_time = snapshot.now;

    snprintf(_log_buffer, sizeof(_log_buffer), "TC: Ramp %.2fA (+%.2fA)",
             new_current, step);
    decision.log_message = _log_buffer;

    return decision;
}

// =============================================================================
// STEADY_STATE Mode
// =============================================================================

ThermalOptimizationDecision
ThermalOptimizer::attemptSteadyProbe(const ThermalSnapshot &snapshot) {

    ThermalOptimizationDecision decision = {false, 0.0f,  false,
                                            false, false, nullptr};

    float current = snapshot.current_setpoint;
    float step = calculateStepSize(snapshot);

    // Record baseline before step
    _baseline_current = current;
    _baseline_temp = snapshot.cold_temp;
    _baseline_rate = snapshot.cooling_rate;

    // Calculate target current based on probe direction
    float new_current = 0.0f;
    bool can_step = false;

    if (_probe_direction > 0) {
        // Try increasing (if we have room and thermal headroom)
        if (current < Limits::MAX_CURRENT_PER_CHANNEL &&
            snapshot.hot_temp <
                Limits::HOT_SIDE_WARNING_C - Limits::HOT_SIDE_PROBE_HEADROOM_C) {
            new_current = Clamp::current(current + step);
            can_step = true;
        }
    } else {
        // Try decreasing (if we have room)
        if (current > Limits::STARTUP_CURRENT + step) {
            new_current = current - step;
            can_step = true;
        }
    }

    if (!can_step) {
        return decision;
    }

    decision.change_current = true;
    decision.new_current = new_current;

    _awaiting_evaluation = true;
    _eval_start_time = snapshot.now;

    // Log current probe direction (direction changes happen in
    // processEvaluation)
    const char *dir_str = (_probe_direction > 0) ? "up" : "down";
    snprintf(_log_buffer, sizeof(_log_buffer), "TC: Probe %s %.2fA (%+.2fA)",
             dir_str, new_current, new_current - current);
    decision.log_message = _log_buffer;

    return decision;
}
