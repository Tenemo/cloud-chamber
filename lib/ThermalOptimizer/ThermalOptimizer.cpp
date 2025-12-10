/**
 * @file ThermalOptimizer.cpp
 * @brief Implementation of hill-climbing current optimizer
 */

#include "ThermalOptimizer.h"
#include <cmath>

ThermalOptimizer::ThermalOptimizer(Logger &logger)
    : _logger(logger), _optimal_current(0.0f), _temp_at_optimal(100.0f),
      _baseline_current(0.0f), _baseline_temp(100.0f), _baseline_rate(0.0f),
      _awaiting_evaluation(false), _eval_start_time(0), _probe_direction(1),
      _current_step(COARSE_STEP_A), _consecutive_bounces(0), _converged(false) {
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
    _current_step = COARSE_STEP_A;
    _consecutive_bounces = 0;
    _converged = false;
}

void ThermalOptimizer::seedWithCurrent(float current, float temp) {
    _optimal_current = current;
    _temp_at_optimal = temp;
    _baseline_current = current;
    _baseline_temp = temp;
}

// =============================================================================
// Main Update Entry Point
// =============================================================================

ThermalOptimizationDecision ThermalOptimizer::update(
    const ThermalSnapshot &snapshot, ThermalControlPhase phase,
    unsigned long last_adjustment_time, unsigned long adjustment_interval_ms,
    bool psus_ready) {

    ThermalOptimizationDecision no_change = {
        false, 0.0f, false, false, false, nullptr};

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
    // At maximum current
    if (snapshot.current_setpoint >= MAX_CURRENT_PER_CHANNEL)
        return true;

    // Hot side limiting
    if (snapshot.hot_in_alarm)
        return true;

    // Cooling stalled (only if we have enough history)
    if (has_enough_history) {
        if (fabs(snapshot.cooling_rate) < COOLING_STALL_THRESHOLD_C)
            return true;
    }

    return false;
}

// =============================================================================
// Evaluation Logic
// =============================================================================

ThermalEvaluationResult ThermalOptimizer::evaluateEffect(
    const ThermalSnapshot &snapshot) {

    // Minimum delay before evaluation
    if (snapshot.now - _eval_start_time < CURRENT_EVALUATION_DELAY_MS)
        return ThermalEvaluationResult::WAITING;

    // Wait for hot-side stabilization
    bool timeout =
        (snapshot.now - _eval_start_time > HOT_SIDE_STABILIZATION_MAX_WAIT_MS);

    if (!snapshot.hot_side_stable && !timeout)
        return ThermalEvaluationResult::WAITING;

    // Compare against baseline
    float rate_delta = snapshot.cooling_rate - _baseline_rate;

    bool improved = (snapshot.cold_temp <
                     _baseline_temp - Tolerance::TEMP_IMPROVEMENT_THRESHOLD_C);
    bool degraded = (rate_delta > COOLING_RATE_DEGRADATION_THRESHOLD);
    bool worsened =
        (snapshot.cold_temp > _baseline_temp + OVERCURRENT_WARMING_THRESHOLD_C);

    if (improved && !degraded)
        return ThermalEvaluationResult::IMPROVED;
    if (worsened || degraded)
        return ThermalEvaluationResult::DEGRADED;
    return ThermalEvaluationResult::UNCHANGED;
}

ThermalOptimizationDecision ThermalOptimizer::processEvaluation(
    const ThermalSnapshot &snapshot, ThermalControlPhase phase) {

    ThermalOptimizationDecision decision = {
        false, 0.0f, false, false, false, nullptr};

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
        _consecutive_bounces = 0;
        _converged = false;

        snprintf(_log_buffer, sizeof(_log_buffer), "TC: Opt %.1fA=%.1fC",
                 snapshot.current_setpoint, snapshot.cold_temp);
        decision.log_message = _log_buffer;

    } else if (result == ThermalEvaluationResult::DEGRADED) {
        // Revert to baseline (local), not global best
        float revert_current = _baseline_current;
        if (revert_current < MIN_CURRENT_PER_CHANNEL) {
            revert_current = _optimal_current;
        }
        revert_current = Clamp::current(revert_current);

        decision.change_current = true;
        decision.new_current = revert_current;

        _optimal_current = revert_current;
        _temp_at_optimal = _baseline_temp;

        // Bounce: flip direction and track
        _probe_direction *= -1;
        _consecutive_bounces++;

        // Check for convergence (tried both directions)
        if (_consecutive_bounces >= 2) {
            _converged = true;
            decision.converged = true;

            snprintf(_log_buffer, sizeof(_log_buffer),
                     "TC: Converged at %.1fA (%.1fC)", _optimal_current,
                     _temp_at_optimal);
            decision.log_message = _log_buffer;
        } else {
            snprintf(_log_buffer, sizeof(_log_buffer),
                     "TC: Revert %.1fA (bounce #%d)", revert_current,
                     _consecutive_bounces);
            decision.log_message = _log_buffer;
        }

        // Signal transition to steady state if bounced during ramp
        if (phase == ThermalControlPhase::RAMP_UP) {
            decision.should_transition = true;
        }
    }
    // UNCHANGED: no action needed

    return decision;
}

// =============================================================================
// Step Size Calculation
// =============================================================================

float ThermalOptimizer::calculateStepSize(float cooling_rate,
                                          bool is_hot_side_warning) const {
    float rate_magnitude = fabs(cooling_rate);

    // Hot-side in warning zone: cap at medium step regardless of rate
    if (is_hot_side_warning) {
        if (_consecutive_bounces >= 2) {
            return FINE_STEP_A;
        }
        return MEDIUM_STEP_A;
    }

    // If we've bounced multiple times, use finer steps
    if (_consecutive_bounces >= 2) {
        return FINE_STEP_A;
    }

    // Select based on cooling rate magnitude
    // Note: If insufficient history, rate may be large (RATE_INSUFFICIENT_HISTORY)
    // which correctly biases towards COARSE_STEP_A during early ramp
    if (rate_magnitude > STEP_COARSE_RATE_THRESHOLD) {
        return COARSE_STEP_A;
    } else if (rate_magnitude > STEP_MEDIUM_RATE_THRESHOLD) {
        return MEDIUM_STEP_A;
    } else {
        return FINE_STEP_A;
    }
}

// =============================================================================
// RAMP_UP Mode
// =============================================================================

ThermalOptimizationDecision ThermalOptimizer::attemptRampStep(
    const ThermalSnapshot &snapshot) {

    ThermalOptimizationDecision decision = {
        false, 0.0f, false, false, false, nullptr};

    float current = snapshot.current_setpoint;
    float step = calculateStepSize(snapshot.cooling_rate, snapshot.hot_in_warning);
    _current_step = step;

    // Record baseline before step
    _baseline_current = current;
    _baseline_temp = snapshot.cold_temp;
    _baseline_rate = snapshot.cooling_rate;

    // Increase current (ramp is always upward)
    float new_current = fmin(current + step, MAX_CURRENT_PER_CHANNEL);

    decision.change_current = true;
    decision.new_current = new_current;

    _awaiting_evaluation = true;
    _eval_start_time = snapshot.now;

    snprintf(_log_buffer, sizeof(_log_buffer), "TC: Ramp %.1fA (+%.2fA)",
             new_current, step);
    decision.log_message = _log_buffer;

    return decision;
}

// =============================================================================
// STEADY_STATE Mode
// =============================================================================

ThermalOptimizationDecision ThermalOptimizer::attemptSteadyProbe(
    const ThermalSnapshot &snapshot) {

    ThermalOptimizationDecision decision = {
        false, 0.0f, false, false, false, nullptr};

    float current = snapshot.current_setpoint;
    float step = calculateStepSize(snapshot.cooling_rate, snapshot.hot_in_warning);
    _current_step = step;

    // Record baseline before step
    _baseline_current = current;
    _baseline_temp = snapshot.cold_temp;
    _baseline_rate = snapshot.cooling_rate;

    // Calculate target current based on probe direction
    float new_current = 0.0f;
    bool can_step = false;

    if (_probe_direction > 0) {
        // Try increasing (if we have room and thermal headroom)
        if (current < MAX_CURRENT_PER_CHANNEL &&
            snapshot.hot_temp < HOT_SIDE_WARNING_C - HOT_SIDE_PROBE_HEADROOM_C) {
            new_current = fmin(current + step, MAX_CURRENT_PER_CHANNEL);
            can_step = true;
        }
    } else {
        // Try decreasing (if we have room)
        if (current > STARTUP_CURRENT + step) {
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

    // Flip direction for next probe (alternate by default)
    _probe_direction *= -1;

    const char *dir_str = (_probe_direction < 0) ? "up" : "down"; // Already flipped
    snprintf(_log_buffer, sizeof(_log_buffer), "TC: Probe %s %.1fA (%+.2fA)",
             dir_str, new_current, new_current - current);
    decision.log_message = _log_buffer;

    return decision;
}
