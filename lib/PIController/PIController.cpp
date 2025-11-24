#include "PIController.h"
#include <Arduino.h>

PIController::PIController(float kp, float ki, float output_min,
                           float output_max, float integral_max)
    : _kp(kp), _ki(ki), _output_min(output_min), _output_max(output_max),
      _integral_max(integral_max), _integral(0.0f), _last_output(0.0f) {}

float PIController::compute(float error, float dt) {
    // Accumulate integral with time weighting
    _integral += error * dt;

    // Apply anti-windup by clamping integral term
    _integral = constrain(_integral, -_integral_max, _integral_max);

    // Compute PI output
    float output = _kp * error + _ki * _integral;

    // Add to previous output (incremental control)
    output = _last_output + output;

    // Clamp output to valid range
    output = constrain(output, _output_min, _output_max);

    _last_output = output;
    return output;
}

void PIController::reset() {
    _integral = 0.0f;
    _last_output = 0.0f;
}

void PIController::setKp(float kp) { _kp = kp; }

void PIController::setKi(float ki) { _ki = ki; }

void PIController::setOutputLimits(float min, float max) {
    _output_min = min;
    _output_max = max;

    // Re-clamp current output if it exceeds new limits
    _last_output = constrain(_last_output, _output_min, _output_max);
}

float PIController::getIntegral() const { return _integral; }

float PIController::getOutput() const { return _last_output; }
