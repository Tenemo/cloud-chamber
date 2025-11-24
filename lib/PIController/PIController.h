#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

/**
 * @brief Proportional-Integral (PI) controller for closed-loop control
 *
 * Implements a discrete-time PI controller with anti-windup protection
 * and output clamping for duty cycle control applications.
 */
class PIController {
  public:
    /**
     * @brief Construct a new PI Controller object
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param output_min Minimum output value (e.g., 0.0 for duty cycle)
     * @param output_max Maximum output value (e.g., 1.0 for duty cycle)
     * @param integral_max Anti-windup limit for integral term
     */
    PIController(float kp, float ki, float output_min, float output_max,
                 float integral_max);

    /**
     * @brief Compute control output based on error
     *
     * @param error Current error (setpoint - process_variable)
     * @param dt Time step in seconds since last update
     * @return float Control output clamped to [output_min, output_max]
     */
    float compute(float error, float dt);

    /**
     * @brief Reset the controller state (clear integral)
     */
    void reset();

    /**
     * @brief Set new proportional gain
     *
     * @param kp Proportional gain
     */
    void setKp(float kp);

    /**
     * @brief Set new integral gain
     *
     * @param ki Integral gain
     */
    void setKi(float ki);

    /**
     * @brief Set output limits
     *
     * @param min Minimum output value
     * @param max Maximum output value
     */
    void setOutputLimits(float min, float max);

    /**
     * @brief Get current integral term value
     *
     * @return float Integral accumulator value
     */
    float getIntegral() const;

    /**
     * @brief Get last computed output
     *
     * @return float Last output value
     */
    float getOutput() const;

  private:
    float _kp;
    float _ki;
    float _output_min;
    float _output_max;
    float _integral_max;
    float _integral;
    float _last_output;
};

#endif // PI_CONTROLLER_H
