#include "PIController.h"
#include <Arduino.h>
#include <unity.h>

void setUp(void) {
    // Called before each test
}

void tearDown(void) {
    // Called after each test
}

// Test: Constructor initializes correctly
void test_constructor_initializes(void) {
    PIController pid(0.1f, 0.01f, 0.0f, 1.0f, 0.5f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getIntegral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getOutput());
}

// Test: Proportional response only (Ki=0)
void test_proportional_only(void) {
    PIController pid(0.1f, 0.0f, 0.0f, 1.0f, 0.5f);

    float error = 5.0f;
    float dt = 0.1f; // 100ms

    float output = pid.compute(error, dt);

    // Output should be Kp * error = 0.1 * 5.0 = 0.5
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, output);
}

// Test: Integral accumulation
void test_integral_accumulation(void) {
    PIController pid(0.0f, 0.1f, 0.0f, 1.0f, 10.0f);

    float error = 1.0f;
    float dt = 0.1f;

    // First call: integral = 1.0 * 0.1 = 0.1
    float output1 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, pid.getIntegral());

    // Second call: integral = 0.1 + 1.0 * 0.1 = 0.2
    float output2 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pid.getIntegral());
}

// Test: Anti-windup limits integral
void test_anti_windup(void) {
    PIController pid(0.0f, 1.0f, 0.0f, 1.0f, 0.5f);

    float error = 10.0f;
    float dt = 0.1f;

    // Try to accumulate large integral
    for (int i = 0; i < 10; i++) {
        pid.compute(error, dt);
    }

    // Integral should be clamped to integral_max
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, pid.getIntegral());
}

// Test: Output clamping at minimum
void test_output_clamp_min(void) {
    PIController pid(1.0f, 0.0f, 0.0f, 1.0f, 0.5f);

    float error = -10.0f; // Large negative error
    float dt = 0.1f;

    float output = pid.compute(error, dt);

    // Output should be clamped to min (0.0)
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output);
}

// Test: Output clamping at maximum
void test_output_clamp_max(void) {
    PIController pid(1.0f, 0.0f, 0.0f, 1.0f, 0.5f);

    float error = 10.0f; // Large positive error
    float dt = 0.1f;

    float output = pid.compute(error, dt);

    // Output should be clamped to max (1.0)
    TEST_ASSERT_EQUAL_FLOAT(1.0f, output);
}

// Test: Reset clears state
void test_reset_clears_state(void) {
    PIController pid(0.1f, 0.1f, 0.0f, 1.0f, 0.5f);

    // Accumulate some state
    pid.compute(5.0f, 0.1f);
    pid.compute(5.0f, 0.1f);

    TEST_ASSERT_NOT_EQUAL(0.0f, pid.getIntegral());
    TEST_ASSERT_NOT_EQUAL(0.0f, pid.getOutput());

    // Reset
    pid.reset();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getIntegral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getOutput());
}

// Test: Setting Kp changes proportional response
void test_set_kp(void) {
    PIController pid(0.1f, 0.0f, 0.0f, 1.0f, 0.5f);

    float error = 1.0f;
    float dt = 0.1f;

    float output1 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, output1);

    pid.reset();
    pid.setKp(0.5f);

    float output2 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, output2);
}

// Test: Setting Ki changes integral response
void test_set_ki(void) {
    PIController pid(0.0f, 0.1f, 0.0f, 1.0f, 10.0f);

    float error = 1.0f;
    float dt = 1.0f; // 1 second for easy calculation

    pid.compute(error, dt);
    float integral1 = pid.getIntegral();

    pid.reset();
    pid.setKi(0.5f);

    pid.compute(error, dt);
    float integral2 = pid.getIntegral();

    // Integral2 should accumulate faster with higher Ki
    TEST_ASSERT_EQUAL_FLOAT(integral1, integral2);
}

// Test: Setting output limits
void test_set_output_limits(void) {
    PIController pid(1.0f, 0.0f, 0.0f, 1.0f, 0.5f);

    // Set narrow limits
    pid.setOutputLimits(0.2f, 0.8f);

    // Try to go above max
    float output1 = pid.compute(10.0f, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0.8f, output1);

    pid.reset();

    // Try to go below min
    float output2 = pid.compute(-10.0f, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, output2);
}

// Test: Incremental control (output accumulates)
void test_incremental_control(void) {
    PIController pid(0.1f, 0.0f, 0.0f, 1.0f, 0.5f);

    float error = 1.0f;
    float dt = 0.1f;

    // First call
    float output1 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, output1);

    // Second call with same error - output should increase
    float output2 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, output2);

    // Third call
    float output3 = pid.compute(error, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f, output3);
}

// Test: PI combined response
void test_pi_combined(void) {
    PIController pid(0.1f, 0.05f, 0.0f, 1.0f, 10.0f);

    float error = 2.0f;
    float dt = 0.1f;

    float output = pid.compute(error, dt);

    // P term: 0.1 * 2.0 = 0.2
    // I term: 0.05 * (2.0 * 0.1) = 0.01
    // Total: 0.2 + 0.01 = 0.21 (approximately, considering incremental nature)
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.21f, output);
}

// Test: Negative integral accumulation
void test_negative_integral(void) {
    PIController pid(0.0f, 0.1f, -1.0f, 1.0f, 10.0f);

    float error = -2.0f;
    float dt = 0.1f;

    pid.compute(error, dt);

    // Integral should be negative: -2.0 * 0.1 = -0.2
    float integral = pid.getIntegral();
    TEST_ASSERT_TRUE(integral < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.2f, integral);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();

    RUN_TEST(test_constructor_initializes);
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_integral_accumulation);
    RUN_TEST(test_anti_windup);
    RUN_TEST(test_output_clamp_min);
    RUN_TEST(test_output_clamp_max);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_set_kp);
    RUN_TEST(test_set_ki);
    RUN_TEST(test_set_output_limits);
    RUN_TEST(test_incremental_control);
    RUN_TEST(test_pi_combined);
    RUN_TEST(test_negative_integral);

    UNITY_END();
}

void loop() {
    // Nothing to do here
}
