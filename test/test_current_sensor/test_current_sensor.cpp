#include "CurrentSensor.h"
#include <Arduino.h>
#include <unity.h>

// Mock ADC values for testing
static uint16_t mock_adc_value = 2048; // Mid-point for 12-bit ADC

// Override analogRead for testing
uint16_t analogRead(uint8_t pin) { return mock_adc_value; }

void setUp(void) {
    // Reset mock before each test
    mock_adc_value = 2048;
}

void tearDown(void) {
    // Clean up after each test
}

// Test: Constructor initializes correctly
void test_constructor_initializes_correctly(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.2f);

    TEST_ASSERT_FALSE(sensor.isCalibrated());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sensor.getFilteredCurrent());
}

// Test: Calibration with valid readings
void test_calibration_success(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.2f);

    // Set mock to mid-point (1.65V at 12-bit resolution)
    mock_adc_value = 2048;

    bool result = sensor.calibrate(10);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(sensor.isCalibrated());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.65f, sensor.getOffsetVoltage());
}

// Test: Calibration fails with invalid readings (too low)
void test_calibration_fails_too_low(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.2f);

    // Set mock to very low value (0.05V)
    mock_adc_value = 62; // ~0.05V

    bool result = sensor.calibrate(10);

    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_FALSE(sensor.isCalibrated());
}

// Test: Calibration fails with invalid readings (too high)
void test_calibration_fails_too_high(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.2f);

    // Set mock to very high value (3.25V)
    mock_adc_value = 4030; // ~3.25V

    bool result = sensor.calibrate(10);

    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_FALSE(sensor.isCalibrated());
}

// Test: Reading current before calibration returns zero
void test_read_current_not_calibrated(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.2f);

    float current = sensor.readCurrent(10);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, current);
}

// Test: Reading current after calibration
void test_read_current_after_calibration(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f,
                         1.0f); // Alpha=1.0 for no filtering

    // Calibrate at mid-point (1.65V)
    mock_adc_value = 2048;
    sensor.calibrate(10);

    // Simulate 3A current: V = 1.65V + (3A * 0.026V/A) = 1.728V
    // ADC value = 1.728V * 4095 / 3.3V = 2144
    mock_adc_value = 2144;

    float current = sensor.readCurrent(10);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.0f, current);
}

// Test: Filter smooths readings
void test_filter_smoothing(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.5f); // Alpha=0.5

    // Calibrate
    mock_adc_value = 2048;
    sensor.calibrate(10);

    // First reading at 0A
    mock_adc_value = 2048;
    float current1 = sensor.readCurrent(10);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, current1);

    // Sudden jump to ~2A
    mock_adc_value = 2113; // 1.65V + 0.052V
    float current2 = sensor.readCurrent(10);

    // With alpha=0.5, filtered value should be between 0 and 2A
    // Raw is ~2A, filtered with alpha=0.5 from 0A: 0.5*2 + 0.5*0 = 1A
    TEST_ASSERT_TRUE(current2 > 0.5f);
    TEST_ASSERT_TRUE(current2 < 1.5f);

    // Third reading should converge closer to 2A
    float current3 = sensor.readCurrent(10);
    // Now: 0.5*2 + 0.5*1 = 1.5A
    TEST_ASSERT_TRUE(current3 >= current2);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.5f, current3);
}

// Test: Reset filter clears state
void test_reset_filter(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 0.5f);

    // Calibrate and read some current
    mock_adc_value = 2048;
    sensor.calibrate(10);

    mock_adc_value = 2113;
    sensor.readCurrent(10);

    // Reset filter
    sensor.resetFilter();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, sensor.getFilteredCurrent());
}

// Test: Clamp small currents (static method)
void test_clamp_small_current_below_threshold(void) {
    float result = CurrentSensor::clampSmallCurrent(0.05f, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result);
}

void test_clamp_small_current_above_threshold(void) {
    float result = CurrentSensor::clampSmallCurrent(0.15f, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0.15f, result);
}

void test_clamp_small_current_negative(void) {
    float result = CurrentSensor::clampSmallCurrent(-0.05f, 0.1f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result);
}

// Test: Negative current readings
void test_read_negative_current(void) {
    CurrentSensor sensor(5, 3.3f, 12, 0.026f, 1.0f);

    // Calibrate at mid-point
    mock_adc_value = 2048;
    sensor.calibrate(10);

    // Simulate -2A current: V = 1.65V - (2A * 0.026V/A) = 1.598V
    mock_adc_value = 1983;

    float current = sensor.readCurrent(10);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, -2.0f, current);
}

void setup() {
    delay(2000); // Wait for serial monitor

    UNITY_BEGIN();

    RUN_TEST(test_constructor_initializes_correctly);
    RUN_TEST(test_calibration_success);
    RUN_TEST(test_calibration_fails_too_low);
    RUN_TEST(test_calibration_fails_too_high);
    RUN_TEST(test_read_current_not_calibrated);
    RUN_TEST(test_read_current_after_calibration);
    RUN_TEST(test_filter_smoothing);
    RUN_TEST(test_reset_filter);
    RUN_TEST(test_clamp_small_current_below_threshold);
    RUN_TEST(test_clamp_small_current_above_threshold);
    RUN_TEST(test_clamp_small_current_negative);
    RUN_TEST(test_read_negative_current);

    UNITY_END();
}

void loop() {
    // Nothing to do here
}
