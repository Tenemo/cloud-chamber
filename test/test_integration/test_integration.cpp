#include "CurrentSensor.h"
#include "Display.h"
#include "Logger.h"
#include "PIController.h"
#include "config.h"
#include <Arduino.h>
#include <unity.h>

// Integration test - tests interaction between components
// This requires actual hardware and is ignored by default

void setUp(void) {}
void tearDown(void) {}

void test_current_sensor_and_display_integration(void) {
    // This would test real hardware interaction
    Display display(TFT_DC, TFT_CS, TFT_RST, LCD_BL);
    display.begin();

    CurrentSensor sensor1(A1, 3.3f, 12, 0.026f, 0.2f);

    TEST_ASSERT_TRUE(sensor1.calibrate(50));

    display.printLine("Test Complete", 0, 0, 1);

    delay(1000);
}

void test_full_control_loop_simulation(void) {
    // Test the full control loop with simulated inputs
    PIController pid(0.02f, 0.001f, 0.0f, 0.65f, 0.2f);

    float target = 6.0f; // Target 6A total
    float measured = 0.0f;
    float dt = 0.1f; // 100ms

    // Simulate ramping up to target
    for (int i = 0; i < 20; i++) {
        float error = target - measured;
        float duty = pid.compute(error, dt);

        // Simulate system response (simplified)
        measured += duty * 2.0f; // System gain

        if (measured > target) {
            measured = target;
        }
    }

    // Should reach near target
    TEST_ASSERT_FLOAT_WITHIN(0.5f, target, measured);
}

void setup() {
    delay(2000);

    UNITY_BEGIN();

    // Uncomment these when running on actual hardware
    // RUN_TEST(test_current_sensor_and_display_integration);
    RUN_TEST(test_full_control_loop_simulation);

    UNITY_END();
}

void loop() {}
