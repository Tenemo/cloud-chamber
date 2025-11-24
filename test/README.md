# Unit Testing for TEC Controller

This directory is intended for PlatformIO Test Runner and project tests.

Unit Testing is a software testing method by which individual units of
source code, sets of one or more MCU program modules together with associated
control data, usage procedures, and operating procedures, are tested to
determine whether they are fit for use. Unit testing finds problems early
in the development cycle.

More information about PlatformIO Unit Testing:

- https://docs.platformio.org/en/latest/advanced/unit-testing/index.html

This directory contains unit tests for the TEC Controller firmware.

## Test Structure

- `test_current_sensor/` - Tests for CurrentSensor class

  - Calibration logic
  - Current reading and filtering
  - Edge cases and error handling

- `test_pi_controller/` - Tests for PIController class

  - Proportional and integral control
  - Anti-windup behavior
  - Output limiting

- `test_integration/` - Integration tests (ignored by default)
  - Full system tests requiring hardware

## Running Tests

### Run all unit tests on the target hardware:

```bash
pio test -e dfrobot_firebeetle2_esp32s3
```

### Run a specific test:

```bash
pio test -e dfrobot_firebeetle2_esp32s3 -f test_current_sensor
```

### Run tests on native (desktop) - for pure logic tests:

```bash
pio test -e native
```

### Verbose output:

```bash
pio test -e dfrobot_firebeetle2_esp32s3 -v
```

## Test Output

Tests use the Unity test framework. Each test outputs:

- `OK` - Test passed
- `FAIL` - Test failed with details
- Summary at the end with pass/fail counts

## Adding New Tests

1. Create a new directory in `test/` with name starting with `test_`
2. Add a `.cpp` file with your test cases
3. Include `<unity.h>` and use Unity assertions
4. Implement `setup()` to run tests and `loop()` (empty)

## Mocking Hardware

For hardware-dependent code (like ADC reads), override the function in your test file:

```cpp
int analogRead(uint8_t pin) {
    return mock_value;
}
```

## Unity Assertions

Common assertions:

- `TEST_ASSERT_EQUAL(expected, actual)`
- `TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual)`
- `TEST_ASSERT_TRUE(condition)`
- `TEST_ASSERT_FALSE(condition)`
- `TEST_ASSERT_NULL(pointer)`

See: https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityAssertionsReference.md
