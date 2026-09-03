#include <unity.h>
#include "sensor_types.h"

// ============================================================================
//  Unit Tests — Sensor Data Structures & Read-Function Contracts
//
//  These tests validate the data-layer contracts without requiring real
//  hardware.  On-target integration tests can extend this by calling the
//  actual read_*() functions with a live I2C bus.
// ============================================================================

// ---- Sensor Types Sanity ---------------------------------------------------

void test_ppg_reading_zero_init()
{
    PpgReading r = {};
    TEST_ASSERT_EQUAL_UINT32(0, r.ir);
    TEST_ASSERT_EQUAL_UINT32(0, r.red);
    TEST_ASSERT_EQUAL_UINT32(0, r.timestamp_ms);
}

void test_imu_reading_zero_init()
{
    ImuReading r = {};
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.ax);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, r.gz);
}

void test_env_reading_mq135_invalid_by_default()
{
    EnvReading r = {};
    // A zero-initialized EnvReading should report MQ135 as invalid
    TEST_ASSERT_FALSE(r.mq135_valid);
}

void test_sensor_frame_aggregate()
{
    SensorFrame f = {};
    f.ppg.ir  = 12345;
    f.imu.ax  = 9.81f;
    f.temp.skin_temp_c = 36.5f;
    f.env.bme_humidity = 55.0f;
    f.env.mq135_raw    = 2048;
    f.env.mq135_valid  = true;

    TEST_ASSERT_EQUAL_UINT32(12345, f.ppg.ir);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.81f, f.imu.ax);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 36.5f, f.temp.skin_temp_c);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 55.0f, f.env.bme_humidity);
    TEST_ASSERT_EQUAL_UINT16(2048, f.env.mq135_raw);
    TEST_ASSERT_TRUE(f.env.mq135_valid);
}

// ---- MQ135 Warm-up Logic ---------------------------------------------------

/// Simulates the blackout-gate check used in read_environment().
static bool mq135_is_valid(uint32_t boot_ms, uint32_t now_ms,
                           uint32_t warmup_ms)
{
    return (now_ms - boot_ms) >= warmup_ms;
}

void test_mq135_warmup_blackout()
{
    // Immediately after boot — should be invalid
    TEST_ASSERT_FALSE(mq135_is_valid(0, 0, 90000));
    TEST_ASSERT_FALSE(mq135_is_valid(0, 45000, 90000));
    TEST_ASSERT_FALSE(mq135_is_valid(0, 89999, 90000));

    // At exactly the boundary and beyond — should be valid
    TEST_ASSERT_TRUE(mq135_is_valid(0, 90000, 90000));
    TEST_ASSERT_TRUE(mq135_is_valid(0, 120000, 90000));
}

void test_mq135_warmup_with_nonzero_boot()
{
    uint32_t boot = 5000;
    TEST_ASSERT_FALSE(mq135_is_valid(boot, 5000, 90000));
    TEST_ASSERT_FALSE(mq135_is_valid(boot, 94999, 90000));
    TEST_ASSERT_TRUE(mq135_is_valid(boot, 95000, 90000));
}

// ---- Accel Magnitude Helper ------------------------------------------------

static float accel_magnitude(float ax, float ay, float az)
{
    return sqrtf(ax * ax + ay * ay + az * az);
}

void test_accel_magnitude_stationary()
{
    // At rest: only gravity on Z
    float mag = accel_magnitude(0.0f, 0.0f, 9.81f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.81f, mag);
}

void test_accel_magnitude_3d()
{
    // Classic 3-4-5 triangle scaled
    float mag = accel_magnitude(3.0f, 4.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, mag);
}

// ---- IMU Scale Factor Verification -----------------------------------------

void test_imu_scale_conversion()
{
    // ±4 g range: 8192 LSB/g
    constexpr float ACCEL_SCALE = 9.80665f / 8192.0f;
    int16_t raw_1g = 8192;
    float result = raw_1g * ACCEL_SCALE;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.81f, result);

    // ±500 °/s range: 65.5 LSB/(°/s)
    constexpr float GYRO_SCALE = 1.0f / 65.5f;
    int16_t raw_500 = (int16_t)(500 * 65.5f);
    float gyro = raw_500 * GYRO_SCALE;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 500.0f, gyro);
}

// ============================================================================
//  Test Runner
// ============================================================================

void setup()
{
    delay(2000);  // let serial attach
    UNITY_BEGIN();

    RUN_TEST(test_ppg_reading_zero_init);
    RUN_TEST(test_imu_reading_zero_init);
    RUN_TEST(test_env_reading_mq135_invalid_by_default);
    RUN_TEST(test_sensor_frame_aggregate);

    RUN_TEST(test_mq135_warmup_blackout);
    RUN_TEST(test_mq135_warmup_with_nonzero_boot);

    RUN_TEST(test_accel_magnitude_stationary);
    RUN_TEST(test_accel_magnitude_3d);

    RUN_TEST(test_imu_scale_conversion);

    UNITY_END();
}

void loop() {}
