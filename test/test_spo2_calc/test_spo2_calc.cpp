#include <unity.h>
#include <Arduino.h>
#include <cmath>

#include "spo2_calc.h"

// ============================================================================
//  Unit Tests — Streaming SpO2 Calculator
//
//  All tests use synthetic PPG waveforms with known R-ratios.
//  No hardware, no I2C, no timers.
// ============================================================================

// ---- Helper: Generate one cardiac cycle of synthetic PPG data --------------
//  Produces a sine-like pulse on both Red and IR channels.
//  `r_ratio` controls the AC_red/DC_red vs AC_ir/DC_ir relationship.
//
//  For a given target R:
//    R = (AC_red/DC_red) / (AC_ir/DC_ir)
//
//  We fix DC_red = DC_ir = 50000 and AC_ir = 2000 (4% modulation).
//  Then: AC_red = R × AC_ir = R × 2000.
//
//  The waveform is a half-sine bump over `samples_per_cycle` samples.
//  Call this function repeatedly to generate multiple cardiac cycles.

static constexpr uint32_t DC_BASE     = 50000;
static constexpr float    AC_IR       = 2000.0f;
static constexpr uint32_t SAMPLE_RATE = 80;  // Hz (matches MAX30102 config)

static void feed_one_cycle(Spo2Calculator &calc, float r_ratio,
                           uint32_t &timestamp_ms, uint32_t cycle_duration_ms)
{
    uint32_t samples = (cycle_duration_ms * SAMPLE_RATE) / 1000;
    float ac_red = r_ratio * AC_IR;  // AC_red = R × AC_ir (since DC_red == DC_ir)

    float dt_ms = static_cast<float>(cycle_duration_ms) / static_cast<float>(samples);

    for (uint32_t i = 0; i < samples; i++) {
        // Sine-like modulation: peak at mid-cycle, trough at boundaries
        float phase = static_cast<float>(i) / static_cast<float>(samples);
        float envelope = sinf(phase * 3.14159f);  // 0 → 1 → 0

        uint32_t ir  = static_cast<uint32_t>(DC_BASE + AC_IR  * envelope);
        uint32_t red = static_cast<uint32_t>(DC_BASE + ac_red * envelope);

        calc.feed(red, ir, timestamp_ms);
        timestamp_ms += static_cast<uint32_t>(dt_ms);
    }
}

// ############################################################################
//  1. Basic Functionality
// ############################################################################

void test_spo2_initial_state_invalid()
{
    Spo2Calculator calc;
    TEST_ASSERT_EQUAL_INT32(-1, calc.spo2());
    TEST_ASSERT_FALSE(calc.valid());
}

void test_spo2_becomes_valid_after_min_cycles()
{
    Spo2Calculator calc;
    uint32_t ts = 1000;

    // Feed MIN_VALID_CYCLES - 1 cycles — should still be invalid
    for (uint32_t i = 0; i < Spo2Config::MIN_VALID_CYCLES - 1; i++) {
        feed_one_cycle(calc, 0.5f, ts, 800);  // 75 bpm
    }
    TEST_ASSERT_FALSE(calc.valid());

    // One more cycle should make it valid
    feed_one_cycle(calc, 0.5f, ts, 800);
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_TRUE(calc.spo2() > 0);
}

// ############################################################################
//  2. R-Ratio to SpO2 Mapping (Beer-Lambert: SpO2 ≈ 110 - 25×R)
// ############################################################################

void test_spo2_r04_healthy()
{
    // R = 0.4 → SpO2 = 110 - 25×0.4 = 100%
    Spo2Calculator calc;
    uint32_t ts = 1000;
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.4f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    // Should be 100% (clamped)
    TEST_ASSERT_INT_WITHIN(2, 100, calc.spo2());
}

void test_spo2_r06_normal()
{
    // R = 0.6 → SpO2 = 110 - 25×0.6 = 95%
    Spo2Calculator calc;
    uint32_t ts = 1000;
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.6f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(3, 95, calc.spo2());
}

void test_spo2_r10_moderate_hypoxia()
{
    // R = 1.0 → SpO2 = 110 - 25×1.0 = 85%
    Spo2Calculator calc;
    uint32_t ts = 1000;
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 1.0f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(3, 85, calc.spo2());
}

void test_spo2_r16_severe_hypoxia()
{
    // R = 1.6 → SpO2 = 110 - 25×1.6 = 70%
    Spo2Calculator calc;
    uint32_t ts = 1000;
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 1.6f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(4, 70, calc.spo2());
}

void test_spo2_r20_critical()
{
    // R = 2.0 → SpO2 = 110 - 25×2.0 = 60%
    Spo2Calculator calc;
    uint32_t ts = 1000;
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 2.0f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(4, 60, calc.spo2());
}

void test_spo2_clamped_at_100()
{
    // R = 0.2 → SpO2 = 110 - 25×0.2 = 105 → clamped to 100
    Spo2Calculator calc;
    uint32_t ts = 1000;
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.2f, ts, 800);
    }
    // R=0.2 is at the edge of the sanity check (>= 0.2 passes)
    if (calc.valid()) {
        TEST_ASSERT_LESS_OR_EQUAL(100, calc.spo2());
    }
}

// ############################################################################
//  3. Contact Loss & Reset
// ############################################################################

void test_spo2_contact_loss_resets()
{
    Spo2Calculator calc;
    uint32_t ts = 1000;

    // Build up valid SpO2
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.6f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());

    // Feed sub-threshold IR (contact lost)
    calc.feed(100, 100, ts);  // IR = 100 < 5000
    TEST_ASSERT_FALSE(calc.valid());
    TEST_ASSERT_EQUAL_INT32(-1, calc.spo2());
}

void test_spo2_manual_reset()
{
    Spo2Calculator calc;
    uint32_t ts = 1000;

    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.6f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());

    calc.reset();
    TEST_ASSERT_FALSE(calc.valid());
    TEST_ASSERT_EQUAL_INT32(-1, calc.spo2());
}

void test_spo2_recovers_after_reset()
{
    Spo2Calculator calc;
    uint32_t ts = 1000;

    // Build up, reset, then rebuild
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.6f, ts, 800);
    }
    calc.reset();
    TEST_ASSERT_FALSE(calc.valid());

    // Feed again — should recover
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.6f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(3, 95, calc.spo2());
}

// ############################################################################
//  4. Edge Cases
// ############################################################################

void test_spo2_flat_signal_no_ac()
{
    // Flat signal (no pulsatile component) — should not produce valid SpO2
    Spo2Calculator calc;
    uint32_t ts = 1000;

    for (uint32_t i = 0; i < 1000; i++) {
        calc.feed(50000, 50000, ts);
        ts += 12;  // ~80 Hz
    }
    // No cardiac cycles detected → still invalid
    TEST_ASSERT_FALSE(calc.valid());
}

void test_spo2_rejects_too_fast_cycles()
{
    // 250ms cycle = 240 bpm — should be rejected (MIN_CYCLE_MS = 300)
    Spo2Calculator calc;
    uint32_t ts = 1000;

    for (uint32_t i = 0; i < 20; i++) {
        feed_one_cycle(calc, 0.6f, ts, 250);
    }
    // Cycles too fast — all rejected
    TEST_ASSERT_FALSE(calc.valid());
}

void test_spo2_rejects_too_slow_cycles()
{
    // 2500ms cycle = 24 bpm — should be rejected (MAX_CYCLE_MS = 2000)
    Spo2Calculator calc;
    uint32_t ts = 1000;

    for (uint32_t i = 0; i < 10; i++) {
        feed_one_cycle(calc, 0.6f, ts, 2500);
    }
    // Cycles too slow — all rejected
    TEST_ASSERT_FALSE(calc.valid());
}

void test_spo2_normal_hr_range_60bpm()
{
    // 1000ms cycle = 60 bpm — should be accepted
    Spo2Calculator calc;
    uint32_t ts = 1000;

    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.5f, ts, 1000);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(3, 98, calc.spo2());
}

void test_spo2_fast_hr_150bpm()
{
    // 400ms cycle = 150 bpm — should be accepted (> MIN_CYCLE_MS)
    Spo2Calculator calc;
    uint32_t ts = 1000;

    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.6f, ts, 400);
    }
    TEST_ASSERT_TRUE(calc.valid());
    TEST_ASSERT_INT_WITHIN(3, 95, calc.spo2());
}

// ############################################################################
//  5. Rolling Buffer Behavior
// ############################################################################

void test_spo2_rolling_update_tracks_changes()
{
    Spo2Calculator calc;
    uint32_t ts = 1000;

    // Start with healthy R = 0.5 → SpO2 ~98%
    for (uint32_t i = 0; i < 8; i++) {
        feed_one_cycle(calc, 0.5f, ts, 800);
    }
    TEST_ASSERT_TRUE(calc.valid());
    int32_t healthy_spo2 = calc.spo2();
    TEST_ASSERT_GREATER_THAN(90, healthy_spo2);

    // Transition to moderate hypoxia R = 1.2 → SpO2 ~80%
    // Feed enough cycles to flush the ring buffer
    for (uint32_t i = 0; i < 16; i++) {
        feed_one_cycle(calc, 1.2f, ts, 800);
    }
    int32_t hypoxic_spo2 = calc.spo2();
    TEST_ASSERT_LESS_THAN(healthy_spo2, hypoxic_spo2);
    TEST_ASSERT_INT_WITHIN(5, 80, hypoxic_spo2);
}

// ############################################################################
//  Test Runner
// ############################################################################

void setUp(void) {}
void tearDown(void) {}

void setup()
{
    delay(2000);
    UNITY_BEGIN();

    // Basic functionality
    RUN_TEST(test_spo2_initial_state_invalid);
    RUN_TEST(test_spo2_becomes_valid_after_min_cycles);

    // R-ratio to SpO2 mapping
    RUN_TEST(test_spo2_r04_healthy);
    RUN_TEST(test_spo2_r06_normal);
    RUN_TEST(test_spo2_r10_moderate_hypoxia);
    RUN_TEST(test_spo2_r16_severe_hypoxia);
    RUN_TEST(test_spo2_r20_critical);
    RUN_TEST(test_spo2_clamped_at_100);

    // Contact loss & reset
    RUN_TEST(test_spo2_contact_loss_resets);
    RUN_TEST(test_spo2_manual_reset);
    RUN_TEST(test_spo2_recovers_after_reset);

    // Edge cases
    RUN_TEST(test_spo2_flat_signal_no_ac);
    RUN_TEST(test_spo2_rejects_too_fast_cycles);
    RUN_TEST(test_spo2_rejects_too_slow_cycles);
    RUN_TEST(test_spo2_normal_hr_range_60bpm);
    RUN_TEST(test_spo2_fast_hr_150bpm);

    // Rolling buffer
    RUN_TEST(test_spo2_rolling_update_tracks_changes);

    UNITY_END();
}

void loop() {}
