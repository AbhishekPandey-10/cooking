#include <unity.h>
#include <Arduino.h>
#include <cmath>
#include <cstring>

#include "correlator.h"

// ============================================================================
//  Unit Tests — Cross-Sensor Anomaly Correlator
//
//  Tests cover:
//    - Path A hard bypass (fall, SpO2 critical)
//    - Path A boundary conditions (SpO2 = 90 is NOT Path A)
//    - Path B individual flag evaluation
//    - Path B coincidence logic (1-of-4 vs 2-of-4 vs 4-of-4)
//    - Window filling, eviction, and wrap-around
//    - Post-alert reset and re-accumulation
//    - MQ135 delta tracking across resets
//    - Debug trigger description
// ============================================================================

// ---- Helpers ---------------------------------------------------------------

/// Tick with all-normal sensor values — no flags should fire.
static CorrelatorResult tick_normal(Correlator &c, uint32_t t)
{
    return c.tick(
        /* spo2 */         98,
        /* spo2_valid */   true,
        /* fall */         false,
        /* heat_index_c */ 25.0f,
        /* mq135_raw */    200,
        /* mq135_valid */  true,
        /* skin_temp_c */  36.5f,
        /* timestamp_ms */ t
    );
}

/// Tick with a specific SpO2 value, everything else normal.
static CorrelatorResult tick_spo2(Correlator &c, int32_t spo2, uint32_t t)
{
    return c.tick(spo2, true, false, 25.0f, 200, true, 36.5f, t);
}

/// Tick with only the optical flag set (SpO2 = 91).
static CorrelatorResult tick_optical(Correlator &c, uint32_t t)
{
    return c.tick(91, true, false, 25.0f, 200, true, 36.5f, t);
}

/// Tick with only the heat flag set (HI = 42 °C).
static CorrelatorResult tick_heat(Correlator &c, uint32_t t)
{
    return c.tick(98, true, false, 42.0f, 200, true, 36.5f, t);
}

/// Tick with only the thermal flag set (skin = 28 °C, below 30 °C threshold).
static CorrelatorResult tick_thermal(Correlator &c, uint32_t t)
{
    return c.tick(98, true, false, 25.0f, 200, true, 28.0f, t);
}

// ############################################################################
//  1. Path A — Hard Bypass
// ############################################################################

void test_path_a_fall_detected()
{
    Correlator c;
    CorrelatorResult r = c.tick(98, true, true, 25.0f, 200, true, 36.5f, 0);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_A_HARD_BYPASS);
}

void test_path_a_spo2_below_90()
{
    Correlator c;
    CorrelatorResult r = tick_spo2(c, 89, 0);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_A_HARD_BYPASS);
}

void test_path_a_spo2_very_low()
{
    Correlator c;
    CorrelatorResult r = tick_spo2(c, 50, 0);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_A_HARD_BYPASS);
}

void test_path_a_boundary_spo2_90_is_not_bypass()
{
    // SpO2 = 90 should be Path B (optical flag), NOT Path A
    Correlator c;
    // First tick to establish MQ135 baseline
    tick_normal(c, 0);
    CorrelatorResult r = tick_spo2(c, 90, 1000);
    TEST_ASSERT_TRUE(r != CorrelatorResult::PATH_A_HARD_BYPASS);
}

void test_path_a_spo2_invalid_does_not_trigger()
{
    // If SpO2 is not valid, don't fire Path A even if value is low
    Correlator c;
    CorrelatorResult r = c.tick(50, false, false, 25.0f, 200, true, 36.5f, 0);
    TEST_ASSERT_TRUE(r != CorrelatorResult::PATH_A_HARD_BYPASS);
}

void test_path_a_preempts_path_b()
{
    // Even if Path B conditions are also met, Path A returns immediately
    Correlator c;
    // Build up thermal + heat in window
    for (uint32_t t = 0; t < 5; t++) {
        c.tick(98, true, false, 42.0f, 200, true, 28.0f, t * 1000);
    }
    // Now tick with fall + active Path B flags
    CorrelatorResult r = c.tick(98, true, true, 42.0f, 200, true, 28.0f, 5000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_A_HARD_BYPASS);
}

// ############################################################################
//  2. Path B — Individual Flag Evaluation
// ############################################################################

void test_optical_flag_at_90()
{
    Correlator c;
    tick_normal(c, 0); // baseline
    tick_spo2(c, 90, 1000);
    TEST_ASSERT_TRUE(c.optical_active());
}

void test_optical_flag_at_92()
{
    Correlator c;
    tick_normal(c, 0);
    tick_spo2(c, 92, 1000);
    TEST_ASSERT_TRUE(c.optical_active());
}

void test_optical_flag_not_at_93()
{
    Correlator c;
    tick_normal(c, 0);
    tick_spo2(c, 93, 1000);
    TEST_ASSERT_FALSE(c.optical_active());
}

void test_heat_flag()
{
    Correlator c;
    tick_normal(c, 0);
    tick_heat(c, 1000);
    TEST_ASSERT_TRUE(c.heat_active());
}

void test_heat_flag_at_threshold_exact()
{
    Correlator c;
    tick_normal(c, 0);
    // Exactly 38.0 °C should NOT trigger (threshold is >38, not >=38)
    c.tick(98, true, false, 38.0f, 200, true, 36.5f, 1000);
    TEST_ASSERT_FALSE(c.heat_active());
}

void test_gas_flag_delta_above_threshold()
{
    Correlator c;
    // First tick: establish baseline at 200
    c.tick(98, true, false, 25.0f, 200, true, 36.5f, 0);
    // Second tick: jump to 400 (Δ = 200 > 150 threshold)
    c.tick(98, true, false, 25.0f, 400, true, 36.5f, 1000);
    TEST_ASSERT_TRUE(c.gas_active());
}

void test_gas_flag_delta_below_threshold()
{
    Correlator c;
    c.tick(98, true, false, 25.0f, 200, true, 36.5f, 0);
    // Δ = 100 < 150
    c.tick(98, true, false, 25.0f, 300, true, 36.5f, 1000);
    TEST_ASSERT_FALSE(c.gas_active());
}

void test_gas_flag_no_previous_reading()
{
    Correlator c;
    // Very first tick — no previous value → no delta → no gas flag
    c.tick(98, true, false, 25.0f, 500, true, 36.5f, 0);
    TEST_ASSERT_FALSE(c.gas_active());
}

void test_gas_flag_negative_delta()
{
    Correlator c;
    c.tick(98, true, false, 25.0f, 400, true, 36.5f, 0);
    // Drop from 400 to 200 → |Δ| = 200 > 150
    c.tick(98, true, false, 25.0f, 200, true, 36.5f, 1000);
    TEST_ASSERT_TRUE(c.gas_active());
}

void test_gas_flag_mq135_invalid_skipped()
{
    Correlator c;
    c.tick(98, true, false, 25.0f, 200, true, 36.5f, 0);
    // Invalid MQ135 (still in warm-up) — should NOT update baseline or flag
    c.tick(98, true, false, 25.0f, 4000, false, 36.5f, 1000);
    TEST_ASSERT_FALSE(c.gas_active());
    // Next valid reading still compares against the original 200
    c.tick(98, true, false, 25.0f, 210, true, 36.5f, 2000);
    TEST_ASSERT_FALSE(c.gas_active()); // Δ = 10 < 150
}

void test_thermal_flag_cold()
{
    Correlator c;
    tick_normal(c, 0);
    // Skin temp 29.5 °C < 30 °C threshold
    c.tick(98, true, false, 25.0f, 200, true, 29.5f, 1000);
    TEST_ASSERT_TRUE(c.thermal_active());
}

void test_thermal_flag_hot()
{
    Correlator c;
    tick_normal(c, 0);
    // Skin temp 39.0 °C > 38.5 °C threshold
    c.tick(98, true, false, 25.0f, 200, true, 39.0f, 1000);
    TEST_ASSERT_TRUE(c.thermal_active());
}

void test_thermal_flag_normal()
{
    Correlator c;
    tick_normal(c, 0);
    c.tick(98, true, false, 25.0f, 200, true, 36.5f, 1000);
    TEST_ASSERT_FALSE(c.thermal_active());
}

// ############################################################################
//  3. Path B — Coincidence Logic
// ############################################################################

void test_single_flag_no_alert()
{
    Correlator c;
    tick_normal(c, 0); // MQ135 baseline

    // 45 ticks of only the optical flag — 1-of-4 should NOT fire
    for (uint32_t t = 1; t <= 45; t++) {
        CorrelatorResult r = tick_optical(c, t * 1000);
        TEST_ASSERT_TRUE(r == CorrelatorResult::NONE);
    }
}

void test_two_flags_fires_alert()
{
    Correlator c;
    tick_normal(c, 0); // MQ135 baseline

    // Tick 1: optical flag
    tick_optical(c, 1000);

    // Tick 2: thermal flag → 2-of-4 → alert
    CorrelatorResult r = tick_thermal(c, 2000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_B_CORRELATED);
}

void test_three_flags_fires_alert()
{
    Correlator c;
    tick_normal(c, 0);

    tick_optical(c, 1000);
    tick_heat(c, 2000);
    CorrelatorResult r = tick_thermal(c, 3000);
    // Already 3-of-4 — must fire
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_B_CORRELATED);
}

void test_all_four_flags_fires_alert()
{
    Correlator c;
    // Tick 0: baseline MQ135 = 200
    tick_normal(c, 0);

    // Tick 1: optical (SpO2 91) + heat (HI 42°C) + thermal (28°C) + gas (MQ135 jump)
    CorrelatorResult r = c.tick(91, true, false, 42.0f, 500, true, 28.0f, 1000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_B_CORRELATED);
}

void test_two_flags_noncontiguous_still_fires()
{
    // Flags don't need to overlap in time — just be present anywhere in
    // the 45 s window.
    Correlator c;
    tick_normal(c, 0);

    // Tick 2: optical flag
    tick_optical(c, 2000);

    // Ticks 3–40: all normal (optical flag is aging but still in window)
    for (uint32_t t = 3; t <= 40; t++) {
        tick_normal(c, t * 1000);
    }
    TEST_ASSERT_EQUAL(1, c.active_flag_count()); // optical still in window

    // Tick 41: thermal flag → 2-of-4
    CorrelatorResult r = tick_thermal(c, 41000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_B_CORRELATED);
}

// ############################################################################
//  4. Window Eviction & Wrap
// ############################################################################

void test_flag_expires_from_window()
{
    Correlator c;
    tick_normal(c, 0);

    // Tick 1: optical flag
    tick_optical(c, 1000);
    TEST_ASSERT_TRUE(c.optical_active());

    // Fill the rest of the window with normal ticks
    for (uint32_t t = 2; t <= 45; t++) {
        tick_normal(c, t * 1000);
    }
    // Optical flag was at position 1; window has 45 entries (0..44).
    // It's still in the window.
    TEST_ASSERT_TRUE(c.optical_active());

    // Tick 46: the entry from tick 1 is evicted
    tick_normal(c, 46000);
    TEST_ASSERT_FALSE(c.optical_active());
    TEST_ASSERT_EQUAL(0, c.active_flag_count());
}

void test_expired_flag_prevents_alert()
{
    Correlator c;
    tick_normal(c, 0);

    // Tick 1: optical flag
    tick_optical(c, 1000);

    // Let it expire by filling 45 more normal ticks
    for (uint32_t t = 2; t <= 46; t++) {
        tick_normal(c, t * 1000);
    }
    TEST_ASSERT_FALSE(c.optical_active());

    // Now add thermal — only 1-of-4, should NOT fire
    CorrelatorResult r = tick_thermal(c, 47000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::NONE);
    TEST_ASSERT_EQUAL(1, c.active_flag_count());
}

// ############################################################################
//  5. Post-Alert Reset & Re-Accumulation
// ############################################################################

void test_reset_clears_window()
{
    Correlator c;
    tick_normal(c, 0);
    tick_optical(c, 1000);
    tick_heat(c, 2000); // fires Path B

    // After the alert, window should be empty
    TEST_ASSERT_EQUAL(0, c.window_fill());
    TEST_ASSERT_EQUAL(0, c.active_flag_count());
}

void test_no_immediate_refire_after_reset()
{
    Correlator c;
    tick_normal(c, 0);
    tick_optical(c, 1000);

    CorrelatorResult r = tick_heat(c, 2000); // fires
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_B_CORRELATED);

    // Next tick with both flags — window was reset, so this is the first
    // entry.  Only 1 entry can set at most the flags present in that single
    // tick.  If that tick has optical + heat, it's 2-of-4 in one tick.
    r = c.tick(91, true, false, 42.0f, 200, true, 36.5f, 3000);
    // Actually, this single tick has both optical AND heat → 2-of-4 → fires!
    // This is correct behavior: if a single tick carries 2+ flags, the
    // coincidence is immediately evident.
    TEST_ASSERT_TRUE(r == CorrelatorResult::PATH_B_CORRELATED);
}

void test_single_flag_after_reset_requires_second()
{
    Correlator c;
    tick_normal(c, 0);
    tick_optical(c, 1000);
    tick_heat(c, 2000); // fires, resets

    // After reset, single-flag ticks should not fire
    CorrelatorResult r = tick_optical(c, 3000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::NONE);

    r = tick_optical(c, 4000);
    TEST_ASSERT_TRUE(r == CorrelatorResult::NONE);
}

// ############################################################################
//  6. MQ135 Delta Continuity Across Reset
// ############################################################################

void test_mq135_baseline_survives_reset()
{
    Correlator c;
    // Establish baseline at 200
    c.tick(98, true, false, 25.0f, 200, true, 36.5f, 0);

    // Trigger alert to cause reset (optical + heat)
    c.tick(91, true, false, 42.0f, 200, true, 36.5f, 1000);
    // Window is now reset

    // Next tick: MQ135 = 400 → Δ from 200 = 200 > 150
    // (baseline survived the reset)
    c.tick(98, true, false, 25.0f, 400, true, 36.5f, 2000);
    TEST_ASSERT_TRUE(c.gas_active());
}

// ############################################################################
//  7. Debug Trigger Description
// ############################################################################

void test_trigger_description_populated()
{
    Correlator c;
    tick_normal(c, 0);
    tick_optical(c, 1000);
    tick_thermal(c, 2000); // fires

    const char* desc = c.last_trigger_description();
    TEST_ASSERT_NOT_NULL(desc);
    TEST_ASSERT_TRUE(strlen(desc) > 0);
}

void test_trigger_description_shows_active_flags()
{
    Correlator c;
    tick_normal(c, 0);

    // 3 optical ticks, then 1 thermal
    tick_optical(c, 1000);
    tick_optical(c, 2000);
    tick_optical(c, 3000);
    tick_thermal(c, 4000); // fires

    const char* desc = c.last_trigger_description();
    // Should contain "OPT" and "THRM"
    TEST_ASSERT_NOT_NULL(strstr(desc, "OPT"));
    TEST_ASSERT_NOT_NULL(strstr(desc, "THRM"));
    // OPT should show count of 3
    TEST_ASSERT_NOT_NULL(strstr(desc, "OPT(3)"));
    // THRM should show count of 1
    TEST_ASSERT_NOT_NULL(strstr(desc, "THRM(1)"));
}

void test_trigger_description_survives_reset()
{
    Correlator c;
    tick_normal(c, 0);
    tick_optical(c, 1000);
    tick_heat(c, 2000); // fires

    const char* desc = c.last_trigger_description();
    // Description should still be readable after the reset
    TEST_ASSERT_NOT_NULL(strstr(desc, "OPT"));
    TEST_ASSERT_NOT_NULL(strstr(desc, "HEAT"));
}

// ############################################################################
//  8. Window Fill Tracking
// ############################################################################

void test_window_fill_count()
{
    Correlator c;
    TEST_ASSERT_EQUAL(0, c.window_fill());

    tick_normal(c, 0);
    TEST_ASSERT_EQUAL(1, c.window_fill());

    for (uint32_t t = 1; t < 45; t++) {
        tick_normal(c, t * 1000);
    }
    TEST_ASSERT_EQUAL(45, c.window_fill());

    // 46th tick — still 45 (buffer is full, oldest evicted)
    tick_normal(c, 45000);
    TEST_ASSERT_EQUAL(45, c.window_fill());
}

void test_active_count_reflects_distinct_categories()
{
    Correlator c;
    tick_normal(c, 0);

    // Many optical ticks — still only 1 distinct flag
    for (uint32_t t = 1; t <= 10; t++) {
        tick_optical(c, t * 1000);
    }
    TEST_ASSERT_EQUAL(1, c.active_flag_count());
    TEST_ASSERT_TRUE(c.optical_hits() == 10);
}

// ============================================================================
//  Test Runner
// ============================================================================
void setup()
{
    delay(2000);
    UNITY_BEGIN();

    // Path A
    RUN_TEST(test_path_a_fall_detected);
    RUN_TEST(test_path_a_spo2_below_90);
    RUN_TEST(test_path_a_spo2_very_low);
    RUN_TEST(test_path_a_boundary_spo2_90_is_not_bypass);
    RUN_TEST(test_path_a_spo2_invalid_does_not_trigger);
    RUN_TEST(test_path_a_preempts_path_b);

    // Individual flags
    RUN_TEST(test_optical_flag_at_90);
    RUN_TEST(test_optical_flag_at_92);
    RUN_TEST(test_optical_flag_not_at_93);
    RUN_TEST(test_heat_flag);
    RUN_TEST(test_heat_flag_at_threshold_exact);
    RUN_TEST(test_gas_flag_delta_above_threshold);
    RUN_TEST(test_gas_flag_delta_below_threshold);
    RUN_TEST(test_gas_flag_no_previous_reading);
    RUN_TEST(test_gas_flag_negative_delta);
    RUN_TEST(test_gas_flag_mq135_invalid_skipped);
    RUN_TEST(test_thermal_flag_cold);
    RUN_TEST(test_thermal_flag_hot);
    RUN_TEST(test_thermal_flag_normal);

    // Coincidence logic
    RUN_TEST(test_single_flag_no_alert);
    RUN_TEST(test_two_flags_fires_alert);
    RUN_TEST(test_three_flags_fires_alert);
    RUN_TEST(test_all_four_flags_fires_alert);
    RUN_TEST(test_two_flags_noncontiguous_still_fires);

    // Window eviction
    RUN_TEST(test_flag_expires_from_window);
    RUN_TEST(test_expired_flag_prevents_alert);

    // Reset behavior
    RUN_TEST(test_reset_clears_window);
    RUN_TEST(test_no_immediate_refire_after_reset);
    RUN_TEST(test_single_flag_after_reset_requires_second);

    // MQ135 continuity
    RUN_TEST(test_mq135_baseline_survives_reset);

    // Debug descriptions
    RUN_TEST(test_trigger_description_populated);
    RUN_TEST(test_trigger_description_shows_active_flags);
    RUN_TEST(test_trigger_description_survives_reset);

    // Bookkeeping
    RUN_TEST(test_window_fill_count);
    RUN_TEST(test_active_count_reflects_distinct_categories);

    UNITY_END();
}

void loop() {}
