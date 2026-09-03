#include <unity.h>
#include <cmath>

#include "safety_types.h"
#include "heat_index.h"
#include "fall_detector.h"
#include "ppg_qualify.h"
#include "safety_engine.h"

// ============================================================================
//  Unit Tests — Tier 1 Safety Engine
//
//  All tests use injectable timestamps and raw values.
//  No hardware, no I2C, no timers.
// ============================================================================

// ---- Helper ----------------------------------------------------------------
static bool has_alert(const SafetyEngine &eng, AlertType type)
{
    for (size_t i = 0; i < eng.alert_count(); i++) {
        if (eng.alerts()[i].type == type) return true;
    }
    return false;
}

// ############################################################################
//  1. Rolling Variance
// ############################################################################

void test_rolling_variance_constant_input()
{
    RollingVariance<10> rv;
    for (int i = 0; i < 10; i++) rv.push(5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, rv.variance());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, rv.mean());
}

void test_rolling_variance_known_values()
{
    // [2, 4, 4, 4] → mean = 3.5, var = 0.75
    RollingVariance<4> rv;
    rv.push(2.0f); rv.push(4.0f); rv.push(4.0f); rv.push(4.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.5f,  rv.mean());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.75f, rv.variance());
}

void test_rolling_variance_sliding_eviction()
{
    RollingVariance<3> rv;
    rv.push(1.0f); rv.push(2.0f); rv.push(3.0f);
    // Window: [1,2,3] → mean = 2.0, var = 2/3
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.667f, rv.variance());

    rv.push(3.0f);
    // Window: [2,3,3] → mean = 8/3, var = 2/9 ≈ 0.222
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.222f, rv.variance());
}

void test_rolling_variance_reset()
{
    RollingVariance<5> rv;
    for (int i = 0; i < 5; i++) rv.push((float)i);
    TEST_ASSERT_TRUE(rv.full());
    rv.reset();
    TEST_ASSERT_EQUAL(0, rv.count());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, rv.variance());
}

// ############################################################################
//  2. Heat Index
// ############################################################################

void test_heat_index_steadman_region()
{
    // 25 °C (77 °F), 50 % RH → Steadman region, HI close to air temp
    float hi = heat_index_celsius(25.0f, 50.0f);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 25.0f, hi);
}

void test_heat_index_rothfusz_known_90F_50RH()
{
    // NWS chart: 90 °F (32.2 °C), 50 % RH → HI ≈ 95 °F (35 °C)
    float hi = heat_index_celsius(32.2f, 50.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 35.0f, hi);
}

void test_heat_index_high_humidity_amplification()
{
    // 30 °C, 90 % RH → HI should be well above air temp
    float hi = heat_index_celsius(30.0f, 90.0f);
    TEST_ASSERT_TRUE(hi > 35.0f);
}

void test_heat_index_low_humidity_adjustment()
{
    // At 35 °C with 10 % RH vs 50 % RH — low-humidity adjustment should
    // make the low-RH case yield a lower heat index.
    float hi_low  = heat_index_celsius(35.0f, 10.0f);
    float hi_high = heat_index_celsius(35.0f, 50.0f);
    TEST_ASSERT_TRUE(hi_low < hi_high);
}

void test_heat_index_symmetry_with_nws_chart()
{
    // NWS chart: 96 °F (35.6 °C), 65 % RH → HI ≈ 121 °F (49.4 °C)
    float hi = heat_index_celsius(35.6f, 65.0f);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 49.4f, hi);
}

void test_heat_index_clamps_rh()
{
    // Out-of-range RH should not crash or produce NaN
    float hi_neg = heat_index_celsius(30.0f, -10.0f);
    float hi_over = heat_index_celsius(30.0f, 150.0f);
    TEST_ASSERT_FALSE(isnan(hi_neg));
    TEST_ASSERT_FALSE(isnan(hi_over));
}

// ############################################################################
//  3. PPG Qualifier
// ############################################################################

void test_ppg_starts_calibrating()
{
    PpgQualifier pq;
    TEST_ASSERT_TRUE(pq.state() == PpgState::CALIBRATING);
    TEST_ASSERT_FALSE(pq.is_valid());
    TEST_ASSERT_EQUAL_UINT32(0, pq.sample_count());
}

void test_ppg_buffer_fill_to_valid()
{
    PpgQualifier pq;
    for (int i = 0; i < 99; i++) pq.feed_ppg(10000);
    TEST_ASSERT_TRUE(pq.state() == PpgState::CALIBRATING);

    pq.feed_ppg(10000); // 100th sample
    TEST_ASSERT_TRUE(pq.state() == PpgState::VALID);
    TEST_ASSERT_TRUE(pq.is_valid());
}

void test_ppg_contact_lost_resets_counter()
{
    PpgQualifier pq;
    for (int i = 0; i < 100; i++) pq.feed_ppg(10000);
    TEST_ASSERT_TRUE(pq.is_valid());

    // Drop IR below contact threshold
    pq.feed_ppg(3000);
    TEST_ASSERT_TRUE(pq.state() == PpgState::CONTACT_LOST);
    TEST_ASSERT_EQUAL_UINT32(0, pq.sample_count());

    // Regain contact — must re-fill
    pq.feed_ppg(10000);
    TEST_ASSERT_TRUE(pq.state() == PpgState::CALIBRATING);
    TEST_ASSERT_EQUAL_UINT32(1, pq.sample_count());
}

void test_ppg_contact_boundary_5000()
{
    PpgQualifier pq;
    // Exactly at threshold → should be treated as contact
    for (int i = 0; i < 100; i++) pq.feed_ppg(5000);
    TEST_ASSERT_TRUE(pq.is_valid());

    // One below → contact lost
    pq.feed_ppg(4999);
    TEST_ASSERT_TRUE(pq.state() == PpgState::CONTACT_LOST);
}

void test_ppg_motion_artifact_masking()
{
    PpgQualifier pq;
    for (int i = 0; i < 100; i++) pq.feed_ppg(10000);
    TEST_ASSERT_TRUE(pq.is_valid());

    // Inject high motion
    pq.set_motion_variance(1.0f);
    TEST_ASSERT_TRUE(pq.state() == PpgState::MOTION_ARTIFACT);
    TEST_ASSERT_FALSE(pq.is_valid());

    // Motion subsides
    pq.set_motion_variance(0.1f);
    TEST_ASSERT_TRUE(pq.state() == PpgState::VALID);
}

void test_ppg_motion_ignored_during_calibration()
{
    PpgQualifier pq;
    for (int i = 0; i < 50; i++) pq.feed_ppg(10000);

    // Even if motion is low, still calibrating
    pq.set_motion_variance(0.01f);
    TEST_ASSERT_TRUE(pq.state() == PpgState::CALIBRATING);
}

// ############################################################################
//  4. Fall Detector
// ############################################################################

void test_fall_normal_activity_no_trigger()
{
    FallDetector fd;
    // Simulate walking: |a| ≈ 1 g ± 0.2 g
    for (int i = 0; i < 1000; i++) {
        float a = 9.81f + sinf(i * 0.1f) * 2.0f;
        fd.feed(0.0f, 0.0f, a, i * 10);
    }
    TEST_ASSERT_TRUE(fd.state() == FallState::IDLE);
}

void test_fall_freefall_timeout_no_impact()
{
    FallDetector fd;
    // Freefall: |a| = 0.5 m/s² → 0.051 g
    fd.feed(0.0f, 0.0f, 0.5f, 0);
    TEST_ASSERT_TRUE(fd.state() == FallState::FREEFALL_DETECTED);

    // 400 ms later (> 300 ms window) — normal accel, no impact
    fd.feed(0.0f, 0.0f, 9.81f, 400);
    TEST_ASSERT_TRUE(fd.state() == FallState::IDLE);
}

void test_fall_freefall_then_impact()
{
    FallDetector fd;
    // Freefall
    fd.feed(0.0f, 0.0f, 0.5f, 0);
    TEST_ASSERT_TRUE(fd.state() == FallState::FREEFALL_DETECTED);

    // Impact within 300 ms: 35 m/s² ≈ 3.57 g
    fd.feed(0.0f, 0.0f, 35.0f, 100);
    TEST_ASSERT_TRUE(fd.state() == FallState::IMPACT_DETECTED);
}

void test_fall_complete_sequence()
{
    FallDetector fd;
    uint32_t t = 0;

    // Phase 1: Normal — 10 samples
    for (int i = 0; i < 10; i++) {
        fd.feed(0.0f, 0.0f, 9.81f, t);
        t += 10;
    }

    // Phase 2: Freefall
    fd.feed(0.0f, 0.0f, 0.5f, t);
    t += 10;
    TEST_ASSERT_TRUE(fd.state() == FallState::FREEFALL_DETECTED);

    // Phase 3: Impact
    fd.feed(0.0f, 0.0f, 35.0f, t);
    t += 10;
    TEST_ASSERT_TRUE(fd.state() == FallState::IMPACT_DETECTED);

    // Phase 4: Immobility for 6+ s (1 s buffer fill + 5 s confirmation)
    for (int i = 0; i < 650; i++) {
        fd.feed(0.0f, 0.0f, 9.81f, t);
        t += 10;
    }
    TEST_ASSERT_TRUE(fd.state() == FallState::FALL_CONFIRMED);
    TEST_ASSERT_TRUE(fd.is_fall_confirmed());
}

void test_fall_movement_after_impact_prevents_confirmation()
{
    FallDetector fd;
    uint32_t t = 0;

    // Freefall + impact
    fd.feed(0.0f, 0.0f, 0.5f, t);  t += 10;
    fd.feed(0.0f, 0.0f, 35.0f, t); t += 10;

    // Fill immobility buffer (100 samples)
    for (int i = 0; i < 100; i++) {
        fd.feed(0.0f, 0.0f, 9.81f, t);
        t += 10;
    }

    // Vigorous movement (person got up) — variance will spike
    for (int i = 0; i < 300; i++) {
        float a = 9.81f + sinf(i * 0.5f) * 5.0f;
        fd.feed(0.0f, 0.0f, a, t);
        t += 10;
    }

    TEST_ASSERT_TRUE(fd.state() != FallState::FALL_CONFIRMED);
}

void test_fall_reset()
{
    FallDetector fd;
    uint32_t t = 0;

    // Drive to FALL_CONFIRMED
    fd.feed(0.0f, 0.0f, 0.5f, t); t += 10;
    fd.feed(0.0f, 0.0f, 35.0f, t); t += 10;
    for (int i = 0; i < 650; i++) {
        fd.feed(0.0f, 0.0f, 9.81f, t); t += 10;
    }
    TEST_ASSERT_TRUE(fd.is_fall_confirmed());

    fd.reset();
    TEST_ASSERT_TRUE(fd.state() == FallState::IDLE);
    TEST_ASSERT_FALSE(fd.is_fall_confirmed());
}

// ############################################################################
//  5. SpO2 Thresholds (via SafetyEngine)
// ############################################################################

// Helper: fill the PPG buffer to VALID state
static void fill_ppg_buffer(SafetyEngine &eng)
{
    for (int i = 0; i < 100; i++) {
        eng.feed_ppg(10000, -1, false, i * 12);
    }
}

void test_spo2_immediate_critical_below_90()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    eng.feed_ppg(10000, 89, true, 2000);
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::SPO2_CRITICAL));
}

void test_spo2_at_90_no_immediate_critical()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    // SpO2 = 90 is NOT < 90, so no immediate critical
    eng.feed_ppg(10000, 90, true, 2000);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_CRITICAL));
}

void test_spo2_sustained_warning_at_91()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    uint32_t t = 1200;

    // First reading at 91 % — timer starts
    eng.feed_ppg(10000, 91, true, t);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_LOW));

    // 15 s later — not sustained long enough
    eng.feed_ppg(10000, 91, true, t + 15000);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_LOW));

    // 31 s later — sustained ≥ 30 s → WARNING
    eng.feed_ppg(10000, 91, true, t + 31000);
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::SPO2_LOW));
}

void test_spo2_recovery_resets_sustained_timer()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);
    uint32_t t = 1200;

    // Drop to 91 %
    eng.feed_ppg(10000, 91, true, t);

    // Recover to 95 % at t + 15 s
    eng.feed_ppg(10000, 95, true, t + 15000);

    // Drop again at t + 20 s — timer restarts from here
    eng.feed_ppg(10000, 91, true, t + 20000);

    // t + 45 s = 25 s since second drop — not yet 30 s
    eng.feed_ppg(10000, 91, true, t + 45000);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_LOW));

    // t + 51 s = 31 s since second drop — now sustained
    eng.feed_ppg(10000, 91, true, t + 51000);
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::SPO2_LOW));
}

void test_spo2_at_92_no_warning()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    // 92 % is NOT < 92, so no sustained warning even after forever
    eng.feed_ppg(10000, 92, true, 0);
    eng.feed_ppg(10000, 92, true, 60000);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_LOW));
}

// ############################################################################
//  6. PPG Masking Suppresses Alerts
// ############################################################################

void test_calibrating_suppresses_spo2()
{
    SafetyEngine eng;

    // Only 50 samples — still calibrating
    for (int i = 0; i < 50; i++) {
        eng.feed_ppg(10000, 85, true, i * 12);
    }

    // SpO2 is 85 % (critically low!) but PPG is calibrating → no alert
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::CALIBRATING);
    TEST_ASSERT_EQUAL(0, eng.alert_count());
}

void test_contact_loss_suppresses_spo2()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    // Lose contact
    eng.feed_ppg(3000, 88, true, 5000);

    // PPG invalid → no SpO2 alert despite 88 %
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::CONTACT_LOST);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_CRITICAL));
}

void test_contact_loss_triggers_recalibration()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::VALID);

    // Lose contact
    eng.feed_ppg(2000, 95, true, 5000);
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::CONTACT_LOST);

    // Regain contact — must go through calibration again
    eng.feed_ppg(10000, 95, true, 5100);
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::CALIBRATING);

    // Fill up again
    for (int i = 1; i < 100; i++) {
        eng.feed_ppg(10000, 95, true, 5100 + i * 12);
    }
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::VALID);
}

// ############################################################################
//  7. Motion-Gated PPG
// ############################################################################

void test_motion_gates_ppg_to_artifact()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::VALID);

    // Feed vigorous motion → variance > 0.3 (m/s²)²
    for (int i = 0; i < 60; i++) {
        float a = 9.81f + sinf(i * 0.8f) * 4.0f;
        eng.feed_imu(0.0f, 0.0f, a, i * 10);
    }

    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::MOTION_ARTIFACT);
}

void test_motion_artifact_suppresses_spo2()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    // High motion
    for (int i = 0; i < 60; i++) {
        float a = 9.81f + sinf(i * 0.8f) * 4.0f;
        eng.feed_imu(0.0f, 0.0f, a, i * 10);
    }
    TEST_ASSERT_TRUE(eng.ppg_state() == PpgState::MOTION_ARTIFACT);

    // Low SpO2 during motion — should be suppressed
    eng.feed_ppg(10000, 85, true, 1000);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::SPO2_CRITICAL));
}

// ############################################################################
//  8. OLED Status Strings
// ############################################################################

void test_oled_calibrating_string()
{
    SafetyEngine eng;
    // Buffer not filled
    TEST_ASSERT_EQUAL_STRING("Calibrating Vitals...",
                             eng.ppg_status_string());
}

void test_oled_contact_lost_string()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);
    eng.feed_ppg(2000, 95, true, 5000); // lose contact
    TEST_ASSERT_EQUAL_STRING("Calibrating Vitals...",
                             eng.ppg_status_string());
}

void test_oled_valid_string()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);
    TEST_ASSERT_EQUAL_STRING("Vitals OK",
                             eng.ppg_status_string());
}

// ############################################################################
//  9. Heat Index Alerting
// ############################################################################

void test_heat_warning_at_40c()
{
    SafetyEngine eng;
    // 40 °C, 50 % RH → HI well above 40 °C threshold
    eng.feed_environment(40.0f, 50.0f, 1000);
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::HEAT_STRESS) ||
                     has_alert(eng, AlertType::HEAT_DANGER));
}

void test_heat_no_alert_mild()
{
    SafetyEngine eng;
    // 25 °C, 50 % RH → mild, no alert
    eng.feed_environment(25.0f, 50.0f, 1000);
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::HEAT_STRESS));
    TEST_ASSERT_FALSE(has_alert(eng, AlertType::HEAT_DANGER));
}

void test_heat_critical_extreme()
{
    SafetyEngine eng;
    // 50 °C, 80 % RH → extreme heat danger
    eng.feed_environment(50.0f, 80.0f, 1000);
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::HEAT_DANGER));
}

// ############################################################################
//  10. Integration: Multiple Alerts Coexist
// ############################################################################

void test_multiple_simultaneous_alerts()
{
    SafetyEngine eng;
    fill_ppg_buffer(eng);

    // Drive SpO2 critical
    eng.feed_ppg(10000, 88, true, 5000);

    // Drive heat danger
    eng.feed_environment(50.0f, 80.0f, 5000);

    // Should have both alerts active
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::SPO2_CRITICAL));
    TEST_ASSERT_TRUE(has_alert(eng, AlertType::HEAT_DANGER));
    TEST_ASSERT_TRUE(eng.alert_count() >= 2);
}

// ============================================================================
//  Test Runner
// ============================================================================
void setup()
{
    delay(2000);
    UNITY_BEGIN();

    // Rolling variance
    RUN_TEST(test_rolling_variance_constant_input);
    RUN_TEST(test_rolling_variance_known_values);
    RUN_TEST(test_rolling_variance_sliding_eviction);
    RUN_TEST(test_rolling_variance_reset);

    // Heat index
    RUN_TEST(test_heat_index_steadman_region);
    RUN_TEST(test_heat_index_rothfusz_known_90F_50RH);
    RUN_TEST(test_heat_index_high_humidity_amplification);
    RUN_TEST(test_heat_index_low_humidity_adjustment);
    RUN_TEST(test_heat_index_symmetry_with_nws_chart);
    RUN_TEST(test_heat_index_clamps_rh);

    // PPG qualifier
    RUN_TEST(test_ppg_starts_calibrating);
    RUN_TEST(test_ppg_buffer_fill_to_valid);
    RUN_TEST(test_ppg_contact_lost_resets_counter);
    RUN_TEST(test_ppg_contact_boundary_5000);
    RUN_TEST(test_ppg_motion_artifact_masking);
    RUN_TEST(test_ppg_motion_ignored_during_calibration);

    // Fall detector
    RUN_TEST(test_fall_normal_activity_no_trigger);
    RUN_TEST(test_fall_freefall_timeout_no_impact);
    RUN_TEST(test_fall_freefall_then_impact);
    RUN_TEST(test_fall_complete_sequence);
    RUN_TEST(test_fall_movement_after_impact_prevents_confirmation);
    RUN_TEST(test_fall_reset);

    // SpO2 thresholds
    RUN_TEST(test_spo2_immediate_critical_below_90);
    RUN_TEST(test_spo2_at_90_no_immediate_critical);
    RUN_TEST(test_spo2_sustained_warning_at_91);
    RUN_TEST(test_spo2_recovery_resets_sustained_timer);
    RUN_TEST(test_spo2_at_92_no_warning);

    // PPG masking / suppression
    RUN_TEST(test_calibrating_suppresses_spo2);
    RUN_TEST(test_contact_loss_suppresses_spo2);
    RUN_TEST(test_contact_loss_triggers_recalibration);

    // Motion gating
    RUN_TEST(test_motion_gates_ppg_to_artifact);
    RUN_TEST(test_motion_artifact_suppresses_spo2);

    // OLED strings
    RUN_TEST(test_oled_calibrating_string);
    RUN_TEST(test_oled_contact_lost_string);
    RUN_TEST(test_oled_valid_string);

    // Heat index alerting
    RUN_TEST(test_heat_warning_at_40c);
    RUN_TEST(test_heat_no_alert_mild);
    RUN_TEST(test_heat_critical_extreme);

    // Integration
    RUN_TEST(test_multiple_simultaneous_alerts);

    UNITY_END();
}

void loop() {}
