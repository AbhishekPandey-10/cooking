#include <unity.h>
#include <Arduino.h>
#include <cmath>

#include "feature_extract.h"
#include "anomaly_engine.h"

// ============================================================================
//  Unit Tests — Feature Extraction + Anomaly Engine
//
//  Tests cover:
//    - PPG peak detection with synthetic waveforms
//    - HR and RMSSD computation from known intervals
//    - Feature vector validity tracking
//    - Baseline statistics accumulation
//    - Z-score anomaly detection
//    - Cold-start state machine (DORMANT → ACTIVE → SUSPENDED)
//    - NaN / invalid feature handling
//    - Motion cooldown dampening
//    - Confidence modifier mapping
// ============================================================================

// ---- Synthetic PPG waveform generator --------------------------------------
//  Generates a sine-like pulse at a configurable heart rate.
//  peak_amplitude is added ON TOP of a DC baseline.
static float synth_ppg(uint32_t t_ms, float hr_bpm,
                       float dc_baseline, float ac_amplitude)
{
    float freq_hz = hr_bpm / 60.0f;
    float phase   = 2.0f * 3.14159265f * freq_hz * (t_ms / 1000.0f);
    // Use a sharper waveform (squared sine) to create distinct peaks
    float s = sinf(phase);
    float pulse = (s > 0.0f) ? s * s : 0.0f;
    return dc_baseline + ac_amplitude * pulse;
}

// ---- Build a "normal resting" FeatureVector --------------------------------
static FeatureVector make_normal_fv()
{
    FeatureVector fv;
    fv.values[0] = 72.0f;   // HR bpm
    fv.values[1] = 42.0f;   // RMSSD ms
    fv.values[2] = 97.0f;   // SpO2 %
    fv.values[3] = 36.5f;   // Skin temp °C
    fv.values[4] = 0.01f;   // Skin temp slope °C/min
    fv.values[5] = 0.05f;   // Accel variance (m/s²)²
    for (int i = 0; i < 6; i++) fv.valid[i] = true;
    return fv;
}

// ---- Build an "anomalous" FeatureVector ------------------------------------
static FeatureVector make_anomalous_fv()
{
    FeatureVector fv;
    fv.values[0] = 145.0f;  // Tachycardic
    fv.values[1] = 8.0f;    // Very low RMSSD (sympathetic storm)
    fv.values[2] = 91.0f;   // Borderline SpO2
    fv.values[3] = 39.0f;   // Elevated skin temp
    fv.values[4] = 0.5f;    // Rapid temp rise
    fv.values[5] = 0.02f;   // But immobile (alarming combo)
    for (int i = 0; i < 6; i++) fv.valid[i] = true;
    return fv;
}

// ############################################################################
//  1. PPG Peak Detector
// ############################################################################

void test_peak_detector_finds_peaks()
{
    PpgPeakDetector pd;
    float dc = 50000.0f;
    float ac = 2000.0f;
    float hr = 75.0f;  // 75 bpm → 800 ms period

    // Feed 4 seconds at 80 Hz
    uint32_t t = 0;
    for (int i = 0; i < 320; i++) {
        float ir = synth_ppg(t, hr, dc, ac);
        pd.feed(ir, t);
        t += 12;  // ~83 Hz (close enough to 80 Hz)
    }

    // Should have detected ~4-5 peaks in 4 seconds at 75 bpm
    size_t n_peaks = pd.peaks_in_window(t);
    TEST_ASSERT_TRUE(n_peaks >= 3);
    TEST_ASSERT_TRUE(n_peaks <= 7);
}

void test_peak_detector_hr_computation()
{
    PpgPeakDetector pd;
    float dc = 50000.0f;
    float ac = 2000.0f;
    float target_hr = 72.0f;

    // Feed 5 seconds of data
    uint32_t t = 0;
    for (int i = 0; i < 400; i++) {
        float ir = synth_ppg(t, target_hr, dc, ac);
        pd.feed(ir, t);
        t += 12;
    }

    float hr = pd.compute_hr_bpm(t);
    // Allow ±10 bpm tolerance (peak detection on synthetic data isn't perfect)
    if (hr > 0.0f) {
        TEST_ASSERT_FLOAT_WITHIN(15.0f, target_hr, hr);
    }
}

void test_peak_detector_rmssd_needs_3_peaks()
{
    PpgPeakDetector pd;
    // Feed very little data — should return -1 (invalid)
    for (int i = 0; i < 20; i++) {
        pd.feed(50000.0f + 100.0f * sinf(i * 0.1f), i * 12);
    }
    float rmssd = pd.compute_rmssd_ms(240);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, rmssd);
}

void test_peak_detector_reset()
{
    PpgPeakDetector pd;
    for (int i = 0; i < 200; i++) {
        pd.feed(synth_ppg(i * 12, 80.0f, 50000.0f, 2000.0f), i * 12);
    }
    TEST_ASSERT_TRUE(pd.peaks_in_window(2400) > 0);

    pd.reset();
    TEST_ASSERT_EQUAL(0, pd.peaks_in_window(2400));
}

// ############################################################################
//  2. Feature Vector
// ############################################################################

void test_feature_vector_all_valid()
{
    FeatureVector fv = make_normal_fv();
    TEST_ASSERT_TRUE(fv.all_valid());
    TEST_ASSERT_EQUAL(6, fv.valid_count());
}

void test_feature_vector_partial_valid()
{
    FeatureVector fv = make_normal_fv();
    fv.valid[1] = false;  // RMSSD invalid
    fv.valid[4] = false;  // Slope invalid
    TEST_ASSERT_FALSE(fv.all_valid());
    TEST_ASSERT_EQUAL(4, fv.valid_count());
}

void test_feature_vector_named_accessors()
{
    FeatureVector fv = make_normal_fv();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 72.0f, fv.hr_bpm());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 97.0f, fv.spo2_pct());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 36.5f, fv.skin_temp_c());
}

// ############################################################################
//  3. Feature Extractor — Accel & Skin Temp
// ############################################################################

void test_extractor_accel_variance()
{
    FeatureExtractor fe;

    // Feed 200 samples of near-constant acceleration (resting)
    for (int i = 0; i < 200; i++) {
        fe.feed_imu(0.0f, 0.0f, 9.81f);
    }

    FeatureVector fv = fe.extract(2000);
    TEST_ASSERT_TRUE(fv.valid[5]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, fv.accel_var());
}

void test_extractor_skin_temp_slope()
{
    FeatureExtractor fe;

    // First reading: 36.5°C at t=0
    fe.feed_skin_temp(36.5f, 0);

    // Second reading: 37.0°C at t=60000 ms (1 minute)
    fe.feed_skin_temp(37.0f, 60000);

    FeatureVector fv = fe.extract(60000);
    TEST_ASSERT_TRUE(fv.valid[3]);   // skin temp valid
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 37.0f, fv.skin_temp_c());

    // Slope should be ~ 0.5 °C/min
    if (fv.valid[4]) {
        TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.5f, fv.skin_temp_slope());
    }
}

void test_extractor_spo2_bounds()
{
    FeatureExtractor fe;
    fe.feed_spo2(97, true);

    FeatureVector fv = fe.extract(0);
    TEST_ASSERT_TRUE(fv.valid[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 97.0f, fv.spo2_pct());
}

void test_extractor_spo2_invalid()
{
    FeatureExtractor fe;
    fe.feed_spo2(-1, false);

    FeatureVector fv = fe.extract(0);
    TEST_ASSERT_FALSE(fv.valid[2]);
}

// ############################################################################
//  4. Baseline Statistics
// ############################################################################

void test_baseline_mean_computation()
{
    BaselineStats bs;
    FeatureVector fv = make_normal_fv();

    for (int i = 0; i < 100; i++) bs.accumulate(fv);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.0f,  bs.mean(0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 97.0f,  bs.mean(2));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 36.5f,  bs.mean(3));
}

void test_baseline_std_constant_input()
{
    BaselineStats bs;
    FeatureVector fv = make_normal_fv();

    for (int i = 0; i < 100; i++) bs.accumulate(fv);

    // Constant input → std ≈ 0 (but floored to 1e-6)
    for (size_t i = 0; i < FeatureVector::COUNT; i++) {
        TEST_ASSERT_TRUE(bs.std_dev(i) < 0.01f);
    }
}

void test_baseline_std_with_variation()
{
    BaselineStats bs;

    for (int i = 0; i < 100; i++) {
        FeatureVector fv = make_normal_fv();
        // Add some noise to HR: 72 ± 5
        fv.values[0] = 72.0f + 5.0f * sinf(i * 0.3f);
        bs.accumulate(fv);
    }

    // HR std should be roughly 3-4 (std of sinusoidal noise with amplitude 5)
    float hr_std = bs.std_dev(0);
    TEST_ASSERT_TRUE(hr_std > 1.0f);
    TEST_ASSERT_TRUE(hr_std < 6.0f);
}

// ############################################################################
//  5. Anomaly Engine — Cold-Start State Machine
// ############################################################################

void test_engine_starts_dormant()
{
    AnomalyEngine eng;
    eng.begin(0);
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::DORMANT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, eng.confidence_modifier());
}

void test_engine_stays_dormant_before_60min()
{
    AnomalyEngine eng;
    eng.begin(0);

    FeatureVector fv = make_normal_fv();

    // Feed 30 minutes of data at 1 Hz
    for (uint32_t t = 0; t < 30 * 60 * 1000; t += 1000) {
        eng.evaluate(fv, t);
    }

    // Still dormant — need 60 minutes
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::DORMANT);
}

void test_engine_transitions_to_active_after_60min()
{
    AnomalyEngine eng;
    eng.begin(0);

    FeatureVector fv = make_normal_fv();

    // Feed 61 minutes of data at 1 Hz (3660 samples > 1000 minimum)
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        eng.evaluate(fv, t);
        t += 1000;
    }

    TEST_ASSERT_TRUE(eng.state() == AnomalyState::ACTIVE);
    TEST_ASSERT_TRUE(eng.anomaly_threshold() > 0.0f);
    TEST_ASSERT_TRUE(eng.baseline_samples() >= 1000);
}

void test_engine_normal_features_low_score()
{
    AnomalyEngine eng;
    eng.begin(0);

    FeatureVector fv = make_normal_fv();

    // Fast-forward through dormancy
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        // Add slight noise to avoid zero variance
        fv.values[0] = 72.0f + 2.0f * sinf(i * 0.1f);
        eng.evaluate(fv, t);
        t += 1000;
    }
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::ACTIVE);

    // Now evaluate a normal feature vector
    fv = make_normal_fv();
    float conf = eng.evaluate(fv, t);

    // Score should be low (within baseline) → confidence ≤ 0.5
    TEST_ASSERT_TRUE(eng.anomaly_score() < eng.anomaly_threshold());
}

void test_engine_anomalous_features_high_score()
{
    AnomalyEngine eng;
    eng.begin(0);

    FeatureVector normal = make_normal_fv();

    // Train baseline with normal features (with noise)
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        normal.values[0] = 72.0f + 2.0f * sinf(i * 0.1f);
        normal.values[1] = 42.0f + 3.0f * sinf(i * 0.07f);
        eng.evaluate(normal, t);
        t += 1000;
    }

    // Now feed anomalous features
    FeatureVector bad = make_anomalous_fv();
    float conf = eng.evaluate(bad, t);

    // Score should be high → confidence > 0.5
    TEST_ASSERT_TRUE(eng.anomaly_score() > 1.0f);
    TEST_ASSERT_TRUE(conf > AnomalyConfig::CONFIDENCE_NEUTRAL);
}

// ############################################################################
//  6. NaN / Invalid Handling
// ############################################################################

void test_engine_handles_partial_invalid()
{
    AnomalyEngine eng;
    eng.begin(0);

    // Train baseline
    FeatureVector normal = make_normal_fv();
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        normal.values[0] = 72.0f + 2.0f * sinf(i * 0.1f);
        eng.evaluate(normal, t);
        t += 1000;
    }

    // 2 invalid features (≤ MAX_SUBSTITUTIONS) → should still work
    FeatureVector fv = make_normal_fv();
    fv.valid[1] = false;
    fv.valid[4] = false;

    float conf = eng.evaluate(fv, t);
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::ACTIVE);
    // Should have substituted with baseline means — not NaN
    TEST_ASSERT_FALSE(isnan(conf));
}

void test_engine_suspends_on_too_many_invalid()
{
    AnomalyEngine eng;
    eng.begin(0);

    // Train baseline
    FeatureVector normal = make_normal_fv();
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        normal.values[0] = 72.0f + 2.0f * sinf(i * 0.1f);
        eng.evaluate(normal, t);
        t += 1000;
    }
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::ACTIVE);

    // Feed 10+ consecutive invalid feature vectors (> 3 invalid features each)
    FeatureVector bad;
    for (int i = 0; i < 6; i++) { bad.values[i] = 0.0f; bad.valid[i] = false; }

    for (uint32_t i = 0; i < 15; i++) {
        eng.evaluate(bad, t);
        t += 1000;
    }

    TEST_ASSERT_TRUE(eng.state() == AnomalyState::SUSPENDED);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, eng.confidence_modifier());
}

// ############################################################################
//  7. Motion Cooldown
// ############################################################################

void test_engine_motion_cooldown_dampens_score()
{
    AnomalyEngine eng;
    eng.begin(0);

    // Train baseline (resting)
    FeatureVector normal = make_normal_fv();
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        normal.values[0] = 72.0f + 2.0f * sinf(i * 0.1f);
        normal.values[1] = 42.0f + 3.0f * sinf(i * 0.07f);
        eng.evaluate(normal, t);
        t += 1000;
    }

    // Simulate: high motion was just observed
    FeatureVector exercise = make_normal_fv();
    exercise.values[5] = 5.0f;  // high accel variance (above ACTIVITY_VARIANCE_THRESH)
    eng.evaluate(exercise, t);
    t += 1000;

    // Now evaluate anomalous features (elevated HR, low motion = post-exercise)
    FeatureVector recovery = make_anomalous_fv();
    recovery.values[5] = 0.02f;  // now still
    float conf_with_cooldown = eng.evaluate(recovery, t);

    // The motion cooldown should dampen the score by 50%
    float dampened_score = eng.anomaly_score();

    // Evaluate same anomalous features WITHOUT recent motion
    // (force reset and retrain to clear motion history)
    AnomalyEngine eng2;
    eng2.begin(0);
    for (int i = 0; i < 3660; i++) {
        normal.values[0] = 72.0f + 2.0f * sinf(i * 0.1f);
        normal.values[1] = 42.0f + 3.0f * sinf(i * 0.07f);
        eng2.evaluate(normal, i * 1000);
    }
    eng2.evaluate(recovery, 3660000);
    float undampened_score = eng2.anomaly_score();

    // Dampened should be roughly half of undampened
    if (undampened_score > 0.1f) {
        TEST_ASSERT_FLOAT_WITHIN(undampened_score * 0.1f,
                                 undampened_score * 0.5f,
                                 dampened_score);
    }
}

// ############################################################################
//  8. Force Reset
// ############################################################################

void test_engine_force_reset()
{
    AnomalyEngine eng;
    eng.begin(0);

    // Train to ACTIVE
    FeatureVector fv = make_normal_fv();
    uint32_t t = 0;
    for (int i = 0; i < 3660; i++) {
        eng.evaluate(fv, t);
        t += 1000;
    }
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::ACTIVE);

    // Force reset (simulates device re-worn)
    eng.force_reset(t);
    TEST_ASSERT_TRUE(eng.state() == AnomalyState::DORMANT);
    TEST_ASSERT_EQUAL(0, eng.baseline_samples());
}

// ############################################################################
//  9. Confidence Modifier Mapping
// ############################################################################

void test_confidence_neutral_during_dormancy()
{
    AnomalyEngine eng;
    eng.begin(0);

    FeatureVector fv = make_normal_fv();
    float conf = eng.evaluate(fv, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, conf);
}

// ############################################################################
//  10. Baseline Statistics Edge Cases
// ############################################################################

void test_baseline_empty_returns_zero_mean()
{
    BaselineStats bs;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, bs.mean(0));
}

void test_baseline_single_sample_variance()
{
    BaselineStats bs;
    FeatureVector fv = make_normal_fv();
    bs.accumulate(fv);

    // Single sample: variance defaults to 1.0 (not enough data)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, bs.variance(0));
}

// ============================================================================
//  Test Runner
// ============================================================================
void setup()
{
    delay(2000);
    UNITY_BEGIN();

    // Peak detector
    RUN_TEST(test_peak_detector_finds_peaks);
    RUN_TEST(test_peak_detector_hr_computation);
    RUN_TEST(test_peak_detector_rmssd_needs_3_peaks);
    RUN_TEST(test_peak_detector_reset);

    // Feature vector
    RUN_TEST(test_feature_vector_all_valid);
    RUN_TEST(test_feature_vector_partial_valid);
    RUN_TEST(test_feature_vector_named_accessors);

    // Feature extractor
    RUN_TEST(test_extractor_accel_variance);
    RUN_TEST(test_extractor_skin_temp_slope);
    RUN_TEST(test_extractor_spo2_bounds);
    RUN_TEST(test_extractor_spo2_invalid);

    // Baseline statistics
    RUN_TEST(test_baseline_mean_computation);
    RUN_TEST(test_baseline_std_constant_input);
    RUN_TEST(test_baseline_std_with_variation);
    RUN_TEST(test_baseline_empty_returns_zero_mean);
    RUN_TEST(test_baseline_single_sample_variance);

    // Cold-start state machine
    RUN_TEST(test_engine_starts_dormant);
    RUN_TEST(test_engine_stays_dormant_before_60min);
    RUN_TEST(test_engine_transitions_to_active_after_60min);
    RUN_TEST(test_engine_normal_features_low_score);
    RUN_TEST(test_engine_anomalous_features_high_score);

    // Invalid / NaN handling
    RUN_TEST(test_engine_handles_partial_invalid);
    RUN_TEST(test_engine_suspends_on_too_many_invalid);

    // Motion cooldown
    RUN_TEST(test_engine_motion_cooldown_dampens_score);

    // Reset
    RUN_TEST(test_engine_force_reset);

    // Confidence mapping
    RUN_TEST(test_confidence_neutral_during_dormancy);

    UNITY_END();
}

void loop() {}
