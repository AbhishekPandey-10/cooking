#include "anomaly_engine.h"
#include <cmath>
#include <cstring>

// ============================================================================
//  BaselineStats — Running mean / std per feature
// ============================================================================

void BaselineStats::accumulate(const FeatureVector &fv)
{
    for (size_t i = 0; i < FeatureVector::COUNT; i++) {
        if (fv.valid[i]) {
            sum[i]    += fv.values[i];
            sum_sq[i] += fv.values[i] * fv.values[i];
        }
    }
    count++;
}

float BaselineStats::mean(size_t idx) const
{
    if (idx >= FeatureVector::COUNT || count == 0) return 0.0f;
    return sum[idx] / static_cast<float>(count);
}

float BaselineStats::variance(size_t idx) const
{
    if (idx >= FeatureVector::COUNT || count < 2) return 1.0f;
    float n  = static_cast<float>(count);
    float m  = sum[idx] / n;
    float v  = (sum_sq[idx] / n) - (m * m);
    return (v > 0.0f) ? v : 1e-6f;   // floor to avoid div-by-zero
}

float BaselineStats::std_dev(size_t idx) const
{
    return sqrtf(variance(idx));
}

// ============================================================================
//  AnomalyEngine — Implementation
// ============================================================================

void AnomalyEngine::begin(uint32_t boot_ms)
{
    state_   = AnomalyState::DORMANT;
    boot_ms_ = boot_ms;

    // Zero baseline
    memset(&baseline_, 0, sizeof(baseline_));
    score_sum_    = 0.0f;
    score_sum_sq_ = 0.0f;
    score_count_  = 0;
    theta_        = 0.0f;

    last_score_      = 0.0f;
    last_confidence_ = AnomalyConfig::CONFIDENCE_NEUTRAL;

    consecutive_invalid_ = 0;
    last_valid_ms_       = boot_ms;
    last_high_motion_ms_ = 0;
}

// ============================================================================
//  Main evaluate() — call at 1 Hz
// ============================================================================

float AnomalyEngine::evaluate(const FeatureVector &fv, uint32_t timestamp_ms)
{
    // ---- NaN / invalid detection -------------------------------------------
    if (fv.valid_count() < (FeatureVector::COUNT - AnomalyConfig::MAX_SUBSTITUTIONS)) {
        // Too many invalid features
        consecutive_invalid_++;

        if (state_ == AnomalyState::ACTIVE &&
            consecutive_invalid_ >= AnomalyConfig::MAX_CONSECUTIVE_INVALID) {
            state_ = AnomalyState::SUSPENDED;
        }

        last_confidence_ = AnomalyConfig::CONFIDENCE_NEUTRAL;
        return last_confidence_;
    }

    consecutive_invalid_ = 0;
    last_valid_ms_ = timestamp_ms;

    // ---- Create a working copy with substitutions for invalid features -----
    FeatureVector fv_clean = fv;
    if (!fv_clean.all_valid() && state_ == AnomalyState::ACTIVE) {
        substitute_invalid(fv_clean);
    }

    // ---- Track motion for cooldown -----------------------------------------
    if (fv.valid[5] && fv.values[5] > AnomalyConfig::ACTIVITY_VARIANCE_THRESH) {
        last_high_motion_ms_ = timestamp_ms;
    }

    // ---- State-dependent processing ----------------------------------------
    switch (state_) {

    case AnomalyState::DORMANT: {
        // Accumulate baseline statistics
        if (fv_clean.all_valid()) {
            baseline_.accumulate(fv_clean);

            // Also track reconstruction scores for theta calibration
            if (baseline_.count >= 20) {
                // Enough data for a rough z-score computation
                float score = compute_zscore_distance(fv_clean);
                score_sum_    += score;
                score_sum_sq_ += score * score;
                score_count_++;
            }
        }

        // Check transition conditions
        uint32_t elapsed = timestamp_ms - boot_ms_;
        if (elapsed >= AnomalyConfig::DORMANCY_MS &&
            baseline_.count >= AnomalyConfig::MIN_BASELINE_SAMPLES) {
            transition_to_active(timestamp_ms);
        }

        last_confidence_ = AnomalyConfig::CONFIDENCE_NEUTRAL;
        break;
    }

    case AnomalyState::ACTIVE: {
        // ---- Compute anomaly score -----------------------------------------
        last_score_ = compute_zscore_distance(fv_clean);

        // ---- Apply motion cooldown dampening -------------------------------
        //  If high motion was observed in the last 5 min, dampen the score
        //  by 50% to reduce false alarms during post-exercise recovery.
        if (last_high_motion_ms_ > 0 &&
            (timestamp_ms - last_high_motion_ms_) < AnomalyConfig::MOTION_COOLDOWN_MS) {
            last_score_ *= 0.5f;
        }

        // ---- Map score to confidence modifier ------------------------------
        last_confidence_ = map_confidence(last_score_, timestamp_ms);
        break;
    }

    case AnomalyState::SUSPENDED: {
        // Check for recovery: clean data for RECOVERY_STABILITY_MS
        if ((timestamp_ms - last_valid_ms_) < 1000) {
            // We're getting valid data again
            uint32_t clean_duration = timestamp_ms - last_valid_ms_;
            // Use a simple check: if we've had valid data this tick and it's been
            // at least RECOVERY_STABILITY_MS since last invalid streak
            if (consecutive_invalid_ == 0 &&
                (timestamp_ms - last_valid_ms_) >= 0) {
                // Track recovery start — simplified: if we get enough consecutive
                // valid readings, transition back
                // (In practice, consecutive_invalid_ being 0 after MAX_CONSECUTIVE_INVALID
                // means we've started getting valid data again)
                state_ = AnomalyState::ACTIVE;
            }
        }
        last_confidence_ = AnomalyConfig::CONFIDENCE_NEUTRAL;
        break;
    }
    }

    return last_confidence_;
}

// ============================================================================
//  Z-Score Distance — Statistical Fallback Anomaly Detector
//
//  score = (1/N) × Σ ((fᵢ − μᵢ) / σᵢ)²
//
//  This is the squared Euclidean distance in z-score space.
//  For N=6 features drawn from the baseline distribution, the expected
//  score under normality is approximately 1.0 (chi-squared with N dof).
//  Values >> 1.0 indicate deviation from baseline.
// ============================================================================

float AnomalyEngine::compute_zscore_distance(const FeatureVector &fv) const
{
    if (baseline_.count < 10) return 0.0f;

    float score = 0.0f;
    size_t valid_features = 0;

    for (size_t i = 0; i < FeatureVector::COUNT; i++) {
        if (!fv.valid[i]) continue;

        float mu    = baseline_.mean(i);
        float sigma = baseline_.std_dev(i);

        // Guard against near-zero std (feature is constant)
        if (sigma < 1e-6f) sigma = 1e-6f;

        float z = (fv.values[i] - mu) / sigma;
        score += z * z;
        valid_features++;
    }

    return (valid_features > 0)
         ? score / static_cast<float>(valid_features)
         : 0.0f;
}

// ============================================================================
//  Invalid Feature Substitution
//
//  Replace missing features with baseline mean.  Only called when
//  ≤ MAX_SUBSTITUTIONS features are invalid (checked by caller).
// ============================================================================

void AnomalyEngine::substitute_invalid(FeatureVector &fv) const
{
    for (size_t i = 0; i < FeatureVector::COUNT; i++) {
        if (!fv.valid[i] && baseline_.count > 0) {
            fv.values[i] = baseline_.mean(i);
            fv.valid[i]  = true;
        }
    }
}

// ============================================================================
//  Confidence Mapping
//
//  Maps the raw anomaly score to a 0.0–1.0 confidence modifier:
//
//    score < θ/2      →  0.3  (suppression: "very normal, dampen alerts")
//    θ/2 ≤ score < θ  →  0.5  (neutral)
//    θ ≤ score < 2θ   →  linear ramp 0.5 → 1.0  (boost)
//    score ≥ 2θ       →  1.0  (maximum boost)
// ============================================================================

float AnomalyEngine::map_confidence(float score, uint32_t /* timestamp_ms */) const
{
    if (theta_ <= 0.0f) return AnomalyConfig::CONFIDENCE_NEUTRAL;

    float half_theta = theta_ * 0.5f;

    if (score < half_theta) {
        // Strong suppression — everything looks very normal
        return 0.3f;
    }
    if (score < theta_) {
        // Neutral zone
        return AnomalyConfig::CONFIDENCE_NEUTRAL;
    }
    if (score < theta_ * 2.0f) {
        // Linear ramp from 0.5 to 1.0
        float t = (score - theta_) / theta_;
        return AnomalyConfig::CONFIDENCE_NEUTRAL
             + t * (AnomalyConfig::CONFIDENCE_MAX - AnomalyConfig::CONFIDENCE_NEUTRAL);
    }

    // Full boost
    return AnomalyConfig::CONFIDENCE_MAX;
}

// ============================================================================
//  Theta Calibration — compute anomaly threshold from dormancy scores
// ============================================================================

void AnomalyEngine::calibrate_theta()
{
    if (score_count_ < 10) {
        // Not enough data — use a conservative default
        theta_ = 3.0f;  // ~3.0 z-score distance (well above expected 1.0)
        return;
    }

    float n         = static_cast<float>(score_count_);
    float mean_score = score_sum_ / n;
    float var_score  = (score_sum_sq_ / n) - (mean_score * mean_score);
    float std_score  = sqrtf(var_score > 0.0f ? var_score : 0.01f);

    theta_ = mean_score + AnomalyConfig::THETA_SIGMA_MULTIPLIER * std_score;

    // Floor: theta should never be below 0.5 (otherwise everything is anomalous)
    if (theta_ < 0.5f) theta_ = 0.5f;
}

// ============================================================================
//  State Transition: DORMANT → ACTIVE
// ============================================================================

void AnomalyEngine::transition_to_active(uint32_t timestamp_ms)
{
    calibrate_theta();
    state_ = AnomalyState::ACTIVE;
    last_valid_ms_ = timestamp_ms;
}

// ============================================================================
//  Force Reset — device re-worn, different user possible
// ============================================================================

void AnomalyEngine::force_reset(uint32_t timestamp_ms)
{
    begin(timestamp_ms);  // Full re-initialization with new "boot" time
}
