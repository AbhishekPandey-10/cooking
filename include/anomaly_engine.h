#ifndef ANOMALY_ENGINE_H
#define ANOMALY_ENGINE_H

#include <cstdint>
#include <cstddef>
#include <cmath>
#include "feature_extract.h"

// ============================================================================
//  Anomaly Engine — Cold-Start + Inference + Confidence Modifier
//
//  State machine:
//    DORMANT   → first 60 min, accumulating baseline (mean/std per feature)
//    ACTIVE    → running inference, outputting confidence modifier
//    SUSPENDED → sensor failure / NaN storm, reverts to neutral
//
//  During DORMANT and SUSPENDED, the system relies 100% on Tier 1
//  deterministic rules and the Phase 4.5 correlation engine.
//  The anomaly engine outputs a neutral confidence of 0.5.
//
//  Inference backend:
//    #define USE_EDGE_IMPULSE  → use EI run_classifier()
//    (default)                 → statistical fallback (z-score distance)
//
//  The statistical fallback computes:
//    score = (1/N) × Σ ((fᵢ − μᵢ) / σᵢ)²
//  which is the squared Euclidean distance in z-score space.  This acts
//  as a "poor man's autoencoder" that detects obvious outliers without a
//  trained model.  It shares the same interface as the EI backend, so
//  upgrading is a drop-in replacement.
// ============================================================================

namespace AnomalyConfig {
    // Cold-start dormancy period
    constexpr uint32_t DORMANCY_MS             = 60UL * 60UL * 1000UL;  // 60 minutes
    constexpr uint32_t MIN_BASELINE_SAMPLES    = 1000;    // ~17 min at 1 Hz

    // NaN / invalid handling
    constexpr uint32_t MAX_CONSECUTIVE_INVALID  = 10;     // vectors → SUSPENDED
    constexpr uint32_t MAX_SUBSTITUTIONS        = 2;      // features per vector
    constexpr uint32_t RECOVERY_STABILITY_MS    = 30000;  // 30 s clean → ACTIVE

    // Motion cooldown (post-exercise dampening)
    constexpr float    ACTIVITY_VARIANCE_THRESH = 2.0f;   // (m/s²)² threshold
    constexpr uint32_t MOTION_COOLDOWN_MS       = 5UL * 60UL * 1000UL;  // 5 min

    // Contact loss detection
    constexpr uint32_t CONTACT_LOSS_RESET_MS    = 60000;  // 60 s no contact → re-baseline

    // Anomaly threshold multiplier (σ above baseline mean MSE)
    constexpr float    THETA_SIGMA_MULTIPLIER   = 3.0f;

    // Confidence modifier output mapping
    constexpr float    CONFIDENCE_NEUTRAL       = 0.5f;
    constexpr float    CONFIDENCE_MIN           = 0.0f;
    constexpr float    CONFIDENCE_MAX           = 1.0f;
}

// ============================================================================
//  Engine State
// ============================================================================
enum class AnomalyState : uint8_t {
    DORMANT,     // Accumulating baseline, no inference
    ACTIVE,      // Running inference
    SUSPENDED    // Sensor failure, fallback to deterministic only
};

// ============================================================================
//  Baseline Statistics — per-feature running mean and variance
// ============================================================================
struct BaselineStats {
    float sum[FeatureVector::COUNT]    = {};
    float sum_sq[FeatureVector::COUNT] = {};
    uint32_t count = 0;

    void accumulate(const FeatureVector &fv);

    float mean(size_t idx)     const;
    float std_dev(size_t idx)  const;
    float variance(size_t idx) const;
};

// ============================================================================
//  AnomalyEngine
// ============================================================================
class AnomalyEngine {
public:
    /// Call once at boot.
    void begin(uint32_t boot_ms);

    /// Evaluate a feature vector.  Call at 1 Hz.
    /// Returns the confidence modifier (0.0–1.0):
    ///   0.0–0.4 → suppression  (autoencoder says "this looks very normal")
    ///   0.5     → neutral      (dormant / suspended / uncertain)
    ///   0.6–1.0 → boost        (autoencoder detects baseline deviation)
    float evaluate(const FeatureVector &fv, uint32_t timestamp_ms);

    // ---- Query methods -----------------------------------------------------
    AnomalyState state()               const { return state_; }
    float        anomaly_score()       const { return last_score_; }
    float        anomaly_threshold()   const { return theta_; }
    bool         is_anomalous()        const { return state_ == AnomalyState::ACTIVE
                                                   && last_score_ > theta_; }
    float        confidence_modifier() const { return last_confidence_; }

    uint32_t     baseline_samples()    const { return baseline_.count; }
    float        baseline_mean(size_t i) const { return baseline_.mean(i); }
    float        baseline_std(size_t i)  const { return baseline_.std_dev(i); }

    /// Force a reset to DORMANT (e.g., device re-worn after contact loss).
    void force_reset(uint32_t timestamp_ms);

private:
    AnomalyState state_     = AnomalyState::DORMANT;
    uint32_t     boot_ms_   = 0;

    // Baseline statistics
    BaselineStats baseline_;

    // Anomaly threshold (calibrated after dormancy)
    float theta_ = 0.0f;

    // Running anomaly score tracking for theta calibration
    float score_sum_    = 0.0f;
    float score_sum_sq_ = 0.0f;
    uint32_t score_count_ = 0;

    // Latest outputs
    float last_score_      = 0.0f;
    float last_confidence_ = AnomalyConfig::CONFIDENCE_NEUTRAL;

    // NaN / invalid tracking
    uint32_t consecutive_invalid_ = 0;
    uint32_t last_valid_ms_       = 0;

    // Motion cooldown tracking
    uint32_t last_high_motion_ms_ = 0;

    // ---- Internal methods --------------------------------------------------
    float compute_zscore_distance(const FeatureVector &fv) const;
    void  substitute_invalid(FeatureVector &fv) const;
    float map_confidence(float score, uint32_t timestamp_ms) const;
    void  calibrate_theta();
    void  transition_to_active(uint32_t timestamp_ms);
};

#endif // ANOMALY_ENGINE_H
