#ifndef SAFETY_TYPES_H
#define SAFETY_TYPES_H

#include <cstdint>
#include <cstddef>

// ============================================================================
//  Safety Engine — Type Definitions & Utilities
//
//  Pure data types and a templated rolling-variance calculator.
//  No hardware dependencies — safe for desktop unit testing.
// ============================================================================

// ---- Alert Severity --------------------------------------------------------
enum class AlertSeverity : uint8_t {
    NONE = 0,
    WARNING,        // Needs attention
    CRITICAL        // Immediate danger
};

// ---- Alert Types -----------------------------------------------------------
enum class AlertType : uint8_t {
    NONE = 0,
    SPO2_LOW,       // SpO2 < 92% sustained ≥ 30 s
    SPO2_CRITICAL,  // SpO2 < 90% (immediate)
    FALL_DETECTED,  // Freefall + impact + immobility
    HEAT_STRESS,    // Heat index in NWS "Danger" band
    HEAT_DANGER     // Heat index in NWS "Extreme Danger" band
};

// ---- PPG Signal Quality State ----------------------------------------------
enum class PpgState : uint8_t {
    CALIBRATING,     // Ring buffer filling (< 100 samples) or re-filling
    CONTACT_LOST,    // IR amplitude < 5 000 counts
    MOTION_ARTIFACT, // Accel variance too high for reliable optical read
    VALID            // Good signal — alerting enabled
};

// ---- Fall Detector State Machine -------------------------------------------
enum class FallState : uint8_t {
    IDLE,                // Monitoring for freefall
    FREEFALL_DETECTED,   // |a| < 0.2 g — watching for impact within 300 ms
    IMPACT_DETECTED,     // |a| > 3.0 g — monitoring immobility
    FALL_CONFIRMED       // Immobility confirmed — alert raised
};

// ---- Active Alert ----------------------------------------------------------
struct Alert {
    AlertType     type;
    AlertSeverity severity;
    const char*   message;
};

// ============================================================================
//  Tunable Constants (Safety Config)
// ============================================================================
namespace SafetyConfig {

    // PPG qualification
    constexpr uint32_t PPG_BUFFER_FILL_SAMPLES   = 100;
    constexpr uint32_t PPG_IR_CONTACT_THRESHOLD  = 5000;
    constexpr float    MOTION_VARIANCE_THRESHOLD  = 0.3f;  // (m/s²)²

    // SpO2 thresholds
    constexpr int32_t  SPO2_WARNING_THRESHOLD    = 92;     // %
    constexpr int32_t  SPO2_CRITICAL_THRESHOLD   = 90;     // %
    constexpr uint32_t SPO2_SUSTAINED_MS         = 30000;  // 30 s

    // Fall detection
    constexpr float    FREEFALL_THRESHOLD_G      = 0.2f;   // g
    constexpr float    IMPACT_THRESHOLD_G         = 3.0f;   // g
    constexpr uint32_t FREEFALL_TO_IMPACT_MS     = 300;    // ms
    constexpr uint32_t IMMOBILITY_DURATION_MS    = 5000;   // 5 s (see rationale below)
    constexpr float    IMMOBILITY_VARIANCE_THRESH = 0.05f; // (m/s²)²
    //  5 s rationale:
    //    - Short enough for timely detection (literature uses 3–10 s).
    //    - Long enough to reject normal pauses (stopping to check phone,
    //      sitting down quickly).  A genuinely incapacitated person will
    //      remain immobile far longer than 5 s.
    //    - Matches the "post-impact orientation" window observed in
    //      clinical fall studies using tri-axial accelerometers.

    // Heat index thresholds (°C output from heat_index_celsius())
    constexpr float    HEAT_WARNING_C  = 40.0f;   // NWS "Danger" onset
    constexpr float    HEAT_CRITICAL_C = 54.0f;   // NWS "Extreme Danger"

    // Physical constant
    constexpr float    G_MPS2 = 9.80665f;
}

// ============================================================================
//  RollingVariance<N> — fixed-size ring buffer for online mean / variance
//
//  Uses the naïve two-pass-equivalent formula:
//      Var = E[X²] − (E[X])²
//  which is O(1) per push().  The template parameter N is the window size
//  in samples (e.g. 50 for a 0.5 s window at 100 Hz).
// ============================================================================
template <size_t N>
class RollingVariance {
public:
    void push(float value) {
        if (count_ >= N) {
            // Evict oldest value
            float old = buf_[head_];
            sum_    -= old;
            sum_sq_ -= old * old;
        } else {
            count_++;
        }
        buf_[head_] = value;
        sum_    += value;
        sum_sq_ += value * value;
        head_ = (head_ + 1) % N;
    }

    float variance() const {
        if (count_ < 2) return 0.0f;
        float n    = static_cast<float>(count_);
        float mean = sum_ / n;
        float var  = (sum_sq_ / n) - (mean * mean);
        return (var > 0.0f) ? var : 0.0f;   // clamp floating-point undershoot
    }

    float mean() const {
        return (count_ == 0) ? 0.0f : sum_ / static_cast<float>(count_);
    }

    size_t count() const { return count_; }
    bool   full()  const { return count_ >= N; }

    void reset() {
        head_   = 0;
        count_  = 0;
        sum_    = 0.0f;
        sum_sq_ = 0.0f;
        for (size_t i = 0; i < N; i++) buf_[i] = 0.0f;
    }

private:
    float  buf_[N] = {};
    size_t head_   = 0;
    size_t count_  = 0;
    float  sum_    = 0.0f;
    float  sum_sq_ = 0.0f;
};

#endif // SAFETY_TYPES_H
