#ifndef PPG_QUALIFY_H
#define PPG_QUALIFY_H

#include "safety_types.h"

// ============================================================================
//  PPG Signal Qualifier
//
//  Tracks three independent quality gates:
//
//    1. Buffer fill   – The SpO2 algorithm needs 100 samples to populate its
//                       ring buffer.  Until then, SpO2 values are meaningless.
//
//    2. Skin contact  – IR amplitude must be ≥ 5 000 counts.  Falling below
//                       this resets the buffer-fill counter so the 100-sample
//                       window restarts on contact regain.
//
//    3. Motion artifact – If the MPU6050 accel variance exceeds 0.3 (m/s²)²,
//                         optical readings are unreliable.
//
//  The resulting PpgState drives both OLED status text and Tier 1 alert
//  suppression logic.
// ============================================================================

class PpgQualifier {
public:
    /// Feed a raw IR amplitude from the MAX30102.
    void feed_ppg(uint32_t ir_amplitude);

    /// Update the current motion variance (computed externally from IMU data).
    void set_motion_variance(float variance);

    PpgState state()        const { return state_; }
    bool     is_valid()     const { return state_ == PpgState::VALID; }
    uint32_t sample_count() const { return sample_count_; }

    void reset();

private:
    PpgState state_        = PpgState::CALIBRATING;
    uint32_t sample_count_ = 0;
    bool     contact_      = false;
    float    motion_var_   = 0.0f;

    void evaluate_state();
};

#endif // PPG_QUALIFY_H
