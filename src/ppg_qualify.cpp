#include "ppg_qualify.h"

// ============================================================================
//  PPG Qualifier — Implementation
// ============================================================================

void PpgQualifier::feed_ppg(uint32_t ir_amplitude)
{
    bool had_contact = contact_;
    contact_ = (ir_amplitude >= SafetyConfig::PPG_IR_CONTACT_THRESHOLD);

    if (!contact_) {
        // Contact lost — reset calibration counter.  When the finger/wrist
        // comes back the algorithm needs a fresh 100-sample fill.
        sample_count_ = 0;
        state_ = PpgState::CONTACT_LOST;
        return;
    }

    // Contact just regained after a loss — restart fill counter
    if (!had_contact) {
        sample_count_ = 0;
    }

    if (sample_count_ < SafetyConfig::PPG_BUFFER_FILL_SAMPLES) {
        sample_count_++;
    }

    evaluate_state();
}

void PpgQualifier::set_motion_variance(float variance)
{
    motion_var_ = variance;

    // Re-evaluate only when we have enough samples and contact
    if (contact_ && sample_count_ >= SafetyConfig::PPG_BUFFER_FILL_SAMPLES) {
        evaluate_state();
    }
}

void PpgQualifier::evaluate_state()
{
    if (!contact_) {
        state_ = PpgState::CONTACT_LOST;
    } else if (sample_count_ < SafetyConfig::PPG_BUFFER_FILL_SAMPLES) {
        state_ = PpgState::CALIBRATING;
    } else if (motion_var_ > SafetyConfig::MOTION_VARIANCE_THRESHOLD) {
        state_ = PpgState::MOTION_ARTIFACT;
    } else {
        state_ = PpgState::VALID;
    }
}

void PpgQualifier::reset()
{
    state_        = PpgState::CALIBRATING;
    sample_count_ = 0;
    contact_      = false;
    motion_var_   = 0.0f;
}
