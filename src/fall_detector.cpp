#include "fall_detector.h"
#include <cmath>

// ============================================================================
//  Fall Detector — State Machine Implementation
// ============================================================================

void FallDetector::feed(float ax, float ay, float az, uint32_t timestamp_ms)
{
    float amag   = sqrtf(ax * ax + ay * ay + az * az);
    float amag_g = amag / SafetyConfig::G_MPS2;

    switch (state_) {

    // ------------------------------------------------------------------ IDLE
    case FallState::IDLE:
        if (amag_g < SafetyConfig::FREEFALL_THRESHOLD_G) {
            state_             = FallState::FREEFALL_DETECTED;
            freefall_start_ms_ = timestamp_ms;
        }
        break;

    // --------------------------------------------------- FREEFALL_DETECTED
    case FallState::FREEFALL_DETECTED:
        if (amag_g > SafetyConfig::IMPACT_THRESHOLD_G) {
            // Impact within the 300 ms window → advance
            state_              = FallState::IMPACT_DETECTED;
            impact_time_ms_     = timestamp_ms;
            immobility_var_.reset();
            immobility_start_ms_ = 0;
            immobility_tracking_ = false;
        } else if ((timestamp_ms - freefall_start_ms_) >
                   SafetyConfig::FREEFALL_TO_IMPACT_MS) {
            // No impact arrived in time — probably not a fall
            state_ = FallState::IDLE;
        }
        break;

    // ---------------------------------------------------  IMPACT_DETECTED
    case FallState::IMPACT_DETECTED: {
        immobility_var_.push(amag);

        if (immobility_var_.full()) {
            float var = immobility_var_.variance();

            if (var < SafetyConfig::IMMOBILITY_VARIANCE_THRESH) {
                // Acceleration variance is very low → person is still
                if (!immobility_tracking_) {
                    immobility_tracking_  = true;
                    immobility_start_ms_  = timestamp_ms;
                } else if ((timestamp_ms - immobility_start_ms_) >=
                           SafetyConfig::IMMOBILITY_DURATION_MS) {
                    state_ = FallState::FALL_CONFIRMED;
                }
            } else {
                // Movement detected — reset immobility timer
                immobility_tracking_ = false;
                immobility_start_ms_ = 0;

                // If sustained movement persists for 15 s post-impact the
                // person likely recovered — return to IDLE.
                if ((timestamp_ms - impact_time_ms_) > 15000) {
                    state_ = FallState::IDLE;
                    immobility_var_.reset();
                }
            }
        }
        break;
    }

    // ---------------------------------------------------  FALL_CONFIRMED
    case FallState::FALL_CONFIRMED:
        // Remain here until explicitly reset by the application.
        break;
    }
}

void FallDetector::reset()
{
    state_               = FallState::IDLE;
    freefall_start_ms_   = 0;
    impact_time_ms_      = 0;
    immobility_var_.reset();
    immobility_start_ms_ = 0;
    immobility_tracking_ = false;
}
