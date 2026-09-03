#ifndef FALL_DETECTOR_H
#define FALL_DETECTOR_H

#include "safety_types.h"

// ============================================================================
//  Two-Stage Fall Detector
//
//  State machine:
//
//    IDLE ──(|a| < 0.2 g)──► FREEFALL_DETECTED
//                                  │
//                    ┌─(300 ms timeout)──► IDLE
//                    │
//                (|a| > 3.0 g within 300 ms)
//                    │
//                    ▼
//            IMPACT_DETECTED ──(variance < thresh for 5 s)──► FALL_CONFIRMED
//                    │                                              │
//            (movement resumes                              (stays until
//             + 15 s timeout)──► IDLE                        explicit reset)
//
//  All timestamps are passed in externally so the module is fully testable
//  without hardware timers.
// ============================================================================

class FallDetector {
public:
    /// Feed one accelerometer sample.
    /// @param ax, ay, az   Acceleration in m/s² (not g)
    /// @param timestamp_ms Monotonic timestamp
    void feed(float ax, float ay, float az, uint32_t timestamp_ms);

    FallState state()             const { return state_; }
    bool      is_fall_confirmed() const { return state_ == FallState::FALL_CONFIRMED; }

    /// Reset to IDLE (e.g. after the alert is acknowledged).
    void reset();

private:
    FallState state_ = FallState::IDLE;

    uint32_t freefall_start_ms_ = 0;
    uint32_t impact_time_ms_    = 0;

    // Post-impact immobility: 1 s rolling variance window (100 samples @ 100 Hz)
    RollingVariance<100> immobility_var_;
    uint32_t immobility_start_ms_ = 0;
    bool     immobility_tracking_ = false;
};

#endif // FALL_DETECTOR_H
