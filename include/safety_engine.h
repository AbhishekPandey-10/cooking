#ifndef SAFETY_ENGINE_H
#define SAFETY_ENGINE_H

#include "safety_types.h"
#include "fall_detector.h"
#include "ppg_qualify.h"

// Maximum concurrent active alerts
constexpr size_t MAX_ACTIVE_ALERTS = 8;

// ============================================================================
//  SafetyEngine — Tier 1 Deterministic Alert Orchestrator
//
//  Owns the sub-modules (PpgQualifier, FallDetector) and performs threshold
//  evaluation on their outputs.  All inputs are raw sensor values passed via
//  feed_*() methods — the engine has zero coupling to sensor drivers.
//
//  Alert suppression hierarchy:
//    • PPG not VALID  →  suppress all SpO2 alerts
//    • Motion artifact →  suppress SpO2, flag on OLED
//    • SpO2 < 90 %    →  immediate CRITICAL (regardless of sustained timer)
//    • SpO2 < 92 %    →  WARNING after 30 s sustained
//    • Fall confirmed  →  CRITICAL
//    • Heat index ≥ 54 °C → CRITICAL  |  ≥ 40 °C → WARNING
// ============================================================================

class SafetyEngine {
public:
    // ---- Feed methods (call from sensor callbacks / scheduler) --------------

    /// Feed PPG data.
    /// @param ir           Raw IR amplitude from MAX30102
    /// @param spo2         Computed SpO2 percentage (0–100), or −1 if invalid
    /// @param spo2_valid   True if the SpO2 algorithm produced a usable value
    /// @param timestamp_ms Monotonic timestamp
    void feed_ppg(uint32_t ir, int32_t spo2, bool spo2_valid,
                  uint32_t timestamp_ms);

    /// Feed IMU accelerometer data.
    void feed_imu(float ax, float ay, float az, uint32_t timestamp_ms);

    /// Feed environmental data (air temp + RH for heat index).
    void feed_environment(float air_temp_c, float humidity_pct,
                          uint32_t timestamp_ms);

    // ---- Query methods -----------------------------------------------------

    PpgState  ppg_state()        const { return ppg_qual_.state(); }
    FallState fall_state()       const { return fall_det_.state(); }
    bool      is_fall_detected() const { return fall_det_.is_fall_confirmed(); }
    float     last_heat_index()  const { return last_heat_index_c_; }
    int32_t   last_spo2()        const { return last_spo2_; }

    /// OLED status string for the PPG/vitals line.
    const char* ppg_status_string() const;

    /// Active alerts.
    size_t       alert_count() const { return alert_count_; }
    const Alert* alerts()      const { return alerts_; }

    /// Acknowledge a fall (resets fall detector to IDLE).
    void reset_fall() { fall_det_.reset(); }

private:
    // Sub-modules
    PpgQualifier ppg_qual_;
    FallDetector fall_det_;

    // Motion variance for PPG gating — 50 samples @ 100 Hz = 0.5 s window
    RollingVariance<50> motion_var_;

    // SpO2 sustained-low tracking
    int32_t  last_spo2_              = -1;
    uint32_t spo2_below_warn_since_  = 0;
    bool     spo2_below_warn_        = false;

    // Heat index
    float last_heat_index_c_ = 0.0f;

    // Alert array (rebuilt on every feed)
    Alert  alerts_[MAX_ACTIVE_ALERTS] = {};
    size_t alert_count_               = 0;

    void rebuild_alerts(uint32_t timestamp_ms);
    void add_alert(AlertType type, AlertSeverity sev, const char* msg);
};

#endif // SAFETY_ENGINE_H
