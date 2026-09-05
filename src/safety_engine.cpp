#include "safety_engine.h"
#include "heat_index.h"
#include <cmath>

// ============================================================================
//  SafetyEngine — Implementation
// ============================================================================

// ---- Feed: PPG + SpO2 ------------------------------------------------------
void SafetyEngine::feed_ppg(uint32_t ir, int32_t spo2, bool spo2_valid,
                            uint32_t timestamp_ms)
{
    // Update PPG quality gate
    ppg_qual_.feed_ppg(ir);

    // Process SpO2 only when the optical signal is trustworthy
    if (ppg_qual_.is_valid() && spo2_valid && spo2 > 0) {
        last_spo2_ = spo2;

        // ---- Sustained-low tracking ----------------------------------------
        if (spo2 < SafetyConfig::SPO2_WARNING_THRESHOLD) {
            if (!spo2_below_warn_) {
                spo2_below_warn_       = true;
                spo2_below_warn_since_ = timestamp_ms;
            }
        } else {
            // Recovered above threshold — reset the clock
            spo2_below_warn_       = false;
            spo2_below_warn_since_ = 0;
        }
    }

    rebuild_alerts(timestamp_ms);
}

// ---- Feed: IMU -------------------------------------------------------------
void SafetyEngine::feed_imu(float ax, float ay, float az,
                            uint32_t timestamp_ms)
{
    // Fall detection state machine
    fall_det_.feed(ax, ay, az, timestamp_ms);

    // Motion variance for PPG gating (rolling 0.5 s window)
    float amag = sqrtf(ax * ax + ay * ay + az * az);
    motion_var_.push(amag);
    ppg_qual_.set_motion_variance(motion_var_.variance());

    rebuild_alerts(timestamp_ms);
}

// ---- Feed: Environment -----------------------------------------------------
void SafetyEngine::feed_environment(float air_temp_c, float humidity_pct,
                                    uint32_t timestamp_ms)
{
    last_heat_index_c_ = heat_index_celsius(air_temp_c, humidity_pct);
    rebuild_alerts(timestamp_ms);
}

// ---- OLED PPG Status String -------------------------------------------------
const char* SafetyEngine::ppg_status_string() const
{
    switch (ppg_qual_.state()) {
    case PpgState::CALIBRATING:     return "Calibrating Vitals...";
    case PpgState::CONTACT_LOST:    return "Calibrating Vitals...";
    case PpgState::MOTION_ARTIFACT: return "Keep still for vitals";
    case PpgState::VALID:           return "Vitals OK";
    default:                        return "Unknown";
    }
}

// ---- Alert Rebuild ----------------------------------------------------------
//  Called after every feed.  Clears and reconstructs the active alert list
//  so it always reflects the current state.
void SafetyEngine::rebuild_alerts(uint32_t timestamp_ms)
{
    alert_count_ = 0;

    // ---- SpO2 alerts (gated by PPG quality) --------------------------------
    if (ppg_qual_.is_valid() && last_spo2_ > 0) {

        if (last_spo2_ < SafetyConfig::SPO2_CRITICAL_THRESHOLD) {
            // Immediate escalation: SpO2 < 90 %
            add_alert(AlertType::SPO2_CRITICAL, AlertSeverity::CRITICAL,
                      "SpO2 CRITICAL < 90%");

        } else if (spo2_below_warn_ &&
                   (timestamp_ms - spo2_below_warn_since_) >=
                       SafetyConfig::SPO2_SUSTAINED_MS) {
            // Sustained warning: SpO2 < 92 % for ≥ 30 s
            add_alert(AlertType::SPO2_LOW, AlertSeverity::WARNING,
                      "SpO2 < 92% for 30s");
        }
    }

    // ---- Fall detection ----------------------------------------------------
    if (fall_det_.is_fall_confirmed()) {
        add_alert(AlertType::FALL_DETECTED, AlertSeverity::CRITICAL,
                  "FALL DETECTED");
    }

    // ---- Heat index --------------------------------------------------------
    if (last_heat_index_c_ >= SafetyConfig::HEAT_CRITICAL_C) {
        add_alert(AlertType::HEAT_DANGER, AlertSeverity::CRITICAL,
                  "EXTREME HEAT DANGER");
    } else if (last_heat_index_c_ >= SafetyConfig::HEAT_WARNING_C) {
        add_alert(AlertType::HEAT_STRESS, AlertSeverity::WARNING,
                  "Heat stress warning");
    }
}

void SafetyEngine::add_alert(AlertType type, AlertSeverity sev,
                             const char* msg)
{
    if (alert_count_ < MAX_ACTIVE_ALERTS) {
        alerts_[alert_count_++] = { type, sev, msg };
    }
}
