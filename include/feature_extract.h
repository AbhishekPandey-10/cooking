#ifndef FEATURE_EXTRACT_H
#define FEATURE_EXTRACT_H

#include <cstdint>
#include <cstddef>
#include <cmath>
#include "safety_types.h"   // RollingVariance

// ============================================================================
//  Feature Vector — 6 biometric features for anomaly detection
//
//  Designed for INT8 autoencoder inference on ESP32-S3.
//  All features are computed from streaming sensor data with O(1) per-sample
//  cost (no large raw-signal buffers).
// ============================================================================

struct FeatureVector {
    static constexpr size_t COUNT = 6;

    float values[COUNT];
    bool  valid[COUNT];

    // Named accessors
    float hr_bpm()          const { return values[0]; }
    float rmssd_ms()        const { return values[1]; }
    float spo2_pct()        const { return values[2]; }
    float skin_temp_c()     const { return values[3]; }
    float skin_temp_slope() const { return values[4]; }  // °C/min
    float accel_var()       const { return values[5]; }  // (m/s²)²

    bool all_valid() const {
        for (size_t i = 0; i < COUNT; i++) { if (!valid[i]) return false; }
        return true;
    }

    size_t valid_count() const {
        size_t n = 0;
        for (size_t i = 0; i < COUNT; i++) { if (valid[i]) n++; }
        return n;
    }
};

// ============================================================================
//  PPG Peak Detector — Streaming, O(1) per sample
//
//  Uses an EMA baseline with hysteresis-based peak detection.
//  Stores last MAX_PEAKS peak timestamps in a ring buffer.
//  Memory footprint: ~120 bytes.
// ============================================================================

namespace PeakConfig {
    constexpr float    EMA_ALPHA       = 0.01f;    // τ ≈ 1.25 s at 80 Hz
    constexpr float    PEAK_OFFSET     = 500.0f;   // ADC counts above EMA
    constexpr float    HYSTERESIS      = 250.0f;    // Exit-peak threshold below peak offset
    constexpr uint32_t REFRACTORY_MS   = 300;       // Min 300 ms between peaks (max 200 bpm)
    constexpr uint32_t PEAK_WINDOW_MS  = 4000;      // Use peaks within last 4 s for HR/RMSSD
    constexpr size_t   MAX_PEAKS       = 16;
}

class PpgPeakDetector {
public:
    /// Feed one IR sample.  Returns true if a peak was just confirmed.
    bool feed(float ir_value, uint32_t timestamp_ms);

    /// Compute heart rate from recent peaks (last PEAK_WINDOW_MS).
    /// Returns 0 if insufficient peaks (< 2).
    float compute_hr_bpm(uint32_t now_ms) const;

    /// Compute HRV RMSSD from recent peaks.
    /// Returns -1.0 if insufficient data (needs ≥ 3 peaks → ≥ 2 IPI → ≥ 1 diff).
    float compute_rmssd_ms(uint32_t now_ms) const;

    /// Number of peaks currently in the window.
    size_t peaks_in_window(uint32_t now_ms) const;

    void reset();

private:
    // EMA baseline
    float ema_         = 0.0f;
    bool  ema_primed_  = false;

    // Peak state machine
    bool     in_peak_      = false;
    float    peak_val_     = 0.0f;
    uint32_t peak_ms_      = 0;
    uint32_t last_peak_ms_ = 0;

    // Circular buffer of confirmed peak timestamps
    uint32_t peak_times_[PeakConfig::MAX_PEAKS] = {};
    size_t   peak_head_  = 0;
    size_t   peak_count_ = 0;

    void store_peak(uint32_t timestamp_ms);

    // Helper: get IPI array from recent peaks.
    // Returns number of IPIs written to `out` (max out_cap).
    size_t get_recent_ipis(uint32_t now_ms, float* out, size_t out_cap) const;
};

// ============================================================================
//  Feature Extractor — Aggregates sensor feeds → FeatureVector
//
//  Call feed_*() as sensor data arrives.  Call extract() at the inference
//  tick rate (1 Hz) to get the current feature vector.
// ============================================================================

class FeatureExtractor {
public:
    /// Feed raw IR amplitude from MAX30102.  Called at PPG sample rate (80 Hz).
    void feed_ppg_ir(float ir_value, uint32_t timestamp_ms);

    /// Feed latest computed SpO2.
    void feed_spo2(int32_t spo2, bool valid);

    /// Feed IMU accelerometer data.  Called at 100 Hz.
    void feed_imu(float ax, float ay, float az);

    /// Feed skin temperature reading.  Called every ~20 s.
    void feed_skin_temp(float temp_c, uint32_t timestamp_ms);

    /// Extract the current feature vector at inference time.
    FeatureVector extract(uint32_t now_ms) const;

    /// Reset all state (e.g., after device re-worn).
    void reset();

private:
    PpgPeakDetector peak_det_;

    // SpO2 cache
    int32_t last_spo2_       = -1;
    bool    spo2_valid_      = false;

    // Skin temp + slope
    float    skin_temp_           = 0.0f;
    bool     skin_temp_received_  = false;
    float    skin_temp_prev_      = 0.0f;
    uint32_t skin_temp_prev_ms_   = 0;
    bool     skin_temp_has_prev_  = false;
    float    skin_temp_slope_     = 0.0f;  // °C/min
    bool     slope_valid_         = false;

    // Accel variance — 200-sample window (2 s at 100 Hz)
    RollingVariance<200> accel_var_;
};

#endif // FEATURE_EXTRACT_H
