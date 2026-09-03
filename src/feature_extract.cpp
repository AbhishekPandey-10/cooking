#include "feature_extract.h"
#include <cmath>

// ============================================================================
//  PPG Peak Detector — Implementation
// ============================================================================

bool PpgPeakDetector::feed(float ir_value, uint32_t timestamp_ms)
{
    // ---- EMA baseline tracking ---------------------------------------------
    if (!ema_primed_) {
        ema_        = ir_value;
        ema_primed_ = true;
    } else {
        ema_ += PeakConfig::EMA_ALPHA * (ir_value - ema_);
    }

    float threshold = ema_ + PeakConfig::PEAK_OFFSET;
    bool peak_confirmed = false;

    if (!in_peak_) {
        // ---- Looking for a peak entry --------------------------------------
        if (ir_value > threshold) {
            in_peak_  = true;
            peak_val_ = ir_value;
            peak_ms_  = timestamp_ms;
        }
    } else {
        // ---- Inside a peak region — track the maximum ----------------------
        if (ir_value > peak_val_) {
            peak_val_ = ir_value;
            peak_ms_  = timestamp_ms;
        }

        // ---- Check for exit (hysteresis) -----------------------------------
        float exit_threshold = ema_ + PeakConfig::PEAK_OFFSET - PeakConfig::HYSTERESIS;
        if (ir_value < exit_threshold) {
            // Confirm peak if refractory period has elapsed
            if (last_peak_ms_ == 0 ||
                (peak_ms_ - last_peak_ms_) >= PeakConfig::REFRACTORY_MS) {
                store_peak(peak_ms_);
                last_peak_ms_  = peak_ms_;
                peak_confirmed = true;
            }
            in_peak_ = false;
        }
    }

    return peak_confirmed;
}

void PpgPeakDetector::store_peak(uint32_t timestamp_ms)
{
    peak_times_[peak_head_] = timestamp_ms;
    peak_head_ = (peak_head_ + 1) % PeakConfig::MAX_PEAKS;
    if (peak_count_ < PeakConfig::MAX_PEAKS) peak_count_++;
}

size_t PpgPeakDetector::peaks_in_window(uint32_t now_ms) const
{
    size_t n = 0;
    for (size_t i = 0; i < peak_count_; i++) {
        size_t idx = (peak_head_ + PeakConfig::MAX_PEAKS - 1 - i)
                     % PeakConfig::MAX_PEAKS;
        if ((now_ms - peak_times_[idx]) <= PeakConfig::PEAK_WINDOW_MS) {
            n++;
        } else {
            break;  // Older peaks are further back — no need to check
        }
    }
    return n;
}

size_t PpgPeakDetector::get_recent_ipis(uint32_t now_ms, float* out,
                                         size_t out_cap) const
{
    // Collect recent peak timestamps (most recent first)
    uint32_t recent[PeakConfig::MAX_PEAKS];
    size_t n_recent = 0;

    for (size_t i = 0; i < peak_count_ && n_recent < PeakConfig::MAX_PEAKS; i++) {
        size_t idx = (peak_head_ + PeakConfig::MAX_PEAKS - 1 - i)
                     % PeakConfig::MAX_PEAKS;
        if ((now_ms - peak_times_[idx]) <= PeakConfig::PEAK_WINDOW_MS) {
            recent[n_recent++] = peak_times_[idx];
        } else {
            break;
        }
    }

    // Need at least 2 peaks for 1 IPI
    if (n_recent < 2) return 0;

    // recent[] is most-recent-first; IPIs are computed between consecutive peaks
    size_t n_ipi = 0;
    for (size_t i = 0; i < n_recent - 1 && n_ipi < out_cap; i++) {
        // recent[i] > recent[i+1] since ordered most-recent-first
        out[n_ipi++] = static_cast<float>(recent[i] - recent[i + 1]);
    }
    return n_ipi;
}

float PpgPeakDetector::compute_hr_bpm(uint32_t now_ms) const
{
    float ipis[PeakConfig::MAX_PEAKS];
    size_t n = get_recent_ipis(now_ms, ipis, PeakConfig::MAX_PEAKS);
    if (n == 0) return 0.0f;

    // Mean IPI
    float sum = 0.0f;
    for (size_t i = 0; i < n; i++) sum += ipis[i];
    float mean_ipi_ms = sum / static_cast<float>(n);

    if (mean_ipi_ms <= 0.0f) return 0.0f;
    return 60000.0f / mean_ipi_ms;
}

float PpgPeakDetector::compute_rmssd_ms(uint32_t now_ms) const
{
    float ipis[PeakConfig::MAX_PEAKS];
    size_t n = get_recent_ipis(now_ms, ipis, PeakConfig::MAX_PEAKS);

    // Need ≥ 2 IPIs for 1 successive difference
    if (n < 2) return -1.0f;

    // RMSSD = sqrt( (1/(K-1)) × Σ (IPI_{i} − IPI_{i+1})² )
    // ipis are in order: most-recent-IPI first
    float sum_sq = 0.0f;
    size_t k = 0;
    for (size_t i = 0; i < n - 1; i++) {
        float diff = ipis[i] - ipis[i + 1];
        sum_sq += diff * diff;
        k++;
    }

    if (k == 0) return -1.0f;
    return sqrtf(sum_sq / static_cast<float>(k));
}

void PpgPeakDetector::reset()
{
    ema_         = 0.0f;
    ema_primed_  = false;
    in_peak_     = false;
    peak_val_    = 0.0f;
    peak_ms_     = 0;
    last_peak_ms_ = 0;
    peak_head_   = 0;
    peak_count_  = 0;
    for (size_t i = 0; i < PeakConfig::MAX_PEAKS; i++) peak_times_[i] = 0;
}

// ============================================================================
//  Feature Extractor — Implementation
// ============================================================================

void FeatureExtractor::feed_ppg_ir(float ir_value, uint32_t timestamp_ms)
{
    peak_det_.feed(ir_value, timestamp_ms);
}

void FeatureExtractor::feed_spo2(int32_t spo2, bool valid)
{
    last_spo2_  = spo2;
    spo2_valid_ = valid;
}

void FeatureExtractor::feed_imu(float ax, float ay, float az)
{
    float amag = sqrtf(ax * ax + ay * ay + az * az);
    accel_var_.push(amag);
}

void FeatureExtractor::feed_skin_temp(float temp_c, uint32_t timestamp_ms)
{
    if (skin_temp_received_ && skin_temp_has_prev_) {
        // Compute slope in °C/min
        float dt_min = static_cast<float>(timestamp_ms - skin_temp_prev_ms_)
                     / 60000.0f;
        if (dt_min > 0.001f) {
            skin_temp_slope_ = (temp_c - skin_temp_prev_) / dt_min;
            slope_valid_     = true;
        }
    }

    skin_temp_prev_    = skin_temp_received_ ? skin_temp_ : temp_c;
    skin_temp_prev_ms_ = skin_temp_received_
                       ? (skin_temp_prev_ms_ > 0 ? skin_temp_prev_ms_ : timestamp_ms)
                       : timestamp_ms;

    // If this is the first reading, set prev from this reading for next time
    if (skin_temp_received_) {
        skin_temp_prev_    = skin_temp_;
        skin_temp_prev_ms_ = timestamp_ms;  // approximate — actual prev timestamp
    } else {
        skin_temp_prev_ms_ = timestamp_ms;
    }

    skin_temp_          = temp_c;
    skin_temp_received_ = true;
    skin_temp_has_prev_ = true;
}

FeatureVector FeatureExtractor::extract(uint32_t now_ms) const
{
    FeatureVector fv;
    for (size_t i = 0; i < FeatureVector::COUNT; i++) {
        fv.values[i] = 0.0f;
        fv.valid[i]  = false;
    }

    // Feature 0: Heart Rate (bpm)
    float hr = peak_det_.compute_hr_bpm(now_ms);
    if (hr > 20.0f && hr < 250.0f) {   // physiological sanity bounds
        fv.values[0] = hr;
        fv.valid[0]  = true;
    }

    // Feature 1: RMSSD (ms)
    float rmssd = peak_det_.compute_rmssd_ms(now_ms);
    if (rmssd >= 0.0f && rmssd < 500.0f) {  // sanity bound
        fv.values[1] = rmssd;
        fv.valid[1]  = true;
    }

    // Feature 2: SpO2 (%)
    if (spo2_valid_ && last_spo2_ > 0 && last_spo2_ <= 100) {
        fv.values[2] = static_cast<float>(last_spo2_);
        fv.valid[2]  = true;
    }

    // Feature 3: Skin Temperature (°C)
    if (skin_temp_received_ && skin_temp_ > 15.0f && skin_temp_ < 45.0f) {
        fv.values[3] = skin_temp_;
        fv.valid[3]  = true;
    }

    // Feature 4: Skin Temp Slope (°C/min)
    if (slope_valid_ && !std::isnan(skin_temp_slope_)
                     && skin_temp_slope_ > -5.0f && skin_temp_slope_ < 5.0f) {
        fv.values[4] = skin_temp_slope_;
        fv.valid[4]  = true;
    }

    // Feature 5: Accel Variance ((m/s²)²)
    if (accel_var_.count() >= 50) {   // need at least 0.5 s of data
        float var = accel_var_.variance();
        if (!std::isnan(var)) {
            fv.values[5] = var;
            fv.valid[5]  = true;
        }
    }

    return fv;
}

void FeatureExtractor::reset()
{
    peak_det_.reset();
    last_spo2_           = -1;
    spo2_valid_          = false;
    skin_temp_           = 0.0f;
    skin_temp_received_  = false;
    skin_temp_prev_      = 0.0f;
    skin_temp_prev_ms_   = 0;
    skin_temp_has_prev_  = false;
    skin_temp_slope_     = 0.0f;
    slope_valid_         = false;
    accel_var_.reset();
}
