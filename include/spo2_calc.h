#ifndef SPO2_CALC_H
#define SPO2_CALC_H

#include <cstdint>
#include <cstddef>

// ============================================================================
//  Streaming SpO2 Calculator — Tick-Driven, Zero Dynamic Allocation
//
//  Computes SpO2 from raw MAX30102 Red + IR channels using the standard
//  pulse oximetry R-ratio method:
//
//    R  = (AC_red / DC_red) / (AC_ir / DC_ir)
//    SpO2 ≈ 110 − 25 × R    (Beer-Lambert empirical linear approximation)
//
//  DC component:  EMA baseline (α = 0.01, τ ≈ 1.25 s at 80 Hz)
//  AC component:  Peak-to-trough amplitude within each cardiac cycle
//  Cycle boundary: Rising zero-crossing of IR above its DC baseline
//
//  The calculator self-calibrates from the first few cardiac cycles and
//  requires no training data.  SpO2 is reported once MIN_VALID_CYCLES
//  cycles have been captured.
//
//  Memory footprint: ~92 bytes (all static struct members).
// ============================================================================

namespace Spo2Config {
    /// EMA smoothing coefficient for DC baseline tracking.
    /// τ ≈ 1 / (α × sample_rate) = 1 / (0.01 × 80) ≈ 1.25 seconds.
    constexpr float    EMA_ALPHA         = 0.01f;

    /// Minimum cardiac cycles before SpO2 is reported as valid.
    constexpr uint32_t MIN_VALID_CYCLES  = 4;

    /// AC must exceed this fraction of DC to be considered a real
    /// pulsatile signal (rejects flat-line or noise-only readings).
    constexpr float    MIN_AC_RATIO      = 0.002f;

    /// Minimum raw IR amplitude for skin contact.
    /// Matches PPG_IR_CONTACT_THRESHOLD in safety_types.h.
    constexpr uint32_t IR_CONTACT_MIN    = 5000;

    /// Rolling R-ratio averaging window (number of cardiac cycles).
    constexpr size_t   R_BUFFER_SIZE     = 8;

    /// Maximum duration of one cardiac cycle (minimum 30 bpm).
    constexpr uint32_t MAX_CYCLE_MS      = 2000;

    /// Minimum duration of one cardiac cycle (maximum 200 bpm).
    constexpr uint32_t MIN_CYCLE_MS      = 300;
}

class Spo2Calculator {
public:
    /// Feed one Red + IR sample pair.  Call at PPG sample rate (80 Hz).
    /// @param red          Raw Red channel count from MAX30102
    /// @param ir           Raw IR channel count from MAX30102
    /// @param timestamp_ms Monotonic timestamp
    void feed(uint32_t red, uint32_t ir, uint32_t timestamp_ms);

    /// Latest computed SpO2 percentage (0–100), or −1 if not ready.
    int32_t spo2()  const { return last_spo2_; }

    /// True once enough valid cardiac cycles have been processed.
    bool    valid() const { return valid_; }

    /// Reset all state (e.g., on contact loss or device re-worn).
    void    reset();

private:
    // ---- DC baselines (EMA) ------------------------------------------------
    float dc_red_     = 0.0f;
    float dc_ir_      = 0.0f;
    bool  ema_primed_ = false;

    // ---- Cycle boundary detection (IR rising zero-crossing above DC) -------
    bool     above_dc_       = false;
    bool     cycle_started_  = false;
    uint32_t cycle_start_ms_ = 0;

    // ---- Per-cycle AC tracking (peak and trough within one cardiac cycle) --
    float cycle_max_red_ = 0.0f;
    float cycle_min_red_ = 0.0f;
    float cycle_max_ir_  = 0.0f;
    float cycle_min_ir_  = 0.0f;
    bool  cycle_has_samples_ = false;

    // ---- R-ratio ring buffer for rolling average ---------------------------
    float    r_buf_[Spo2Config::R_BUFFER_SIZE] = {};
    size_t   r_head_  = 0;
    size_t   r_count_ = 0;

    // ---- Output ------------------------------------------------------------
    int32_t  last_spo2_ = -1;
    bool     valid_     = false;

    // ---- Internal helpers --------------------------------------------------
    void complete_cycle(uint32_t timestamp_ms);
    void reset_cycle_trackers();
    void update_spo2_from_r_buffer();
};

#endif // SPO2_CALC_H
