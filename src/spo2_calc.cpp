#include "spo2_calc.h"
#include <cmath>

// ============================================================================
//  Spo2Calculator — Implementation
//
//  Streaming SpO2 from Red/IR ratio, computed once per cardiac cycle.
//  See spo2_calc.h for algorithm description.
// ============================================================================

// ---- Feed: one Red + IR sample pair ----------------------------------------
void Spo2Calculator::feed(uint32_t red, uint32_t ir, uint32_t timestamp_ms)
{
    // ---- Contact check: skip if sensor not on skin -------------------------
    if (ir < Spo2Config::IR_CONTACT_MIN) {
        reset();
        return;
    }

    float f_red = static_cast<float>(red);
    float f_ir  = static_cast<float>(ir);

    // ---- EMA baseline (DC component) tracking ------------------------------
    //  The DC component is the slowly-varying average (non-pulsatile) part
    //  of the optical signal.  It reflects tissue absorption, ambient light,
    //  and LED power.  The AC component (pulsatile) rides on top of this.
    if (!ema_primed_) {
        dc_red_     = f_red;
        dc_ir_      = f_ir;
        ema_primed_ = true;
    } else {
        dc_red_ += Spo2Config::EMA_ALPHA * (f_red - dc_red_);
        dc_ir_  += Spo2Config::EMA_ALPHA * (f_ir  - dc_ir_);
    }

    // ---- Track per-cycle min/max (AC extraction) ---------------------------
    //  Within each cardiac cycle, the pulsatile component causes the signal
    //  to rise to a peak (systole) and fall to a trough (diastole).
    //  AC = peak - trough.
    if (!cycle_has_samples_) {
        cycle_max_red_ = f_red;
        cycle_min_red_ = f_red;
        cycle_max_ir_  = f_ir;
        cycle_min_ir_  = f_ir;
        cycle_has_samples_ = true;
    } else {
        if (f_red > cycle_max_red_) cycle_max_red_ = f_red;
        if (f_red < cycle_min_red_) cycle_min_red_ = f_red;
        if (f_ir  > cycle_max_ir_)  cycle_max_ir_  = f_ir;
        if (f_ir  < cycle_min_ir_)  cycle_min_ir_  = f_ir;
    }

    // ---- Cardiac cycle boundary detection (rising zero-crossing on IR) -----
    //  We detect when IR crosses from below DC_ir to above DC_ir.
    //  This corresponds to the start of the systolic upstroke (arterial
    //  pressure wave).  Each crossing marks the end of one cardiac cycle
    //  and the beginning of the next.
    bool now_above = (f_ir > dc_ir_);

    if (!above_dc_ && now_above) {
        // Rising crossing detected
        if (cycle_started_ && cycle_has_samples_) {
            // We have a completed cycle — compute R-ratio
            complete_cycle(timestamp_ms);
        }
        // Start new cycle
        cycle_started_  = true;
        cycle_start_ms_ = timestamp_ms;
        reset_cycle_trackers();
        // Seed with current sample
        cycle_max_red_ = f_red;
        cycle_min_red_ = f_red;
        cycle_max_ir_  = f_ir;
        cycle_min_ir_  = f_ir;
        cycle_has_samples_ = true;
    }

    above_dc_ = now_above;
}

// ---- Complete one cardiac cycle and compute R-ratio -----------------------
void Spo2Calculator::complete_cycle(uint32_t timestamp_ms)
{
    // ---- Validate cycle duration -------------------------------------------
    //  Too fast (>200 bpm) or too slow (<30 bpm) → reject as noise/artifact
    uint32_t cycle_ms = timestamp_ms - cycle_start_ms_;
    if (cycle_ms < Spo2Config::MIN_CYCLE_MS ||
        cycle_ms > Spo2Config::MAX_CYCLE_MS) {
        return;
    }

    // ---- Compute AC components (peak − trough) -----------------------------
    float ac_red = cycle_max_red_ - cycle_min_red_;
    float ac_ir  = cycle_max_ir_  - cycle_min_ir_;

    // ---- Validate: DC must be positive, AC must be real pulsatile ----------
    if (dc_red_ < 1.0f || dc_ir_ < 1.0f) return;
    if ((ac_red / dc_red_) < Spo2Config::MIN_AC_RATIO) return;
    if ((ac_ir  / dc_ir_)  < Spo2Config::MIN_AC_RATIO) return;

    // ---- Compute R-ratio ---------------------------------------------------
    //  R = (AC_red / DC_red) / (AC_ir / DC_ir)
    //
    //  For a healthy individual with SpO2 ~97–99%:
    //    - Oxygenated hemoglobin absorbs less red light → small AC_red
    //    - Oxygenated hemoglobin absorbs more IR light  → larger AC_ir
    //    - R ≈ 0.4–0.6
    //
    //  For desaturated blood (SpO2 ~70–85%):
    //    - Deoxygenated hemoglobin absorbs more red light → larger AC_red
    //    - R ≈ 1.5–2.0
    float r = (ac_red / dc_red_) / (ac_ir / dc_ir_);

    // Sanity check — physiological R range is roughly 0.25 to 2.5
    if (r < 0.2f || r > 3.0f) return;

    // ---- Store in ring buffer ----------------------------------------------
    r_buf_[r_head_] = r;
    r_head_ = (r_head_ + 1) % Spo2Config::R_BUFFER_SIZE;
    if (r_count_ < Spo2Config::R_BUFFER_SIZE) r_count_++;

    // ---- Update SpO2 output ------------------------------------------------
    update_spo2_from_r_buffer();
}

// ---- Map averaged R-ratio to SpO2 percentage -------------------------------
void Spo2Calculator::update_spo2_from_r_buffer()
{
    if (r_count_ < Spo2Config::MIN_VALID_CYCLES) {
        valid_     = false;
        last_spo2_ = -1;
        return;
    }

    // Mean R across the ring buffer
    float sum = 0.0f;
    for (size_t i = 0; i < r_count_; i++) {
        sum += r_buf_[i];
    }
    float mean_r = sum / static_cast<float>(r_count_);

    // ---- Beer-Lambert empirical linear approximation -----------------------
    //  SpO2 ≈ 110 − 25 × R
    //
    //  This is a well-established linear calibration curve for the
    //  660 nm (Red) and 880 nm (IR) LED wavelengths used in the MAX30102.
    //  More accurate calibration requires clinical trial with an arterial
    //  blood gas analyzer (ABG), but this approximation is standard for
    //  uncalibrated reflective pulse oximeters.
    //
    //  Reference points:
    //    R = 0.4  → SpO2 = 100%  (fully oxygenated)
    //    R = 1.0  → SpO2 =  85%  (moderate hypoxia)
    //    R = 2.0  → SpO2 =  60%  (severe hypoxia)
    float spo2_f = 110.0f - 25.0f * mean_r;

    // Clamp to physiological bounds
    if (spo2_f > 100.0f) spo2_f = 100.0f;
    if (spo2_f <   0.0f) spo2_f =   0.0f;

    last_spo2_ = static_cast<int32_t>(spo2_f + 0.5f);  // Round to nearest integer
    valid_     = true;
}

// ---- Reset per-cycle peak/trough trackers ----------------------------------
void Spo2Calculator::reset_cycle_trackers()
{
    cycle_max_red_ = 0.0f;
    cycle_min_red_ = 0.0f;
    cycle_max_ir_  = 0.0f;
    cycle_min_ir_  = 0.0f;
    cycle_has_samples_ = false;
}

// ---- Full reset (contact loss / device re-worn) ----------------------------
void Spo2Calculator::reset()
{
    dc_red_     = 0.0f;
    dc_ir_      = 0.0f;
    ema_primed_ = false;

    above_dc_       = false;
    cycle_started_  = false;
    cycle_start_ms_ = 0;

    reset_cycle_trackers();

    for (size_t i = 0; i < Spo2Config::R_BUFFER_SIZE; i++) r_buf_[i] = 0.0f;
    r_head_  = 0;
    r_count_ = 0;

    last_spo2_ = -1;
    valid_     = false;
}
