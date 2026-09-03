#include "correlator.h"
#include <cstdio>

// ============================================================================
//  Correlator — Dual-Path Decision Engine Implementation
//
//  Edge-case notes:
//
//  1. Window wrap:  The circular buffer with running counters is always
//     consistent.  When head_ wraps past WINDOW_SIZE, the oldest entry is
//     evicted and its flags subtracted from the counters *before* the new
//     entry is added.  The coincidence check always reflects the true
//     current window state regardless of wrap position.
//
//  2. MQ135 first reading:  mq135_has_prev_ starts false, so the gas flag
//     cannot trigger until the second valid reading.  This is correct — you
//     need two data points to compute a delta.
//
//  3. MQ135 continuity across reset:  mq135_prev_ is preserved when the
//     window resets after a Path B alert.  Delta tracking is therefore
//     seamless; there is no gap in gas monitoring.
//
//  4. Path A preempts Path B:  When Path A fires, tick() returns immediately
//     without pushing a new entry into the window.  This means Path B does
//     not accumulate data during an active hard bypass.  Rationale: during a
//     confirmed fall or SpO2 < 90 % event the patient is already in the
//     highest alert tier; sub-threshold corroboration is irrelevant.
//
//  5. Post-alert cooldown:  After a Path B alert, reset() zeroes the window
//     and all counters.  A new alert requires a fresh 45 s of accumulation,
//     preventing alert flooding from a single sustained event.
//
//  6. Partial window:  Before the buffer fills (first 45 s after boot or
//     reset), the coincidence check runs against whatever entries are
//     present.  Two flags in the first 3 seconds will still fire — this is
//     intentional, since an early multi-axis anomaly during a disaster is
//     precisely when a fast alert matters most.
// ============================================================================

CorrelatorResult Correlator::tick(
    int32_t  spo2,
    bool     spo2_valid,
    bool     fall_confirmed,
    float    heat_index_c,
    uint16_t mq135_raw,
    bool     mq135_valid,
    float    skin_temp_c,
    uint32_t /* timestamp_ms — reserved */)
{
    // ==== Path A: Hard Bypass ===============================================
    //  Zero-delay, highest priority.  Checked before any Path B work.

    if (fall_confirmed) {
        return CorrelatorResult::PATH_A_HARD_BYPASS;
    }

    if (spo2_valid && spo2 < CorrelatorConfig::SPO2_HARD_BYPASS) {
        return CorrelatorResult::PATH_A_HARD_BYPASS;
    }

    // ==== Path B: 45 s Corroborator =========================================

    CorrelatorFlags flags = { false, false, false, false };

    // ---- Optical flag: SpO2 in the 90–92 % borderline band -----------------
    if (spo2_valid && spo2 >= CorrelatorConfig::SPO2_OPT_LOW
                   && spo2 <= CorrelatorConfig::SPO2_OPT_HIGH) {
        flags.optical = true;
    }

    // ---- Heat flag: Rothfusz heat index exceeds threshold ------------------
    if (heat_index_c > CorrelatorConfig::HEAT_INDEX_THRESHOLD_C) {
        flags.heat = true;
    }

    // ---- Gas flag: MQ135 |Δ| from previous valid reading -------------------
    if (mq135_valid && mq135_has_prev_) {
        int32_t delta = static_cast<int32_t>(mq135_raw)
                      - static_cast<int32_t>(mq135_prev_);
        if (delta < 0) delta = -delta;

        if (delta > static_cast<int32_t>(CorrelatorConfig::MQ135_DELTA_THRESHOLD)) {
            flags.gas = true;
        }
    }
    // Update baseline (even if delta didn't trigger — we always track)
    if (mq135_valid) {
        mq135_prev_     = mq135_raw;
        mq135_has_prev_ = true;
    }

    // ---- Thermal flag: skin temp outside safe band -------------------------
    if (skin_temp_c < CorrelatorConfig::SKIN_TEMP_LOW_C ||
        skin_temp_c > CorrelatorConfig::SKIN_TEMP_HIGH_C) {
        flags.thermal = true;
    }

    // ---- Push into sliding window ------------------------------------------
    push_flags(flags);

    // ---- Coincidence check -------------------------------------------------
    uint8_t active = active_flag_count();
    if (active >= CorrelatorConfig::COINCIDENCE_THRESHOLD) {
        build_trigger_description();
        reset();
        return CorrelatorResult::PATH_B_CORRELATED;
    }

    return CorrelatorResult::NONE;
}

// ============================================================================
//  Circular Buffer Push — O(1) with running counter maintenance
// ============================================================================
void Correlator::push_flags(const CorrelatorFlags &flags)
{
    // If the buffer is full, evict the oldest entry first
    if (count_ >= CorrelatorConfig::WINDOW_SIZE) {
        const CorrelatorFlags &old = window_[head_];
        if (old.optical)  optical_count_--;
        if (old.heat)     heat_count_--;
        if (old.gas)      gas_count_--;
        if (old.thermal)  thermal_count_--;
    } else {
        count_++;
    }

    // Write new entry at head
    window_[head_] = flags;
    if (flags.optical)  optical_count_++;
    if (flags.heat)     heat_count_++;
    if (flags.gas)      gas_count_++;
    if (flags.thermal)  thermal_count_++;

    head_ = (head_ + 1) % CorrelatorConfig::WINDOW_SIZE;
}

// ============================================================================
//  Active Flag Count
// ============================================================================
uint8_t Correlator::active_flag_count() const
{
    uint8_t n = 0;
    if (optical_count_ > 0) n++;
    if (heat_count_    > 0) n++;
    if (gas_count_     > 0) n++;
    if (thermal_count_ > 0) n++;
    return n;
}

// ============================================================================
//  Debug: build human-readable trigger description
//
//  Called just before reset(), so the counters still reflect the window
//  that caused the alert.  Output example: "OPT(3) THRM(8)"
// ============================================================================
void Correlator::build_trigger_description()
{
    int pos = 0;
    const int cap = static_cast<int>(sizeof(trigger_desc_));

    if (optical_count_ > 0) {
        pos += snprintf(trigger_desc_ + pos, cap - pos,
                        "OPT(%u) ", optical_count_);
    }
    if (heat_count_ > 0) {
        pos += snprintf(trigger_desc_ + pos, cap - pos,
                        "HEAT(%u) ", heat_count_);
    }
    if (gas_count_ > 0) {
        pos += snprintf(trigger_desc_ + pos, cap - pos,
                        "GAS(%u) ", gas_count_);
    }
    if (thermal_count_ > 0) {
        pos += snprintf(trigger_desc_ + pos, cap - pos,
                        "THRM(%u) ", thermal_count_);
    }

    // Trim trailing space
    if (pos > 0 && trigger_desc_[pos - 1] == ' ') {
        trigger_desc_[pos - 1] = '\0';
    }
}

const char* Correlator::last_trigger_description() const
{
    return trigger_desc_;
}

// ============================================================================
//  Reset — clears window and counters, preserves MQ135 baseline + last desc
// ============================================================================
void Correlator::reset()
{
    head_  = 0;
    count_ = 0;

    optical_count_ = 0;
    heat_count_    = 0;
    gas_count_     = 0;
    thermal_count_ = 0;

    for (size_t i = 0; i < CorrelatorConfig::WINDOW_SIZE; i++) {
        window_[i] = { false, false, false, false };
    }

    // mq135_prev_ / mq135_has_prev_  intentionally NOT reset —
    //   delta tracking is continuous across alert cycles.
    // trigger_desc_  intentionally NOT reset —
    //   last_trigger_description() should remain readable for logging.
}
