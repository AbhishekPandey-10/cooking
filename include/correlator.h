#ifndef CORRELATOR_H
#define CORRELATOR_H

#include <cstdint>
#include <cstddef>

// ============================================================================
//  Cross-Sensor Anomaly Correlator — Dual-Path Decision Engine
//
//  Path A (Hard Bypass):
//    Kinematic fall confirmed OR SpO2 < 90 %
//    → immediate high-priority alert, zero delay, skip Path B
//
//  Path B (45 s Corroborator):
//    Maintains a fixed-size sliding window (circular buffer, 1 Hz tick rate)
//    tracking 4 boolean flags per tick:
//      • Optical  — SpO2 in the 90–92 % borderline band
//      • Heat     — Rothfusz heat index > 38 °C
//      • Gas      — MQ135 reading delta exceeds threshold
//      • Thermal  — Skin temp < 30 °C  or  > 38.5 °C
//
//    If ≥ COINCIDENCE_THRESHOLD distinct flags have been TRUE at any point
//    within the window, fire a WARNING-severity alert and reset.
//
//  The window duration, coincidence threshold, and every per-flag threshold
//  are configurable compile-time constants for bench-test tuning.
// ============================================================================

// ============================================================================
//  Tunable Constants
// ============================================================================
namespace CorrelatorConfig {

    // ---- Window sizing (1 Hz tick assumed) ----------------------------------
    constexpr uint32_t WINDOW_DURATION_S         = 45;
    constexpr uint32_t TICK_RATE_HZ              = 1;
    constexpr size_t   WINDOW_SIZE               = WINDOW_DURATION_S * TICK_RATE_HZ;

    // ---- Coincidence threshold  ("N-of-4") ---------------------------------
    constexpr uint8_t  COINCIDENCE_THRESHOLD     = 2;
    constexpr uint8_t  FLAG_COUNT                = 4;

    // ---- Path A: hard bypass -----------------------------------------------
    constexpr int32_t  SPO2_HARD_BYPASS          = 90;  // SpO2 < this → Path A

    // ---- Path B flag thresholds --------------------------------------------

    //  Optical: SpO2 borderline band (inclusive both ends)
    constexpr int32_t  SPO2_OPT_LOW              = 90;
    constexpr int32_t  SPO2_OPT_HIGH             = 92;

    //  Heat: Rothfusz heat index above this (°C)
    constexpr float    HEAT_INDEX_THRESHOLD_C     = 38.0f;

    //  Gas: MQ135 absolute delta between consecutive valid readings
    //
    //  Rationale for 150 ADC counts (12-bit, 0–4095 range):
    //    • Normal ambient noise between consecutive 50 s readings: ±20–30 cts
    //      (air currents, breathing, HVAC cycling).
    //    • Moderate air-quality shifts (cooking, AC mode change): ±30–80 cts.
    //    • Significant event (smoke ingress, gas leak, structural collapse
    //      releasing particulates): ±100–300+ cts.
    //    • 150 is ~5–7× above noise, virtually eliminates false positives in
    //      calm indoor air, yet catches real events well before they reach
    //      immediately-dangerous concentrations.
    //    • Conservative for true disaster scenarios where ΔV > 300 cts is
    //      typical — errs on the side of early detection.
    //    • The threshold is absolute (not proportional to baseline) because
    //      the MQ135 output is log-proportional to concentration; a fixed
    //      ADC-delta therefore already represents a larger multiplicative
    //      concentration change at low baselines than at high ones.
    //    • Fully configurable — tune during bench testing with known gas
    //      source at measured distance.
    constexpr uint16_t MQ135_DELTA_THRESHOLD     = 150;

    //  Thermal: skin temperature out-of-range bounds (°C)
    constexpr float    SKIN_TEMP_LOW_C            = 30.0f;
    constexpr float    SKIN_TEMP_HIGH_C           = 38.5f;
}

// ============================================================================
//  Per-tick flag snapshot
// ============================================================================
struct CorrelatorFlags {
    bool optical;   // SpO2 90–92 %
    bool heat;      // Heat index > 38 °C
    bool gas;       // MQ135 |Δ| > threshold
    bool thermal;   // Skin temp out of range
};

// ============================================================================
//  Decision result
// ============================================================================
enum class CorrelatorResult : uint8_t {
    NONE,                  // No alert
    PATH_A_HARD_BYPASS,    // Immediate: fall or SpO2 < 90 %
    PATH_B_CORRELATED      // ≥ 2-of-4 flags in 45 s window
};

// ============================================================================
//  Correlator Class
// ============================================================================
class Correlator {
public:
    /// Main entry point — call once per tick at TICK_RATE_HZ.
    ///
    /// @param spo2            Computed SpO2 (0–100), or −1 if unavailable
    /// @param spo2_valid      True if the value is trustworthy
    /// @param fall_confirmed  True if FallDetector is in FALL_CONFIRMED state
    /// @param heat_index_c    Latest Rothfusz heat index in °C
    /// @param mq135_raw       Latest MQ135 ADC reading (0–4095)
    /// @param mq135_valid     True if MQ135 is past its warm-up blackout
    /// @param skin_temp_c     Latest skin temperature in °C
    /// @param timestamp_ms    Monotonic timestamp (reserved for future use)
    /// @return                Decision result for this tick
    CorrelatorResult tick(int32_t  spo2,
                          bool     spo2_valid,
                          bool     fall_confirmed,
                          float    heat_index_c,
                          uint16_t mq135_raw,
                          bool     mq135_valid,
                          float    skin_temp_c,
                          uint32_t timestamp_ms);

    // ---- Window introspection (for debugging / display) --------------------

    /// How many distinct flag categories have ≥ 1 TRUE in the current window.
    uint8_t active_flag_count() const;

    bool optical_active() const { return optical_count_ > 0; }
    bool heat_active()    const { return heat_count_    > 0; }
    bool gas_active()     const { return gas_count_     > 0; }
    bool thermal_active() const { return thermal_count_ > 0; }

    /// Per-flag hit counts within the current window.
    uint16_t optical_hits() const { return optical_count_; }
    uint16_t heat_hits()    const { return heat_count_; }
    uint16_t gas_hits()     const { return gas_count_; }
    uint16_t thermal_hits() const { return thermal_count_; }

    size_t window_fill() const { return count_; }

    /// Human-readable description of which flags triggered the last Path B
    /// alert.  Format: "OPT(3) HEAT(12)"  — name followed by hit count.
    /// Pointer is to an internal buffer; valid until the next trigger.
    const char* last_trigger_description() const;

    /// Clear the window and running counters.  Preserves MQ135 baseline
    /// (delta tracking is continuous) and last trigger description.
    void reset();

private:
    // ---- Circular buffer ---------------------------------------------------
    CorrelatorFlags window_[CorrelatorConfig::WINDOW_SIZE] = {};
    size_t head_  = 0;
    size_t count_ = 0;

    // ---- Running per-flag counters  (O(1) coincidence check) ---------------
    //  Each counter = number of entries currently in the window where that
    //  flag is true.  Counter > 0 ⟹ flag "has been true at some point."
    uint16_t optical_count_ = 0;
    uint16_t heat_count_    = 0;
    uint16_t gas_count_     = 0;
    uint16_t thermal_count_ = 0;

    // ---- MQ135 delta state (persists across window resets) ------------------
    uint16_t mq135_prev_     = 0;
    bool     mq135_has_prev_ = false;

    // ---- Debug -------------------------------------------------------------
    char trigger_desc_[80] = {};

    // ---- Internal helpers --------------------------------------------------
    void push_flags(const CorrelatorFlags &flags);
    void build_trigger_description();
};

#endif // CORRELATOR_H
