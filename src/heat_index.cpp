#include "heat_index.h"
#include <cmath>

// ============================================================================
//  Internal helpers
// ============================================================================
static inline float c_to_f(float c) { return c * 1.8f + 32.0f; }
static inline float f_to_c(float f) { return (f - 32.0f) / 1.8f; }

// ============================================================================
//  NWS Heat Index — Full Implementation
//
//  Algorithm (following the NWS procedure exactly):
//    1. Compute Steadman simple HI.
//    2. Average with T.  If average < 80 °F → return simple HI (linear regime).
//    3. Otherwise compute the full 9-term Rothfusz regression.
//    4. Apply low-humidity adjustment  if RH < 13 % and 80 ≤ T ≤ 112 °F.
//    5. Apply high-humidity adjustment if RH > 85 % and 80 ≤ T ≤ 87 °F.
// ============================================================================

float heat_index_celsius(float temp_c, float rh_pct)
{
    float T = c_to_f(temp_c);
    float R = rh_pct;

    // Clamp RH to physical bounds
    if (R < 0.0f)   R = 0.0f;
    if (R > 100.0f) R = 100.0f;

    // ---- Step 1: Steadman simple formula -----------------------------------
    float hi_simple = 0.5f * (T + 61.0f + ((T - 68.0f) * 1.2f) + (R * 0.094f));

    // ---- Step 2: Check whether the full regression is needed ---------------
    float hi_avg = (hi_simple + T) * 0.5f;
    if (hi_avg < 80.0f) {
        return f_to_c(hi_simple);
    }

    // ---- Step 3: Full 9-term Rothfusz regression ---------------------------
    //
    //  HI = c1 + c2·T + c3·R + c4·T·R + c5·T² + c6·R²
    //       + c7·T²·R + c8·T·R² + c9·T²·R²
    //
    float T2 = T * T;
    float R2 = R * R;

    float hi =  -42.379f
             +   2.04901523f  * T
             +  10.14333127f  * R
             -   0.22475541f  * T * R
             -   0.00683783f  * T2
             -   0.05481717f  * R2
             +   0.00122874f  * T2 * R
             +   0.00085282f  * T  * R2
             -   0.00000199f  * T2 * R2;

    // ---- Step 4: Low-humidity adjustment -----------------------------------
    //  Subtract when RH < 13 % and temperature is in the 80–112 °F band.
    if (R < 13.0f && T >= 80.0f && T <= 112.0f) {
        float adj = ((13.0f - R) / 4.0f)
                  * sqrtf((17.0f - fabsf(T - 95.0f)) / 17.0f);
        hi -= adj;
    }

    // ---- Step 5: High-humidity adjustment ----------------------------------
    //  Add when RH > 85 % and temperature is in the 80–87 °F band.
    if (R > 85.0f && T >= 80.0f && T <= 87.0f) {
        float adj = ((R - 85.0f) / 10.0f) * ((87.0f - T) / 5.0f);
        hi += adj;
    }

    return f_to_c(hi);
}
