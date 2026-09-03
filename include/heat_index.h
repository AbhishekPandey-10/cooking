#ifndef HEAT_INDEX_H
#define HEAT_INDEX_H

// ============================================================================
//  NWS Heat Index — Pure Math Module
//
//  Implements the full 9-term Rothfusz regression polynomial used by the
//  U.S. National Weather Service, with Steadman linear fallback and the two
//  NWS humidity-range adjustments.
//
//  Input/output in Celsius; the Fahrenheit conversion is internal.
//
//  Rothfusz coefficients (T in °F, R in %RH):
//    c1 = −42.379
//    c2 = +2.04901523        (T)
//    c3 = +10.14333127       (R)
//    c4 = −0.22475541        (T·R)
//    c5 = −0.00683783        (T²)
//    c6 = −0.05481717        (R²)
//    c7 = +0.00122874        (T²·R)
//    c8 = +0.00085282        (T·R²)
//    c9 = −0.00000199        (T²·R²)
//
//  Source: NWS Technical Attachment SR 90-23 (Rothfusz, 1990)
// ============================================================================

/// Compute the NWS Heat Index in °C.
///
/// @param temp_c   Air temperature in °C
/// @param rh_pct   Relative humidity in percent (0–100)
/// @return         Heat Index in °C
float heat_index_celsius(float temp_c, float rh_pct);

#endif // HEAT_INDEX_H
