#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <cstdint>

// ============================================================================
//  Sensor reading data structures
//  Pure data — no hardware dependencies.  Safe to include in unit tests.
// ============================================================================

struct PpgReading {
    uint32_t ir;
    uint32_t red;
    uint32_t timestamp_ms;
};

struct ImuReading {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // °/s
    uint32_t timestamp_ms;
};

struct TempReading {
    float skin_temp_c;
    uint32_t timestamp_ms;
};

struct EnvReading {
    float bme_temp_c;
    float bme_humidity;   // %RH
    float bme_pressure;   // hPa
    uint16_t mq135_raw;   // 0–4095
    bool   mq135_valid;   // false during warm-up blackout
    uint32_t timestamp_ms;
};

// Aggregate snapshot — one logical "frame" of all sensor data
struct SensorFrame {
    PpgReading  ppg;
    ImuReading  imu;
    TempReading temp;
    EnvReading  env;
};

#endif // SENSOR_TYPES_H
