#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "sensor_types.h"

// ============================================================================
//  Sensor Manager — Initialisation + Individual Read Functions
//
//  Design: each read function is a pure "do one thing" call that can be
//  invoked independently by the scheduler *or* by a unit-test harness.
//  None of them know about timers, queues, or scheduling.
// ============================================================================

/// Initialise the I2C bus and probe every device.
/// Returns a bitmask of successfully initialised devices (bit positions below).
///   bit 0 — MAX30102
///   bit 1 — MPU6050
///   bit 2 — MAX30205
///   bit 3 — BME280
///   bit 4 — SSD1306
constexpr uint8_t SENSOR_MAX30102 = (1 << 0);
constexpr uint8_t SENSOR_MPU6050  = (1 << 1);
constexpr uint8_t SENSOR_MAX30205 = (1 << 2);
constexpr uint8_t SENSOR_BME280   = (1 << 3);
constexpr uint8_t SENSOR_SSD1306  = (1 << 4);
constexpr uint8_t SENSOR_ALL      = 0x1F;

uint8_t sensors_init();

// ---- Individual read functions (decoupled from scheduling) -----------------

/// Read one sample from the MAX30102 PPG sensor.
/// Returns true on success, fills `out`.
bool read_ppg(PpgReading &out);

/// Read one sample from the MPU6050 IMU.
/// Returns true on success, fills `out`.
bool read_imu(ImuReading &out);

/// Read skin temperature from MAX30205.
/// Returns true on success, fills `out`.
bool read_skin_temp(TempReading &out);

/// Read BME280 climate data + MQ135 analog gas reading.
/// `out.mq135_valid` is false if the warm-up blackout hasn't elapsed.
/// Returns true on success, fills `out`.
bool read_environment(EnvReading &out);

/// Update the OLED with the latest sensor frame.
void display_update(const SensorFrame &frame);

#endif // SENSOR_MANAGER_H
