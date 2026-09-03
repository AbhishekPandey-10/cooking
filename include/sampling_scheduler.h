#ifndef SAMPLING_SCHEDULER_H
#define SAMPLING_SCHEDULER_H

// ============================================================================
//  Sampling Scheduler — Hardware-Timer Driven
//
//  High-rate sensors (MAX30102 @ 80 Hz, MPU6050 @ 100 Hz) are triggered by
//  ESP32-S3 hardware timer ISRs that set atomic flags.
//
//  Low-rate sensors (MAX30205 @ 0.05 Hz, BME280+MQ135 @ 0.02 Hz) use a
//  software millis()-based check in the scheduler's poll function.
//
//  The scheduler never performs I2C directly inside an ISR — it only sets
//  flags that the main-loop poll function checks.
// ============================================================================

/// Attach hardware timers and start all scheduling.
void scheduler_init();

/// Call from loop().  Checks flags, performs reads, returns true
/// if any new data was produced during this call.
bool scheduler_poll();

/// Get a const reference to the latest aggregated sensor frame.
/// Safe to read from the main loop (single-producer / single-consumer).
const struct SensorFrame &scheduler_latest_frame();

#endif // SAMPLING_SCHEDULER_H
