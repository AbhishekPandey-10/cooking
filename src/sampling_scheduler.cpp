#include "sampling_scheduler.h"
#include "sensor_manager.h"
#include "sensor_types.h"
#include "config.h"

#include <Arduino.h>

// ============================================================================
//  ISR Flags  (volatile — set in ISR, cleared in main-loop poll)
// ============================================================================
static volatile bool s_flag_ppg = false;
static volatile bool s_flag_imu = false;

// ============================================================================
//  Hardware Timer Handles
// ============================================================================
static hw_timer_t *s_timer_ppg = nullptr;   // Timer 0 → MAX30102
static hw_timer_t *s_timer_imu = nullptr;   // Timer 1 → MPU6050

// ============================================================================
//  Software timer bookkeeping (low-rate sensors)
// ============================================================================
static uint32_t s_last_temp_ms = 0;
static uint32_t s_last_env_ms  = 0;

// ============================================================================
//  Latest aggregated frame
// ============================================================================
static SensorFrame s_frame = {};

// ============================================================================
//  ISRs — minimal work: just set a flag
// ============================================================================
static void IRAM_ATTR isr_ppg()
{
    s_flag_ppg = true;
}

static void IRAM_ATTR isr_imu()
{
    s_flag_imu = true;
}

// ============================================================================
//  Scheduler Init
// ============================================================================
void scheduler_init()
{
    // ---- Hardware timer 0: MAX30102 at 80 Hz (12 500 µs) -------------------
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    s_timer_ppg = timerBegin(1000000);                   // 1 MHz tick
    if (s_timer_ppg) {
        timerAttachInterrupt(s_timer_ppg, &isr_ppg);
        timerAlarm(s_timer_ppg, MAX30102_SAMPLE_US, true, 0);  // auto-reload
        Serial.printf("[SCHED] PPG  timer: %d µs period\n", MAX30102_SAMPLE_US);
    }

    // ---- Hardware timer 1: MPU6050 at 100 Hz (10 000 µs) -------------------
    s_timer_imu = timerBegin(1000000);
    if (s_timer_imu) {
        timerAttachInterrupt(s_timer_imu, &isr_imu);
        timerAlarm(s_timer_imu, MPU6050_SAMPLE_US, true, 0);
        Serial.printf("[SCHED] IMU  timer: %d µs period\n", MPU6050_SAMPLE_US);
    }
#else
    // Core 2.x: APB clock 80 MHz, divider 80 -> 1 MHz (1 tick = 1 µs)
    s_timer_ppg = timerBegin(0, 80, true);
    if (s_timer_ppg) {
        timerAttachInterrupt(s_timer_ppg, &isr_ppg, true);
        timerAlarmWrite(s_timer_ppg, MAX30102_SAMPLE_US, true);
        timerAlarmEnable(s_timer_ppg);
        Serial.printf("[SCHED] PPG  timer: %d µs period\n", MAX30102_SAMPLE_US);
    }

    // ---- Hardware timer 1: MPU6050 at 100 Hz (10 000 µs) -------------------
    s_timer_imu = timerBegin(1, 80, true);
    if (s_timer_imu) {
        timerAttachInterrupt(s_timer_imu, &isr_imu, true);
        timerAlarmWrite(s_timer_imu, MPU6050_SAMPLE_US, true);
        timerAlarmEnable(s_timer_imu);
        Serial.printf("[SCHED] IMU  timer: %d µs period\n", MPU6050_SAMPLE_US);
    }
#endif

    // Seed software timer baselines so first reads happen after the interval
    s_last_temp_ms = millis();
    s_last_env_ms  = millis();

    Serial.println("[SCHED] Scheduler started");
}

// ============================================================================
//  Scheduler Poll — call from loop()
// ============================================================================
bool scheduler_poll()
{
    bool new_data = false;
    uint32_t now = millis();

    // ---- High-rate: MAX30102 PPG -------------------------------------------
    if (s_flag_ppg) {
        s_flag_ppg = false;
        if (read_ppg(s_frame.ppg)) {
            new_data = true;
        }
    }

    // ---- High-rate: MPU6050 IMU --------------------------------------------
    if (s_flag_imu) {
        s_flag_imu = false;
        if (read_imu(s_frame.imu)) {
            new_data = true;
        }
    }

    // ---- Low-rate: MAX30205 skin temp (every 20 s) -------------------------
    if ((now - s_last_temp_ms) >= MAX30205_SAMPLE_MS) {
        s_last_temp_ms = now;
        if (read_skin_temp(s_frame.temp)) {
            new_data = true;
        }
    }

    // ---- Low-rate: BME280 + MQ135 environment (every 50 s) -----------------
    if ((now - s_last_env_ms) >= ENV_SAMPLE_MS) {
        s_last_env_ms = now;
        if (read_environment(s_frame.env)) {
            new_data = true;
        }
    }

    return new_data;
}

// ============================================================================
//  Latest Frame Accessor
// ============================================================================
const SensorFrame &scheduler_latest_frame()
{
    return s_frame;
}
