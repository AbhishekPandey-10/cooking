#include <Arduino.h>
#include <Wire.h>

// ============================================================================
//  Disaster Health Monitoring Wearable — Production Main Loop
//
//  ESP32-S3 / PlatformIO / Arduino Framework
//
//  Module Integration:
//    Phase 1:   sensor_manager, i2c_recovery, sampling_scheduler
//    Phase 2-3: safety_engine (fall FSM, heat index, PPG qualify)
//    Phase 4.5: correlator (dual-path 45s sliding window)
//    Phase 5-6: feature_extract + anomaly_engine (z-score cold-start)
//    Phase 7.5: degraded_sos (AT state machine + P-MOSFET gating)
//    Phase 10:  ble_relay (raw ADV broadcaster + scanner + dedup ring)
//
//  Concurrency Model:
//    ┌──────────────────────────────────────────────────────────────────┐
//    │ Core 1 (Arduino loop)                                          │
//    │   ├─ scheduler_poll() — reads ISR flags, performs I2C           │
//    │   ├─ Feed sensor data → SafetyEngine, FeatureExtractor         │
//    │   ├─ 1 Hz eval tick → AnomalyEngine, Correlator                │
//    │   ├─ SOS orchestration → DegradedSos::tick()                   │
//    │   ├─ BLE RX processing → BleRelay::tick()                      │
//    │   └─ OLED update (2 Hz throttle)                               │
//    ├──────────────────────────────────────────────────────────────────┤
//    │ Core 0 (BLE Bluedroid task — managed by ESP-IDF)               │
//    │   └─ BLE scan callback → enqueue_rx() via portMUX spinlock     │
//    ├──────────────────────────────────────────────────────────────────┤
//    │ ISR Context (hardware timers)                                   │
//    │   ├─ Timer 0 → s_flag_ppg (80 Hz)                              │
//    │   └─ Timer 1 → s_flag_imu (100 Hz)                             │
//    └──────────────────────────────────────────────────────────────────┘
//
//  SRAM Budget (ESP32-S3, 512 KB total):
//    SafetyEngine        ~ 0.74 KB  (PpgQualifier, FallDetector w/
//    RollingVar<100>,
//                                    RollingVar<50>, Alert[8])
//    Correlator           ~ 0.28 KB  (45-entry window + counters + trigger
//    desc) FeatureExtractor     ~ 0.95 KB  (peak detector, RollingVar<200>)
//    Spo2Calculator       ~ 0.09 KB  (EMA baselines, R-ratio ring buffer)
//    AnomalyEngine        ~ 0.10 KB  (BaselineStats: sum[6]+sum_sq[6]+count)
//    DegradedSos          ~ 0.65 KB  (AT buffers, SMS body, payload)
//    BleRelay             ~ 0.20 KB  (RX queue, dedup ring, ADV frame)
//    SensorFrame          ~ 0.10 KB
//    ──────────────────────────────
//    Total module state   ~ 3.11 KB  (well within 512 KB)
//    Stack (main task)    ~ 8.0 KB   (separate — configurable via
//    platformio.ini)
// ============================================================================

// ---- Module headers --------------------------------------------------------
#include "config.h"
#include "sampling_scheduler.h"
#include "sensor_manager.h"
#include "sensor_types.h"

#include "anomaly_engine.h"
#include "ble_relay.h"
#include "correlator.h"
#include "degraded_sos.h"
#include "feature_extract.h"
#include "safety_engine.h"
#include "spo2_calc.h"

// ============================================================================
//  Compile-Time Configuration
// ============================================================================
namespace MainConfig {
// Display update throttle (2 Hz = 500 ms)
constexpr uint32_t DISPLAY_UPDATE_MS = 500;

// 1 Hz evaluation tick for anomaly engine + correlator
constexpr uint32_t EVAL_TICK_MS = 1000;

// SIM800L UART pins (HardwareSerial 1)
constexpr uint8_t MODEM_TX_PIN = 17;
constexpr uint8_t MODEM_RX_PIN = 18;

// BLE device ID (unique per wearable — flash-configurable in production)
constexpr uint16_t BLE_DEVICE_ID = 0xA1B2;

// Maximum consecutive I2C failures before triggering bus recovery
constexpr uint8_t I2C_FAIL_RECOVERY = 3;
} // namespace MainConfig

// ============================================================================
//  Global Module Instances
// ============================================================================

// Safety & analysis (pure logic, no hardware deps)
static SafetyEngine g_safety;
static Correlator g_correlator;
static FeatureExtractor g_feature_ext;
static AnomalyEngine g_anomaly;
static Spo2Calculator g_spo2_calc;

// Comms (hardware-backed)
static HardwareSerial ModemSerial(1); // UART1 for SIM800L
static DegradedSos g_sos(ModemSerial);
static BleRelay g_ble;

// ============================================================================
//  Timing State
// ============================================================================
static uint32_t s_last_display_ms = 0;
static uint32_t s_last_eval_ms = 0;
static uint32_t s_boot_ms = 0;

// ============================================================================
//  I2C Failure Tracking
// ============================================================================
static uint8_t s_i2c_fail_count = 0;

// ============================================================================
//  SOS State Tracking
// ============================================================================
static bool s_sos_active = false; // An SOS dispatch is in progress
static float s_last_lat = 0.0f;   // Cached position (GPS/fixed)
static float s_last_lon = 0.0f;
static uint8_t s_last_alert_code = 0;
static char s_last_alert_text[32] = {};

// ============================================================================
//  Forward Declarations
// ============================================================================
static void feed_sensors_to_engines(const SensorFrame &f, uint32_t now);
static void run_eval_tick(uint32_t now);
static void handle_sos_trigger(CorrelatorResult result, uint32_t now);
static void orchestrate_comms(uint32_t now);
static void update_display(const SensorFrame &f, uint32_t now);
static void on_ble_relay_received(const EmergencyPayload &payload);
static void build_timestamp(char *buf, size_t cap, uint32_t uptime_ms);

// ============================================================================
//
//  setup() — Boot Sequence
//
// ============================================================================
#ifndef PIO_UNIT_TESTING
void setup() {
  // ---- Serial (USB-CDC debug output) -------------------------------------
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait for USB-CDC */
  }

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("  Disaster Health Companion — ESP32-S3"));
  Serial.println(F("  Firmware: Phases 1-10 Integrated"));
  Serial.println(F("================================================"));

  s_boot_ms = millis();

  // ---- I2C Sensor Init ---------------------------------------------------
  uint8_t sensor_mask = sensors_init();

  if (sensor_mask == SENSOR_ALL) {
    Serial.println(F("[MAIN] All sensors online"));
  } else {
    Serial.printf("[MAIN] WARNING — sensors offline (mask 0x%02X)\r\n",
                  sensor_mask);
    if (!(sensor_mask & SENSOR_MAX30102))
      Serial.println(F("[MAIN]   MAX30102 (PPG/SpO2) FAILED"));
    if (!(sensor_mask & SENSOR_MPU6050))
      Serial.println(F("[MAIN]   MPU6050 (IMU) FAILED"));
    if (!(sensor_mask & SENSOR_MAX30205))
      Serial.println(F("[MAIN]   MAX30205 (Skin Temp) FAILED"));
    if (!(sensor_mask & SENSOR_BME280))
      Serial.println(F("[MAIN]   BME280 (Climate) FAILED"));
    if (!(sensor_mask & SENSOR_SSD1306))
      Serial.println(F("[MAIN]   SSD1306 (OLED) FAILED"));
  }

  // ---- Sampling Scheduler (hardware timers) ------------------------------
  scheduler_init();

  // ---- Modem UART --------------------------------------------------------
  ModemSerial.begin(SosConfig::MODEM_BAUD, SERIAL_8N1, MainConfig::MODEM_RX_PIN,
                    MainConfig::MODEM_TX_PIN);
  Serial.println(F("[MAIN] Modem UART initialized (115200 baud)"));

  // ---- Degraded SOS Engine -----------------------------------------------
  g_sos.begin(s_boot_ms);
  g_sos.set_debug(&Serial);
  g_sos.set_emergency_number("+911234567890"); // Configure per deployment
  Serial.println(F("[MAIN] DegradedSos engine initialized"));

  // ---- BLE Relay ---------------------------------------------------------
  g_ble.begin(MainConfig::BLE_DEVICE_ID, "DIS-SOS");
  g_ble.set_debug(&Serial);
  g_ble.set_relay_callback(on_ble_relay_received);
  g_ble.start_scanner();
  Serial.println(F("[MAIN] BLE relay initialized (scanner active)"));

  // ---- Anomaly Engine Cold-Start -----------------------------------------
  g_anomaly.begin(s_boot_ms);
  Serial.println(F("[MAIN] Anomaly engine: DORMANT (60-min cold-start)"));

  // ---- Fixed position (no GPS — set per deployment or via BLE config) ----
  s_last_lat = 28.6139f; // Default: New Delhi
  s_last_lon = 77.2090f;
  Serial.printf("[MAIN] Fixed position: %.4f, %.4f\r\n", s_last_lat,
                s_last_lon);

  // ---- Boot complete -----------------------------------------------------
  Serial.println(F("[MAIN] ════════════════════════════════════════"));
  Serial.println(F("[MAIN] Boot complete. Entering main loop."));
  Serial.println(F("[MAIN] ════════════════════════════════════════"));
}

// ============================================================================
//
//  loop() — Main Execution Loop (runs on Core 1)
//
//  Cadence:
//    Every iteration:
//      1. Poll scheduler (ISR flags → I2C reads)
//      2. Feed new data into safety + feature engines
//      3. Every 1s: run anomaly eval + correlator tick
//      4. Handle SOS triggers
//      5. Tick comms (degraded_sos + ble_relay)
//      6. Every 500ms: update OLED display
//
// ============================================================================
void loop() {
  uint32_t now = millis();

  // ========================================================================
  //  Step 1: Poll sensor scheduler (ISR-flag-driven I2C reads)
  // ========================================================================
  bool new_data = scheduler_poll();
  const SensorFrame &frame = scheduler_latest_frame();

  // Track I2C health — trigger bus recovery if reads keep failing
  if (new_data) {
    s_i2c_fail_count = 0;
  } else {
    // Only count failures when we expected data (ISR fired but read failed)
    // The scheduler returns false both when no ISR fired and when reads fail.
    // We rely on the scheduler's internal error handling + periodic recovery.
  }

  // ========================================================================
  //  Step 2: Feed sensor data into safety + feature engines
  // ========================================================================
  if (new_data) {
    feed_sensors_to_engines(frame, now);
  }

  // ========================================================================
  //  Step 3: 1 Hz synchronous evaluation tick
  // ========================================================================
  if ((now - s_last_eval_ms) >= MainConfig::EVAL_TICK_MS) {
    s_last_eval_ms = now;
    run_eval_tick(now);
  }

  // ========================================================================
  //  Step 4: Tick communications (non-blocking)
  // ========================================================================
  orchestrate_comms(now);

  // ========================================================================
  //  Step 5: Throttled OLED display update (2 Hz)
  // ========================================================================
  if ((now - s_last_display_ms) >= MainConfig::DISPLAY_UPDATE_MS) {
    s_last_display_ms = now;
    update_display(frame, now);
  }
}
#endif // !PIO_UNIT_TESTING

// ============================================================================
//
//  feed_sensors_to_engines() — Route sensor data into processing modules
//
//  Called on every scheduler_poll() cycle that produces new data.
//  High-rate: PPG at 80 Hz, IMU at 100 Hz.
//  Low-rate:  Skin temp at 0.05 Hz, Env at 0.02 Hz.
//
// ============================================================================
static void feed_sensors_to_engines(const SensorFrame &f, uint32_t now) {
  // ---- PPG → Spo2Calculator → SafetyEngine + FeatureExtractor ------------
  //  Feed both Red and IR channels to the streaming SpO2 calculator.
  //  The calculator extracts AC/DC components via EMA, detects cardiac
  //  cycle boundaries, and computes the R-ratio → SpO2 mapping.
  //  SpO2 is valid once MIN_VALID_CYCLES (4) cardiac cycles are captured.
  g_spo2_calc.feed(f.ppg.red, f.ppg.ir, f.ppg.timestamp_ms);

  //  Pass the computed SpO2 to SafetyEngine for threshold evaluation.
  //  PpgQualifier inside SafetyEngine gates all SpO2 alerting on skin
  //  contact and motion artifact checks.
  g_safety.feed_ppg(f.ppg.ir,
                    g_spo2_calc.spo2(),
                    g_spo2_calc.valid(), f.ppg.timestamp_ms);

  g_feature_ext.feed_ppg_ir(static_cast<float>(f.ppg.ir), f.ppg.timestamp_ms);

  // ---- IMU → SafetyEngine + FeatureExtractor -----------------------------
  g_safety.feed_imu(f.imu.ax, f.imu.ay, f.imu.az, f.imu.timestamp_ms);

  g_feature_ext.feed_imu(f.imu.ax, f.imu.ay, f.imu.az);

  // ---- Skin Temp → FeatureExtractor (low rate, every 20s) ----------------
  //  Only feed when a new reading is available (timestamp changes).
  static uint32_t s_last_skin_ts = 0;
  if (f.temp.timestamp_ms != s_last_skin_ts && f.temp.timestamp_ms > 0) {
    s_last_skin_ts = f.temp.timestamp_ms;
    g_feature_ext.feed_skin_temp(f.temp.skin_temp_c, f.temp.timestamp_ms);
  }

  // ---- Environment → SafetyEngine (low rate, every 50s) ------------------
  static uint32_t s_last_env_ts = 0;
  if (f.env.timestamp_ms != s_last_env_ts && f.env.timestamp_ms > 0) {
    s_last_env_ts = f.env.timestamp_ms;
    g_safety.feed_environment(f.env.bme_temp_c, f.env.bme_humidity,
                              f.env.timestamp_ms);
  }
}

// ============================================================================
//
//  run_eval_tick() — 1 Hz Synchronous Analysis
//
//  1. Extract feature vector from streaming buffers
//  2. Evaluate anomaly engine (z-score / baseline)
//  3. Feed SpO2 to feature extractor (from safety engine cache)
//  4. Tick the correlator with current metrics
//  5. Handle any triggered SOS
//
// ============================================================================
static void run_eval_tick(uint32_t now) {
  const SensorFrame &f = scheduler_latest_frame();

  // ---- Feature Extraction → Anomaly Engine (Phase 5-6) -------------------
  //  Feed latest SpO2 from safety engine cache
  int32_t spo2 = g_safety.last_spo2();
  bool spo2_valid = (g_safety.ppg_state() == PpgState::VALID && spo2 > 0);
  g_feature_ext.feed_spo2(spo2, spo2_valid);

  FeatureVector fv = g_feature_ext.extract(now);
  float confidence = g_anomaly.evaluate(fv, now);

  // ---- Correlator Tick (Phase 4.5) ---------------------------------------
  bool mq135_valid =
      f.env.mq135_valid && ((now - s_boot_ms) >= MQ135_WARMUP_MS);

  CorrelatorResult cr = g_correlator.tick(
      spo2, spo2_valid, g_safety.is_fall_detected(), g_safety.last_heat_index(),
      f.env.mq135_raw, mq135_valid, f.temp.skin_temp_c, now);

  // ---- Handle SOS trigger ------------------------------------------------
  if (cr != CorrelatorResult::NONE) {
    handle_sos_trigger(cr, now);
  }

  // ---- Periodic status logging (every 10s) -------------------------------
  static uint32_t s_last_status_log = 0;
  if ((now - s_last_status_log) >= 10000) {
    s_last_status_log = now;
    Serial.printf("[EVAL] PPG=%s SpO2=%ld Fall=%s HI=%.1f°C "
                  "Anomaly=%.2f(%s) Corr=%d/4 SOS=%s\r\n",
                  g_safety.ppg_status_string(), (long)spo2,
                  g_safety.is_fall_detected() ? "YES" : "no",
                  g_safety.last_heat_index(), g_anomaly.anomaly_score(),
                  g_anomaly.state() == AnomalyState::DORMANT  ? "DORMANT"
                  : g_anomaly.state() == AnomalyState::ACTIVE ? "ACTIVE"
                                                              : "SUSPENDED",
                  g_correlator.active_flag_count(),
                  s_sos_active ? "ACTIVE" : "idle");
  }
}

// ============================================================================
//
//  handle_sos_trigger() — Escalation Decision
//
//  Path A (hard bypass): Fall or SpO2 < 90% → immediate CRITICAL
//  Path B (correlated):  ≥ 2-of-4 in 45s window → WARNING
//
// ============================================================================
static void handle_sos_trigger(CorrelatorResult result, uint32_t now) {
  if (s_sos_active) {
    // Already dispatching — don't re-trigger
    Serial.println(F("[SOS] Trigger ignored — dispatch already active"));
    return;
  }

  uint8_t alert_code = 0;
  const char *alert_text = "";

  switch (result) {
  case CorrelatorResult::PATH_A_HARD_BYPASS:
    // Determine specific cause
    if (g_safety.is_fall_detected()) {
      alert_code = BleRelayConfig::ALERT_FALL; // 0x01
      strncpy(s_last_alert_text, "FALL DETECTED", sizeof(s_last_alert_text));
    } else {
      alert_code = BleRelayConfig::ALERT_CARDIAC; // 0x02
      strncpy(s_last_alert_text, "SPO2 CRITICAL", sizeof(s_last_alert_text));
    }
    Serial.printf("[SOS] *** PATH A HARD BYPASS: %s ***\r\n",
                  s_last_alert_text);
    break;

  case CorrelatorResult::PATH_B_CORRELATED:
    alert_code = BleRelayConfig::ALERT_HEAT_GAS; // 0x03
    snprintf(s_last_alert_text, sizeof(s_last_alert_text), "CORR %s",
             g_correlator.last_trigger_description());
    Serial.printf("[SOS] *** PATH B CORRELATED: %s ***\r\n",
                  g_correlator.last_trigger_description());
    break;

  default:
    return;
  }

  s_last_alert_code = alert_code;
  s_sos_active = true;

  // Build timestamp from uptime
  char ts[24];
  build_timestamp(ts, sizeof(ts), now);

  // Hand off to degraded SOS engine
  g_sos.request_sos(s_last_lat, s_last_lon, alert_code, s_last_alert_text, ts);

  Serial.printf("[SOS] Dispatched: lat=%.4f lon=%.4f code=0x%02X\r\n",
                s_last_lat, s_last_lon, alert_code);
}

// ============================================================================
//
//  orchestrate_comms() — Tick SOS + BLE, handle fallback transitions
//
// ============================================================================
static void orchestrate_comms(uint32_t now) {
  // ---- Tick the AT state machine (non-blocking) --------------------------
  g_sos.tick(now);

  // ---- Tick the BLE relay (process RX queue) -----------------------------
  g_ble.tick(now);

  // ---- Check for cellular → BLE fallback transition ----------------------
  if (s_sos_active && g_sos.ble_fallback_needed() && !g_ble.is_broadcasting()) {
    Serial.println(
        F("[MAIN] Cellular exhausted — activating BLE burst broadcast"));
    g_ble.broadcast_emergency(s_last_alert_code, s_last_lat, s_last_lon);
  }

  // ---- Check for SOS completion ------------------------------------------
  if (s_sos_active && g_sos.phase() == SosPhase::DONE_SUCCESS) {
    Serial.println(F("[MAIN] *** SOS DELIVERED SUCCESSFULLY ***"));
    g_sos.acknowledge();
    g_ble.stop_broadcast();
    g_safety.reset_fall();
    s_sos_active = false;
  }

  // ---- If SOS via BLE is active, check if we should retry cellular -------
  //  The degraded_sos sleep/wake cycle handles this automatically.
  //  When it wakes and finds signal, it will send the SMS and transition
  //  to DONE_SUCCESS, which we catch above.
}

// ============================================================================
//
//  on_ble_relay_received() — BLE relay callback (called from tick context)
//
//  When this device receives a peer's emergency broadcast with hop_count == 0,
//  the BLE relay engine calls this function.  We forward the decoded payload
//  to the degraded_sos engine for cellular SMS dispatch.
//
// ============================================================================
static void on_ble_relay_received(const EmergencyPayload &payload) {
  float lat = decode_coord(payload.trunc_lat);
  float lon = decode_coord(payload.trunc_lon);

  Serial.printf("[RELAY] Forwarding peer SOS: devid=0x%04X alert=0x%02X "
                "lat=%.4f lon=%.4f\r\n",
                payload.device_id, payload.alert_code, lat, lon);

  // Build relay timestamp
  char ts[24];
  build_timestamp(ts, sizeof(ts), millis());

  // Format alert text for relay context
  char relay_text[32];
  snprintf(relay_text, sizeof(relay_text), "RELAY-0x%04X", payload.device_id);

  // Dispatch via cellular (if this device has signal)
  g_sos.request_sos(lat, lon, payload.alert_code, relay_text, ts);
}

// ============================================================================
//
//  update_display() — OLED UI (2 Hz refresh, I2C-bus-contention-safe)
//
//  Layout (128×64):
//    Line 0:  PPG status / HR       "♥ 72 bpm" or "Calibrating..."
//    Line 1:  SpO2                   "SpO2: 97%"
//    Line 2:  Skin Temp / HI         "Sk:36.5 HI:32.1"
//    Line 3:  Signal / SOS status    "CSQ:18 ▲" or "BLE TX"
//    Line 4:  Anomaly state          "AE:DORMANT 42min"
//    Line 5:  Alert (if any)         "⚠ FALL DETECTED"
//
// ============================================================================
static void update_display(const SensorFrame &f, uint32_t now) {
  // The actual OLED rendering is handled by display_update() in
  // sensor_manager.cpp (which owns the SSD1306 driver instance).
  // Here we enrich the frame with safety/comms state before passing it.
  //
  // For now, delegate to the existing display_update function.
  // In production, replace with a richer UI that shows alerts,
  // correlator state, and comms status.
  display_update(f);

  // ---- Print critical alerts to Serial as well ---------------------------
  if (g_safety.alert_count() > 0) {
    for (size_t i = 0; i < g_safety.alert_count(); i++) {
      const Alert &a = g_safety.alerts()[i];
      Serial.printf("[ALERT] %s: %s\n",
                    a.severity == AlertSeverity::CRITICAL ? "CRITICAL"
                                                          : "WARNING",
                    a.message);
    }
  }
}

// ============================================================================
//  Utility: Build ISO-like timestamp from uptime
// ============================================================================
static void build_timestamp(char *buf, size_t cap, uint32_t uptime_ms) {
  uint32_t secs = uptime_ms / 1000;
  uint32_t mins = secs / 60;
  uint32_t hours = mins / 60;
  snprintf(buf, cap, "T+%02lu:%02lu:%02lu", (unsigned long)(hours % 100),
           (unsigned long)(mins % 60), (unsigned long)(secs % 60));
}
