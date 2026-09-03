# Testing & Verification Guide

> How to run unit tests, interpret results, and perform hardware bench testing.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Unit Test Suites](#unit-test-suites)
- [Running Tests](#running-tests)
- [Test Architecture](#test-architecture)
- [Bench Test: BLE Relay (Dual-Device)](#bench-test-ble-relay-dual-device)
- [Bench Test: Fall Detection](#bench-test-fall-detection)
- [Bench Test: SOS Escalation](#bench-test-sos-escalation)
- [Adding New Tests](#adding-new-tests)

---

## Quick Start

```bash
# Run all test suites on the ESP32-S3 (180 tests across 6 suites)
pio test

# Run a specific suite
pio test --filter test_safety
pio test --filter test_correlator
pio test --filter test_anomaly
pio test --filter test_degraded_sos
pio test --filter test_ble_relay
pio test --filter test_sensors

# Run with verbose output
pio test -v
```

---

## Unit Test Suites

### test_safety (39 tests)

Source: `test/test_safety/test_safety_engine.cpp`  
Tests rolling variance, Rothfusz heat index, PPG qualification, fall FSM, SpO2 thresholds, and alert suppression.

| Test | What It Verifies |
|---|---|
| `test_rolling_variance_constant_input` | Rolling variance of identical values is 0.0 |
| `test_rolling_variance_known_values` | Rolling variance of known dataset matches mathematical formula |
| `test_rolling_variance_sliding_eviction` | FIFO eviction updates mean and variance correctly |
| `test_rolling_variance_reset` | Buffer reset clears sum, sum_sq, count, and buffer elements |
| `test_heat_index_steadman_region` | Steadman formula fallback active when T < 26.7°C or RH < 40% |
| `test_heat_index_rothfusz_known_90F_50RH` | Full 9-term polynomial matches NWS 90°F / 50% RH benchmark |
| `test_heat_index_high_humidity_amplification` | High RH (> 85%) adjustment addition term applied |
| `test_heat_index_low_humidity_adjustment` | Low RH (< 13%) adjustment subtraction term applied |
| `test_heat_index_symmetry_with_nws_chart` | Monotonic temperature increase yields monotonic heat index increase |
| `test_heat_index_clamps_rh` | RH input clamped to [0, 100]% |
| `test_ppg_starts_calibrating` | PpgQualifier initializes to CALIBRATING state on boot |
| `test_ppg_buffer_fill_to_valid` | 100 samples with IR > 5000 transition state to VALID |
| `test_ppg_contact_lost_resets_counter` | IR < 5000 transitions to CONTACT_LOST and resets fill counter |
| `test_ppg_contact_boundary_5000` | Exact 5000 count boundary treated as valid contact |
| `test_ppg_motion_artifact_masking` | Accel variance > 0.3 (m/s²)² forces MOTION_ARTIFACT state |
| `test_ppg_motion_ignored_during_calibration` | Motion artifact does not override CALIBRATING state |
| `test_fall_normal_activity_no_trigger` | 1g baseline does not trigger freefall |
| `test_fall_freefall_timeout_no_impact` | Freefall (< 0.2g) without impact (> 3.0g) within 300ms resets to IDLE |
| `test_fall_freefall_then_impact` | Freefall followed by impact transitions to IMPACT_DETECTED |
| `test_fall_complete_sequence` | Freefall → impact → 5s immobility (< 0.05 variance) confirms fall |
| `test_fall_movement_after_impact_prevents_confirmation` | Motion (> 0.05 variance) during immobility window resets FSM to IDLE |
| `test_fall_reset` | Manual reset clears fall state back to IDLE |
| `test_spo2_immediate_critical_below_90` | SpO2 < 90% triggers immediate CRITICAL alert |
| `test_spo2_at_90_no_immediate_critical` | SpO2 = 90% does not trigger immediate critical escalation |
| `test_spo2_sustained_warning_at_91` | SpO2 < 92% triggers WARNING after 30s sustained window |
| `test_spo2_recovery_resets_sustained_timer` | SpO2 returning ≥ 92% resets the sustained timer |
| `test_spo2_at_92_no_warning` | SpO2 = 92% does not trigger warning |
| `test_calibrating_suppresses_spo2` | All SpO2 alerts suppressed while PPG is CALIBRATING |
| `test_contact_loss_suppresses_spo2` | All SpO2 alerts suppressed while PPG is CONTACT_LOST |
| `test_contact_loss_triggers_recalibration` | Regaining contact requires full 100-sample buffer re-fill |
| `test_motion_gates_ppg_to_artifact` | High IMU variance sets MOTION_ARTIFACT in SafetyEngine feed |
| `test_motion_artifact_suppresses_spo2` | All SpO2 alerts suppressed while PPG is in MOTION_ARTIFACT |
| `test_oled_calibrating_string` | CALIBRATING state returns "Calibrating Vitals..." |
| `test_oled_contact_lost_string` | CONTACT_LOST state returns "Calibrating Vitals..." |
| `test_oled_valid_string` | VALID state returns "Vitals OK" |
| `test_heat_warning_at_40c` | Heat index ≥ 40°C raises WARNING alert |
| `test_heat_no_alert_mild` | Mild heat index (< 40°C) produces no heat alert |
| `test_heat_critical_extreme` | Heat index ≥ 54°C raises CRITICAL alert |
| `test_multiple_simultaneous_alerts` | Multiple concurrent alerts correctly tracked in active alert array |

---

### test_correlator (35 tests)

Source: `test/test_correlator/test_correlator.cpp`  
Tests Path A hard bypass, 45s sliding window coincidence logic, per-flag thresholds, and MQ135 delta tracking.

| Test | What It Verifies |
|---|---|
| `test_path_a_fall_detected` | Confirmed fall triggers immediate PATH_A_HARD_BYPASS |
| `test_path_a_spo2_below_90` | Valid SpO2 < 90% triggers immediate PATH_A_HARD_BYPASS |
| `test_path_a_spo2_very_low` | Extreme SpO2 (e.g. 75%) triggers PATH_A_HARD_BYPASS |
| `test_path_a_boundary_spo2_90_is_not_bypass` | SpO2 = 90% does not trigger Path A (evaluates into Path B) |
| `test_path_a_spo2_invalid_does_not_trigger` | Invalid SpO2 (< 90%) ignored by Path A |
| `test_path_a_preempts_path_b` | Path A condition overrides active Path B coincidence |
| `test_optical_flag_at_90` | SpO2 = 90% sets optical flag |
| `test_optical_flag_at_92` | SpO2 = 92% sets optical flag |
| `test_optical_flag_not_at_93` | SpO2 = 93% does not set optical flag |
| `test_heat_flag` | Heat index > 38°C sets heat flag |
| `test_heat_flag_at_threshold_exact` | Heat index = 38.0°C does not set heat flag (strictly > 38.0) |
| `test_gas_flag_delta_above_threshold` | MQ135 \|Δ\| > 150 counts sets gas flag |
| `test_gas_flag_delta_below_threshold` | MQ135 \|Δ\| ≤ 150 counts does not set gas flag |
| `test_gas_flag_no_previous_reading` | First valid MQ135 reading establishes baseline, does not trigger |
| `test_gas_flag_negative_delta` | Negative drop > 150 counts also sets gas flag (absolute delta) |
| `test_gas_flag_mq135_invalid_skipped` | Uncalibrated / warmup readings (mq135_valid=false) skipped |
| `test_thermal_flag_cold` | Skin temp < 30.0°C sets thermal flag |
| `test_thermal_flag_hot` | Skin temp > 38.5°C sets thermal flag |
| `test_thermal_flag_normal` | Normal skin temp (30.0°C–38.5°C) does not set thermal flag |
| `test_single_flag_no_alert` | Single active flag in 45s window returns NONE |
| `test_two_flags_fires_alert` | Two active flags in window returns PATH_B_CORRELATED |
| `test_three_flags_fires_alert` | Three active flags in window returns PATH_B_CORRELATED |
| `test_all_four_flags_fires_alert` | All four active flags in window returns PATH_B_CORRELATED |
| `test_two_flags_noncontiguous_still_fires` | Two flags appearing at different seconds within 45s trigger |
| `test_flag_expires_from_window` | Flags older than 45 ticks expire from running counter |
| `test_expired_flag_prevents_alert` | Once older flag expires, single remaining flag returns NONE |
| `test_reset_clears_window` | Reset clears all circular buffer entries and running counters |
| `test_no_immediate_refire_after_reset` | Coincident trigger auto-resets window, preventing repeat alerts |
| `test_single_flag_after_reset_requires_second` | Post-reset requires new coincidence to fire again |
| `test_mq135_baseline_survives_reset` | MQ135 baseline tracking is preserved across window resets |
| `test_trigger_description_populated` | Last trigger description string populated on Path B alert |
| `test_trigger_description_shows_active_flags` | Description lists exact active flags and hit counts |
| `test_trigger_description_survives_reset` | Trigger description remains accessible after window reset |
| `test_window_fill_count` | Window fill counter increments up to WINDOW_SIZE (45) |
| `test_active_count_reflects_distinct_categories` | active_flag_count() returns distinct categories with count > 0 |

---

### test_anomaly (26 tests)

Source: `test/test_anomaly/test_anomaly_engine.cpp`  
Tests peak detection, HRV extraction, running statistics, cold-start lifecycle, and score dampening.

| Test | What It Verifies |
|---|---|
| `test_peak_detector_finds_peaks` | EMA peak detector identifies synthetic sine peaks |
| `test_peak_detector_hr_computation` | Heart rate computed correctly from recent peak timestamps |
| `test_peak_detector_rmssd_needs_3_peaks` | RMSSD computation requires at least 3 peaks (2 intervals) |
| `test_peak_detector_reset` | Reset clears EMA state and peak circular buffer |
| `test_feature_vector_all_valid` | all_valid() returns true when all 6 features are valid |
| `test_feature_vector_partial_valid` | valid_count() tracks valid feature count accurately |
| `test_feature_vector_named_accessors` | Named accessors map to correct array indices |
| `test_extractor_accel_variance` | Accel variance matches mathematical variance of tri-axial stream |
| `test_extractor_skin_temp_slope` | Skin temp slope correctly calculated in °C/min |
| `test_extractor_spo2_bounds` | SpO2 values outside [70, 100]% clamped or rejected |
| `test_extractor_spo2_invalid` | Invalid SpO2 flag correctly reflected in FeatureVector |
| `test_baseline_mean_computation` | BaselineStats accurately computes arithmetic mean per feature |
| `test_baseline_std_constant_input` | Standard deviation of identical inputs is 0.0 |
| `test_baseline_std_with_variation` | Standard deviation matches population formula |
| `test_engine_starts_dormant` | AnomalyEngine boots in DORMANT state |
| `test_engine_stays_dormant_before_60min` | Engine remains DORMANT before 60-minute duration expires |
| `test_engine_transitions_to_active_after_60min` | Engine transitions to ACTIVE and calibrates θ after 60 min |
| `test_engine_normal_features_low_score` | Features matching baseline yield low anomaly score (< θ) |
| `test_engine_anomalous_features_high_score` | Significant outliers yield high anomaly score (> θ) |
| `test_engine_handles_partial_invalid` | Up to 2 invalid features substituted with baseline mean |
| `test_engine_suspends_on_too_many_invalid` | > 2 invalid features or > 10 consecutive failures → SUSPENDED |
| `test_engine_motion_cooldown_dampens_score` | Active motion dampens anomaly score by 50% for 5 minutes |
| `test_engine_force_reset` | force_reset() resets engine back to DORMANT state |
| `test_confidence_neutral_during_dormancy` | DORMANT state returns neutral confidence modifier (0.5) |
| `test_baseline_empty_returns_zero_mean` | Baseline with zero samples safely returns 0.0 mean |
| `test_baseline_single_sample_variance` | Baseline with 1 sample safely returns 0.0 variance |

---

### test_degraded_sos (35 tests)

Source: `test/test_degraded_sos/test_degraded_sos.cpp`  
Tests AT command engine, CSQ/CREG parsers, signal state machine, backoff timing, and sleep/wake cycles.

| Test | What It Verifies |
|---|---|
| `test_at_ok_response` | "OK" response parsed to AtResult::OK |
| `test_at_error_response` | "+CMS ERROR" parsed to AtResult::ERROR with data capture |
| `test_at_cme_error` | "+CME ERROR" parsed to AtResult::ERROR |
| `test_at_timeout` | Command times out after specified deadline |
| `test_at_echo_suppression` | Sent command echoed by modem is stripped |
| `test_at_garbage_characters` | Non-printable framing bytes ignored |
| `test_at_prompt_detection` | "> " prompt parsed to AtResult::PROMPT |
| `test_at_data_line_stored` | Informational lines (+CSQ, +CREG) preserved in buffer |
| `test_at_command_transmitted` | Outgoing command string correctly written to UART |
| `test_at_buffer_overrun` | Overlong serial lines truncated safely without crash |
| `test_at_fragmented_response` | Serial response split across multiple ticks parsed correctly |
| `test_parse_csq_good` | "+CSQ: 20,0" parsed to 20 |
| `test_parse_csq_marginal` | "+CSQ: 10,2" parsed to 10 |
| `test_parse_csq_none` | "+CSQ: 3,0" parsed to 3 |
| `test_parse_csq_unknown` | "+CSQ: 99,99" parsed to 99 |
| `test_parse_csq_invalid` | Non-conforming CSQ response returns -1 |
| `test_parse_creg_registered_home` | "+CREG: 0,1" parsed to stat 1 (home network) |
| `test_parse_creg_searching` | "+CREG: 0,2" parsed to stat 2 (searching) |
| `test_parse_creg_roaming` | "+CREG: 0,5" parsed to stat 5 (roaming) |
| `test_parse_creg_not_registered` | "+CREG: 0,0" parsed to stat 0 (not registered) |
| `test_parse_creg_invalid` | Non-conforming CREG response returns -1 |
| `test_signal_good` | CSQ ≥ 15 and CREG 1 or 5 classified as SignalState::GOOD |
| `test_signal_marginal` | 5 ≤ CSQ ≤ 14 and CREG 1 or 5 classified as SignalState::MARGINAL |
| `test_signal_none_low_csq` | CSQ < 5 classified as SignalState::NONE |
| `test_signal_none_not_registered` | Good CSQ but CREG not registered classified as SignalState::NONE |
| `test_signal_none_negative_csq` | Negative/invalid CSQ classified as SignalState::NONE |
| `test_sms_full_format` | Full SMS includes "SOS ALERT", 6dp coords, text, and timestamp |
| `test_sms_compact_format` | Compact SMS formats to single line under 40 bytes with 4dp coords |
| `test_boot_delay_before_at` | 4000ms boot capacitor charge window enforced before first AT command |
| `test_full_sos_good_signal` | Happy path: boot → ATE0 → CMGF → CSQ → CREG → CMGS → SUCCESS |
| `test_no_signal_ble_fallback` | Low CSQ cuts modem power and sets ble_fallback_needed flag |
| `test_marginal_backoff_retry` | Failed SMS in marginal signal triggers 5000ms backoff and re-probe |
| `test_retries_exhausted` | 3 failed retries exhaust cellular attempts → BLE fallback |
| `test_sleep_wake_cycle` | 120s sleep cycle followed by modem re-power and re-attempt |
| `test_modem_unresponsive` | Modem timeout during init powers off modem |

---

### test_ble_relay (36 tests)

Source: `test/test_ble_relay/test_ble_relay.cpp`  
Tests packet bit-packing, CRC-16-CCITT, coordinate fixed-point encoding, raw AD framing, dedup ring buffer, and single-hop drop logic.

| Test | What It Verifies |
|---|---|
| `test_payload_size` | EmergencyPayload packed size is exactly 15 bytes |
| `test_payload_field_offsets` | Byte offsets match protocol layout (0, 2, 3, 7, 11, 12, 13) |
| `test_crc16_known_vector` | "123456789" produces standard CRC-16-CCITT 0x29B1 |
| `test_crc16_empty` | Empty input returns initial vector 0xFFFF |
| `test_crc16_single_byte` | Single byte input computes valid CRC |
| `test_packet_crc_round_trip` | Self-computed packet CRC validates true |
| `test_packet_crc_detects_corruption` | Single-byte alteration fails CRC validation |
| `test_packet_crc_detects_seq_change` | Sequence number modification fails CRC validation |
| `test_encode_positive_coord` | Positive longitude (151.215297) encoded to fixed-point integer |
| `test_encode_negative_coord` | Negative latitude (-33.856784) encoded to fixed-point integer |
| `test_decode_round_trip` | Encode and decode preserves coordinate within ±0.0002° (~11m) |
| `test_encode_zero` | 0.0 coordinate encodes to 0 integer |
| `test_decode_precision` | 4 decimal place precision verified against floating point |
| `test_build_adv_data_length` | Total raw advertising frame is exactly 22 bytes |
| `test_build_adv_data_flags` | AD Flags structure is 0x02, 0x01, 0x06 |
| `test_build_adv_data_manufacturer_header` | Manufacturer header contains length 0x12, type 0xFF, CID 0xFFFF |
| `test_build_adv_data_payload_bytes` | Payload starts at offset 7 with intact binary fields |
| `test_build_adv_data_too_small_buffer` | Buffer < 22 bytes returns 0 bytes written |
| `test_parse_adv_data_round_trip` | Parsed frame perfectly reconstructs original payload |
| `test_parse_adv_data_wrong_company_id` | Non-0xFFFF Company ID ignored during parsing |
| `test_parse_adv_data_truncated` | Truncated frame rejected |
| `test_parse_adv_data_garbage_prefix` | Frame preceded by arbitrary AD structures parsed correctly |
| `test_dedup_empty_not_found` | Empty ring buffer returns false for contains() |
| `test_dedup_insert_and_find` | Inserted {device_id, seq} found in ring buffer |
| `test_dedup_different_seq_not_found` | Matching device with different sequence number not found |
| `test_dedup_multiple_entries` | Multiple distinct entries tracked simultaneously |
| `test_dedup_wraps_at_capacity` | 17th insertion evicts oldest entry in 16-entry ring buffer |
| `test_dedup_clear` | clear() flushes all entries |
| `test_relay_valid_origin_packet` | Valid packet with hop_count=0 returns RelayDecision::RELAY |
| `test_relay_drop_invalid_crc` | Packet with corrupted CRC returns RelayDecision::DROP_INVALID_CRC |
| `test_relay_drop_hop_exceeded` | Packet with hop_count ≥ 1 returns RelayDecision::DROP_HOP_EXCEEDED |
| `test_relay_drop_duplicate` | Repeated packet returns RelayDecision::DROP_DUPLICATE |
| `test_relay_different_seq_not_duplicate` | Different sequence number from same device is relayed |
| `test_relay_different_device_not_duplicate` | Same sequence number from different device is relayed |
| `test_relay_priority_order_crc_first` | Corrupted packet with hop_count=1 dropped for CRC first |
| `test_e2e_serialize_parse_validate_relay` | Full end-to-end: origin build → raw AD → parse → CRC → relay → duplicate drop |

---

### test_sensors (9 tests)

Source: `test/test_sensors/test_sensor_reads.cpp`  
Tests data structure zero-initialization, warmup blackout timing, and vector magnitude calculations.

| Test | What It Verifies |
|---|---|
| `test_ppg_reading_zero_init` | PpgReading zero-initializes cleanly |
| `test_imu_reading_zero_init` | ImuReading zero-initializes cleanly |
| `test_env_reading_mq135_invalid_by_default` | EnvReading.mq135_valid defaults to false |
| `test_sensor_frame_aggregate` | SensorFrame aggregates all reading structures |
| `test_mq135_warmup_blackout` | MQ135 marked invalid before 90000ms warmup elapses |
| `test_mq135_warmup_with_nonzero_boot` | Warmup calculations respect non-zero boot timestamp |
| `test_accel_magnitude_stationary` | 1g stationary acceleration produces ~9.8 m/s² magnitude |
| `test_accel_magnitude_3d` | 3D acceleration vector magnitude correctly computed |
| `test_imu_scale_conversion` | Raw LSB to m/s² and °/s scale conversions accurate |

---

## Test Architecture

### Mocking Strategy

Tests run on target or in desktop environments using the Unity test framework. Hardware dependencies are cleanly decoupled:

| Module Under Test | Mock/Stub | Approach |
|---|---|---|
| SafetyEngine | None | Pure deterministic algorithms and arithmetic |
| Correlator | None | Pure stateful circular buffer logic |
| AnomalyEngine | None | Pure statistical vector calculations |
| DegradedSos | `MockStream` | In-memory ring-buffered Stream simulating SIM800L UART |
| BleRelay | None (in test) | Tests execute pure packet protocol via `ble_relay_types.h` |
| Sensor data structures | None | Pure C++ struct validation |

---

## Bench Test: BLE Relay (Dual-Device)

### Equipment

- **Unit A**: ESP32-S3 DevKit + SIM800L, wrapped in RF-absorbing foil (cellular blocked, BLE exposed)
- **Unit B**: ESP32-S3 DevKit + SIM800L with active SIM card (cellular connected)
- 2× USB cables, 2× serial monitors at 115200 baud

### Procedure

1. Flash both units with the production firmware (`pio run --target upload`)
2. Open serial monitors for both units
3. Verify Unit B shows: `[BLE] Scanner active (passive, continuous)`
4. On Unit A, trigger a fall alert (shake/drop the IMU, or inject via serial command)
5. Observe the following sequence:

### Expected Serial Output

**Unit A (Shielded Broadcaster):**

```
[SOS] IDLE -> POWERING_ON
[SOS] Modem power ON
[SOS] POWERING_ON -> INIT_ECHO_OFF
...
[SOS] Signal: NONE (CSQ=0, CREG=0)
[SOS] No signal — triggering BLE fallback
[MAIN] Cellular exhausted — activating BLE burst broadcast
[BLE] Emergency broadcast: alert=0x01 lat=-338568 lon=1512153 seq=0 hop=0 crc=0x1A2B
[BLE] ADV started (250ms burst, non-connectable)
```

**Unit B (Connected Relay):**

```
[BLE] Scanner active (passive, continuous)
[BLE] RX: devid=0xA1B2 alert=0x01 lat=-338568 lon=1512153 seq=0 hop=0 crc=0x1A2B RSSI=-52
[BLE] Decision: RELAY
[RELAY] Forwarding peer SOS: devid=0xA1B2 alert=0x01 lat=-33.8568 lon=151.2153
[SOS] IDLE -> POWERING_ON
...
[SOS] *** SMS SENT SUCCESSFULLY ***
[BLE] RX: devid=0xA1B2 alert=0x01 ... seq=0 ... RSSI=-54
[BLE] Decision: DROP_DUPLICATE
```

---

## Bench Test: Fall Detection

### Procedure

1. Flash firmware and open serial monitor
2. Hold the device steady for 5 seconds (establish baseline)
3. Drop the device from ~1m onto a padded surface
4. Leave it motionless for 6+ seconds after impact

### Expected State Transitions

```
[EVAL] Fall=no     (IDLE — normal monitoring)
[ALERT] ...        (device dropped — freefall detected)
[EVAL] Fall=no     (FREEFALL_DETECTED — waiting for impact within 300ms)
[ALERT] ...        (impact detected)
[EVAL] Fall=no     (IMPACT_DETECTED — monitoring immobility for 5s)
[ALERT] CRITICAL: FALL DETECTED
[SOS] *** PATH A HARD BYPASS: FALL DETECTED ***
```

---

## Bench Test: SOS Escalation

### Good Signal Test

1. Insert SIM card with SMS allowance
2. Ensure antenna is connected and location has cellular coverage
3. Trigger any alert (fall, SpO2 threshold, or correlator)
4. Verify SMS is received within 15 seconds

### Marginal Signal Test

1. Partially shield the SIM800L antenna (reduce but don't eliminate signal)
2. Trigger an alert
3. Observe compact SMS format and retry backoff in serial log
4. Verify SMS arrives after retry backoff

### No Signal Test

1. Fully shield the SIM800L in RF foil
2. Trigger an alert
3. Verify transition to BLE broadcast after:
   - Boot delay (4000ms)
   - ATE0 + CMGF init
   - CSQ/CREG probe
   - Signal evaluation (NONE)
   - Modem power cut (GPIO 14 HIGH)
4. Verify `ble_fallback_needed()` is true and BLE burst advertising starts

---

## Adding New Tests

### Directory Convention

Each test suite lives in its own directory under `test/`:

```
test/
├── test_safety/
│   └── test_safety_engine.cpp
├── test_correlator/
│   └── test_correlator.cpp
├── test_anomaly/
│   └── test_anomaly_engine.cpp
├── test_degraded_sos/
│   └── test_degraded_sos.cpp
├── test_ble_relay/
│   └── test_ble_relay.cpp
├── test_sensors/
│   └── test_sensor_reads.cpp
└── test_my_new_module/
    └── test_my_new_module.cpp
```
