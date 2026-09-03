#include <unity.h>
#include <cstring>
#include <cstdio>
#include <cmath>

// Only include the pure-logic types header — no BLE hardware dependency
#include "ble_relay_types.h"

// ############################################################################
//
//  Unit Tests — Phase 10 BLE Relay Protocol
//
//  These tests verify bit-packing, CRC integrity, fixed-point encoding,
//  AD frame construction/parsing, ring-buffer dedup, and single-hop drop
//  logic.  All tests are pure C++ — no BLE stack required.
//
// ############################################################################

// ---- Helper: build a valid origin packet -----------------------------------
static EmergencyPayload make_valid_packet(uint16_t devid = 0xA1B2,
                                           uint8_t  alert = 0x01,
                                           float    lat   = -33.856784f,
                                           float    lon   = 151.215297f,
                                           uint8_t  seq   = 42)
{
    EmergencyPayload p;
    memset(&p, 0, sizeof(p));
    p.device_id    = devid;
    p.alert_code   = alert;
    p.trunc_lat    = encode_coord(lat);
    p.trunc_lon    = encode_coord(lon);
    p.sequence_num = seq;
    p.hop_count    = 0;
    p.crc16        = compute_packet_crc(p);
    return p;
}

// ############################################################################
//  1. Packed Struct Layout
// ############################################################################

void test_payload_size()
{
    TEST_ASSERT_EQUAL(15, sizeof(EmergencyPayload));
}

void test_payload_field_offsets()
{
    TEST_ASSERT_EQUAL(0,  offsetof(EmergencyPayload, device_id));
    TEST_ASSERT_EQUAL(2,  offsetof(EmergencyPayload, alert_code));
    TEST_ASSERT_EQUAL(3,  offsetof(EmergencyPayload, trunc_lat));
    TEST_ASSERT_EQUAL(7,  offsetof(EmergencyPayload, trunc_lon));
    TEST_ASSERT_EQUAL(11, offsetof(EmergencyPayload, sequence_num));
    TEST_ASSERT_EQUAL(12, offsetof(EmergencyPayload, hop_count));
    TEST_ASSERT_EQUAL(13, offsetof(EmergencyPayload, crc16));
}

// ############################################################################
//  2. CRC-16-CCITT-FALSE
// ############################################################################

void test_crc16_known_vector()
{
    // Standard test vector: "123456789" → 0x29B1
    const uint8_t data[] = "123456789";
    uint16_t crc = crc16_ccitt(data, 9);
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);
}

void test_crc16_empty()
{
    uint16_t crc = crc16_ccitt(nullptr, 0);
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc);  // init value, no data
}

void test_crc16_single_byte()
{
    uint8_t data[] = { 0x00 };
    uint16_t crc = crc16_ccitt(data, 1);
    TEST_ASSERT_TRUE(crc != 0xFFFF);  // Should differ from init
}

void test_packet_crc_round_trip()
{
    EmergencyPayload p = make_valid_packet();
    TEST_ASSERT_TRUE(validate_packet_crc(p));
}

void test_packet_crc_detects_corruption()
{
    EmergencyPayload p = make_valid_packet();

    // Corrupt one byte
    p.alert_code = 0xFF;

    TEST_ASSERT_FALSE(validate_packet_crc(p));
}

void test_packet_crc_detects_seq_change()
{
    EmergencyPayload p = make_valid_packet();

    // Change sequence without recomputing CRC
    p.sequence_num++;

    TEST_ASSERT_FALSE(validate_packet_crc(p));
}

// ############################################################################
//  3. Fixed-Point Coordinate Encoding
// ############################################################################

void test_encode_positive_coord()
{
    int32_t enc = encode_coord(151.215297f);
    // 151.215297 × 10000 = 1512152.97 → 1512152 or 1512153
    TEST_ASSERT_INT32_WITHIN(2, 1512153, enc);
}

void test_encode_negative_coord()
{
    int32_t enc = encode_coord(-33.856784f);
    // -33.856784 × 10000 = -338567.84 → -338567 or -338568
    TEST_ASSERT_INT32_WITHIN(2, -338568, enc);
}

void test_decode_round_trip()
{
    float original = -33.8568f;
    int32_t enc = encode_coord(original);
    float dec = decode_coord(enc);
    TEST_ASSERT_FLOAT_WITHIN(0.0002f, original, dec);
}

void test_encode_zero()
{
    TEST_ASSERT_EQUAL(0, encode_coord(0.0f));
}

void test_decode_precision()
{
    // 4 decimal places → resolution ≈ 11m at equator
    int32_t enc = encode_coord(1.23456789f);
    float dec = decode_coord(enc);
    // Should match to 4dp
    TEST_ASSERT_FLOAT_WITHIN(0.00015f, 1.2346f, dec);
}

// ############################################################################
//  4. Raw Advertising Frame Construction
// ############################################################################

void test_build_adv_data_length()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t buf[31];
    size_t len = build_adv_data(p, buf, sizeof(buf));

    TEST_ASSERT_EQUAL(22, len);
}

void test_build_adv_data_flags()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t buf[31];
    build_adv_data(p, buf, sizeof(buf));

    TEST_ASSERT_EQUAL_HEX8(0x02, buf[0]);  // Flags AD length
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[1]);  // AD type: Flags
    TEST_ASSERT_EQUAL_HEX8(0x06, buf[2]);  // LE Gen Disc + BR/EDR Not Supported
}

void test_build_adv_data_manufacturer_header()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t buf[31];
    build_adv_data(p, buf, sizeof(buf));

    TEST_ASSERT_EQUAL_HEX8(0x12, buf[3]);  // Mfr AD length (18)
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[4]);  // AD type: Manufacturer Specific
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[5]);  // Company ID low
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[6]);  // Company ID high
}

void test_build_adv_data_payload_bytes()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t buf[31];
    build_adv_data(p, buf, sizeof(buf));

    // Verify payload is at offset 7
    EmergencyPayload extracted;
    memcpy(&extracted, &buf[7], sizeof(EmergencyPayload));

    TEST_ASSERT_EQUAL_HEX16(p.device_id, extracted.device_id);
    TEST_ASSERT_EQUAL(p.alert_code, extracted.alert_code);
    TEST_ASSERT_EQUAL(p.trunc_lat, extracted.trunc_lat);
    TEST_ASSERT_EQUAL(p.trunc_lon, extracted.trunc_lon);
    TEST_ASSERT_EQUAL(p.sequence_num, extracted.sequence_num);
    TEST_ASSERT_EQUAL(p.hop_count, extracted.hop_count);
    TEST_ASSERT_EQUAL_HEX16(p.crc16, extracted.crc16);
}

void test_build_adv_data_too_small_buffer()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t buf[10];  // Too small
    size_t len = build_adv_data(p, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(0, len);
}

// ############################################################################
//  5. Raw Advertising Frame Parsing
// ############################################################################

void test_parse_adv_data_round_trip()
{
    EmergencyPayload original = make_valid_packet();
    uint8_t frame[31];
    build_adv_data(original, frame, sizeof(frame));

    EmergencyPayload parsed;
    bool ok = parse_adv_data(frame, 22, &parsed);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_HEX16(original.device_id, parsed.device_id);
    TEST_ASSERT_EQUAL(original.alert_code, parsed.alert_code);
    TEST_ASSERT_EQUAL(original.trunc_lat, parsed.trunc_lat);
    TEST_ASSERT_EQUAL(original.trunc_lon, parsed.trunc_lon);
    TEST_ASSERT_EQUAL(original.sequence_num, parsed.sequence_num);
    TEST_ASSERT_EQUAL(original.hop_count, parsed.hop_count);
    TEST_ASSERT_EQUAL_HEX16(original.crc16, parsed.crc16);
}

void test_parse_adv_data_wrong_company_id()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t frame[31];
    build_adv_data(p, frame, sizeof(frame));

    // Corrupt company ID
    frame[5] = 0x00;  // Change CID low byte

    EmergencyPayload parsed;
    bool ok = parse_adv_data(frame, 22, &parsed);
    TEST_ASSERT_FALSE(ok);
}

void test_parse_adv_data_truncated()
{
    EmergencyPayload p = make_valid_packet();
    uint8_t frame[31];
    build_adv_data(p, frame, sizeof(frame));

    EmergencyPayload parsed;
    bool ok = parse_adv_data(frame, 10, &parsed);  // Not enough data
    TEST_ASSERT_FALSE(ok);
}

void test_parse_adv_data_garbage_prefix()
{
    // Put a random AD structure before the manufacturer data
    uint8_t frame[31];
    // Random AD: length=3, type=0x09, data="AB"
    frame[0] = 0x03;
    frame[1] = 0x09;
    frame[2] = 'A';
    frame[3] = 'B';

    // Now add manufacturer data at offset 4
    EmergencyPayload p = make_valid_packet();
    frame[4]  = 0x12;  // length = 18
    frame[5]  = 0xFF;  // type
    frame[6]  = 0xFF;  // CID low
    frame[7]  = 0xFF;  // CID high
    memcpy(&frame[8], &p, sizeof(p));

    EmergencyPayload parsed;
    bool ok = parse_adv_data(frame, 23, &parsed);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_HEX16(p.device_id, parsed.device_id);
}

// ############################################################################
//  6. Dedup Ring Buffer
// ############################################################################

void test_dedup_empty_not_found()
{
    DedupRing ring;
    TEST_ASSERT_FALSE(ring.contains(0xA1B2, 42));
    TEST_ASSERT_EQUAL(0, ring.count());
}

void test_dedup_insert_and_find()
{
    DedupRing ring;
    ring.insert(0xA1B2, 42);
    TEST_ASSERT_TRUE(ring.contains(0xA1B2, 42));
    TEST_ASSERT_EQUAL(1, ring.count());
}

void test_dedup_different_seq_not_found()
{
    DedupRing ring;
    ring.insert(0xA1B2, 42);
    TEST_ASSERT_FALSE(ring.contains(0xA1B2, 43));  // Different seq
    TEST_ASSERT_FALSE(ring.contains(0xA1B3, 42));  // Different device
}

void test_dedup_multiple_entries()
{
    DedupRing ring;
    ring.insert(0x0001, 0);
    ring.insert(0x0002, 1);
    ring.insert(0x0003, 2);

    TEST_ASSERT_TRUE(ring.contains(0x0001, 0));
    TEST_ASSERT_TRUE(ring.contains(0x0002, 1));
    TEST_ASSERT_TRUE(ring.contains(0x0003, 2));
    TEST_ASSERT_EQUAL(3, ring.count());
}

void test_dedup_wraps_at_capacity()
{
    DedupRing ring;

    // Fill to capacity (16 entries)
    for (uint8_t i = 0; i < 16; i++) {
        ring.insert(0x0001, i);
    }
    TEST_ASSERT_EQUAL(16, ring.count());

    // Oldest entry (seq=0) should still be findable
    TEST_ASSERT_TRUE(ring.contains(0x0001, 0));

    // Insert one more — should evict the oldest (seq=0)
    ring.insert(0x0001, 16);
    TEST_ASSERT_EQUAL(16, ring.count());

    // seq=0 is now evicted
    TEST_ASSERT_FALSE(ring.contains(0x0001, 0));

    // seq=1 through seq=16 should be present
    TEST_ASSERT_TRUE(ring.contains(0x0001, 1));
    TEST_ASSERT_TRUE(ring.contains(0x0001, 16));
}

void test_dedup_clear()
{
    DedupRing ring;
    ring.insert(0xA1B2, 42);
    ring.clear();
    TEST_ASSERT_FALSE(ring.contains(0xA1B2, 42));
    TEST_ASSERT_EQUAL(0, ring.count());
}

// ############################################################################
//  7. Relay Decision Logic — Core Single-Hop Protocol
// ############################################################################

void test_relay_valid_origin_packet()
{
    DedupRing ring;
    EmergencyPayload p = make_valid_packet(0xA1B2, 0x01, -33.8f, 151.2f, 0);

    RelayDecision d = evaluate_relay(p, ring);

    TEST_ASSERT_TRUE(d == RelayDecision::RELAY);
    TEST_ASSERT_TRUE(ring.contains(0xA1B2, 0));
}

void test_relay_drop_invalid_crc()
{
    DedupRing ring;
    EmergencyPayload p = make_valid_packet();
    p.crc16 ^= 0xBEEF;  // Corrupt CRC

    RelayDecision d = evaluate_relay(p, ring);
    TEST_ASSERT_TRUE(d == RelayDecision::DROP_INVALID_CRC);
    TEST_ASSERT_EQUAL(0, ring.count());  // Not inserted
}

void test_relay_drop_hop_exceeded()
{
    DedupRing ring;
    EmergencyPayload p = make_valid_packet();
    p.hop_count = 1;                      // Already relayed
    p.crc16 = compute_packet_crc(p);      // Fix CRC for modified packet

    RelayDecision d = evaluate_relay(p, ring);
    TEST_ASSERT_TRUE(d == RelayDecision::DROP_HOP_EXCEEDED);
    TEST_ASSERT_EQUAL(0, ring.count());
}

void test_relay_drop_duplicate()
{
    DedupRing ring;
    EmergencyPayload p = make_valid_packet(0xA1B2, 0x01, -33.8f, 151.2f, 42);

    // First time: RELAY
    RelayDecision d1 = evaluate_relay(p, ring);
    TEST_ASSERT_TRUE(d1 == RelayDecision::RELAY);

    // Second time (same device_id + seq): DROP_DUPLICATE
    RelayDecision d2 = evaluate_relay(p, ring);
    TEST_ASSERT_TRUE(d2 == RelayDecision::DROP_DUPLICATE);
}

void test_relay_different_seq_not_duplicate()
{
    DedupRing ring;
    EmergencyPayload p1 = make_valid_packet(0xA1B2, 0x01, -33.8f, 151.2f, 0);
    EmergencyPayload p2 = make_valid_packet(0xA1B2, 0x01, -33.8f, 151.2f, 1);

    TEST_ASSERT_TRUE(evaluate_relay(p1, ring) == RelayDecision::RELAY);
    TEST_ASSERT_TRUE(evaluate_relay(p2, ring) == RelayDecision::RELAY);
}

void test_relay_different_device_not_duplicate()
{
    DedupRing ring;
    EmergencyPayload p1 = make_valid_packet(0x0001, 0x01, -33.8f, 151.2f, 42);
    EmergencyPayload p2 = make_valid_packet(0x0002, 0x01, -33.8f, 151.2f, 42);

    TEST_ASSERT_TRUE(evaluate_relay(p1, ring) == RelayDecision::RELAY);
    TEST_ASSERT_TRUE(evaluate_relay(p2, ring) == RelayDecision::RELAY);
}

void test_relay_priority_order_crc_first()
{
    // CRC check happens BEFORE hop check → corrupt relayed packet = DROP_INVALID_CRC
    DedupRing ring;
    EmergencyPayload p = make_valid_packet();
    p.hop_count = 1;
    // Don't fix CRC → should fail CRC, not hop check

    RelayDecision d = evaluate_relay(p, ring);
    TEST_ASSERT_TRUE(d == RelayDecision::DROP_INVALID_CRC);
}

// ############################################################################
//  8. End-to-End Byte-Level Verification
// ############################################################################

void test_e2e_serialize_parse_validate_relay()
{
    // ---- Origin device builds packet ----
    EmergencyPayload origin;
    memset(&origin, 0, sizeof(origin));
    origin.device_id    = 0xCAFE;
    origin.alert_code   = 0x02;   // Cardiac
    origin.trunc_lat    = encode_coord(28.6139f);  // Delhi
    origin.trunc_lon    = encode_coord(77.2090f);
    origin.sequence_num = 7;
    origin.hop_count    = 0;
    origin.crc16        = compute_packet_crc(origin);

    // ---- Serialize to raw advertising frame ----
    uint8_t frame[31];
    size_t len = build_adv_data(origin, frame, sizeof(frame));
    TEST_ASSERT_EQUAL(22, len);
    TEST_ASSERT_TRUE(len <= 31);  // Strict BLE limit

    // ---- Relay device parses from raw bytes ----
    EmergencyPayload received;
    bool ok = parse_adv_data(frame, len, &received);
    TEST_ASSERT_TRUE(ok);

    // ---- Validate CRC ----
    TEST_ASSERT_TRUE(validate_packet_crc(received));

    // ---- Evaluate relay decision ----
    DedupRing ring;
    RelayDecision d = evaluate_relay(received, ring);
    TEST_ASSERT_TRUE(d == RelayDecision::RELAY);

    // ---- Decode coordinates ----
    float dec_lat = decode_coord(received.trunc_lat);
    float dec_lon = decode_coord(received.trunc_lon);
    TEST_ASSERT_FLOAT_WITHIN(0.0002f, 28.6139f, dec_lat);
    TEST_ASSERT_FLOAT_WITHIN(0.0002f, 77.2090f, dec_lon);

    // ---- Retry → duplicate drop ----
    RelayDecision d2 = evaluate_relay(received, ring);
    TEST_ASSERT_TRUE(d2 == RelayDecision::DROP_DUPLICATE);
}

// ############################################################################
//  Test Runner
// ############################################################################
void setup()
{
    delay(2000);
    UNITY_BEGIN();

    // Packed struct layout
    RUN_TEST(test_payload_size);
    RUN_TEST(test_payload_field_offsets);

    // CRC-16-CCITT
    RUN_TEST(test_crc16_known_vector);
    RUN_TEST(test_crc16_empty);
    RUN_TEST(test_crc16_single_byte);
    RUN_TEST(test_packet_crc_round_trip);
    RUN_TEST(test_packet_crc_detects_corruption);
    RUN_TEST(test_packet_crc_detects_seq_change);

    // Fixed-point encoding
    RUN_TEST(test_encode_positive_coord);
    RUN_TEST(test_encode_negative_coord);
    RUN_TEST(test_decode_round_trip);
    RUN_TEST(test_encode_zero);
    RUN_TEST(test_decode_precision);

    // AD frame construction
    RUN_TEST(test_build_adv_data_length);
    RUN_TEST(test_build_adv_data_flags);
    RUN_TEST(test_build_adv_data_manufacturer_header);
    RUN_TEST(test_build_adv_data_payload_bytes);
    RUN_TEST(test_build_adv_data_too_small_buffer);

    // AD frame parsing
    RUN_TEST(test_parse_adv_data_round_trip);
    RUN_TEST(test_parse_adv_data_wrong_company_id);
    RUN_TEST(test_parse_adv_data_truncated);
    RUN_TEST(test_parse_adv_data_garbage_prefix);

    // Dedup ring buffer
    RUN_TEST(test_dedup_empty_not_found);
    RUN_TEST(test_dedup_insert_and_find);
    RUN_TEST(test_dedup_different_seq_not_found);
    RUN_TEST(test_dedup_multiple_entries);
    RUN_TEST(test_dedup_wraps_at_capacity);
    RUN_TEST(test_dedup_clear);

    // Relay decision logic
    RUN_TEST(test_relay_valid_origin_packet);
    RUN_TEST(test_relay_drop_invalid_crc);
    RUN_TEST(test_relay_drop_hop_exceeded);
    RUN_TEST(test_relay_drop_duplicate);
    RUN_TEST(test_relay_different_seq_not_duplicate);
    RUN_TEST(test_relay_different_device_not_duplicate);
    RUN_TEST(test_relay_priority_order_crc_first);

    // End-to-end
    RUN_TEST(test_e2e_serialize_parse_validate_relay);

    UNITY_END();
}

void loop() {}

// ############################################################################
//
//  TEST 4 VERIFICATION PROTOCOL — Dual-Device Bench Test Walkthrough
//
//  ═══════════════════════════════════════════════════════════════════════
//   EQUIPMENT
//  ═══════════════════════════════════════════════════════════════════════
//
//  - Unit A: ESP32-S3 DevKit + SIM800L.
//            Wrap SIM800L antenna AND SIM in RF-absorbing foil to
//            guarantee cellular failure.  Leave BLE antenna exposed.
//
//  - Unit B: ESP32-S3 DevKit + SIM800L + active SIM card.
//            Cellular connected.  BLE antenna exposed.
//
//  - 2× USB cables, 2× serial monitors (115200 baud).
//
//  ═══════════════════════════════════════════════════════════════════════
//   FIRMWARE CONFIGURATION
//  ═══════════════════════════════════════════════════════════════════════
//
//  Unit A (main.cpp):
//    BleRelay bleRelay;
//    DegradedSos sos(modemSerial);
//
//    void setup() {
//        bleRelay.begin(0xA1B2);
//        bleRelay.set_debug(&Serial);
//        sos.begin(0);
//        sos.set_debug(&Serial);
//    }
//
//    // When SafetyEngine triggers a fall alert:
//    //   1. sos.request_sos(lat, lon, 0x01, "FALL", timestamp);
//    //   2. In loop, tick both: sos.tick(millis()); bleRelay.tick(millis());
//    //   3. When sos.ble_fallback_needed():
//    //        bleRelay.broadcast_emergency(0x01, lat, lon);
//
//  Unit B (main.cpp):
//    BleRelay bleRelay;
//    DegradedSos sos(modemSerial);
//
//    void relay_handler(const EmergencyPayload &p) {
//        float lat = decode_coord(p.trunc_lat);
//        float lon = decode_coord(p.trunc_lon);
//        char ts[24]; snprintf(ts, 24, "RELAY-T%lu", millis());
//        sos.request_sos(lat, lon, p.alert_code, "BLE-RELAY", ts);
//    }
//
//    void setup() {
//        bleRelay.begin(0xB2C3);
//        bleRelay.set_debug(&Serial);
//        bleRelay.set_relay_callback(relay_handler);
//        bleRelay.start_scanner();
//        sos.begin(0);
//        sos.set_debug(&Serial);
//    }
//
//    void loop() {
//        uint32_t t = millis();
//        bleRelay.tick(t);
//        sos.tick(t);
//    }
//
//  ═══════════════════════════════════════════════════════════════════════
//   PROCEDURE
//  ═══════════════════════════════════════════════════════════════════════
//
//  1. Power both units.  Verify "BLE Initialized" on both serial monitors.
//  2. On Unit B, verify "[BLE] Scanner active (passive, continuous)".
//  3. On Unit A, trigger a fall alert (drop/shake the IMU, or inject via
//     serial command).
//  4. Observe Unit A serial: SOS cellular attempt → timeout (RF foil) →
//     BLE fallback → emergency broadcast.
//  5. Observe Unit B serial: BLE frame detected → CRC verified →
//     relay decision → SMS sent.
//  6. Wait 250ms for Unit A's next broadcast → Unit B drops as duplicate.
//  7. (Optional) Remove RF foil from Unit A and verify cellular recovery.
//
//  ═══════════════════════════════════════════════════════════════════════
//   EXPECTED SERIAL OUTPUT
//  ═══════════════════════════════════════════════════════════════════════
//
//  ---- UNIT A (Shielded Broadcaster) ----
//
//  [SOS] IDLE -> POWERING_ON
//  [SOS] Modem power ON
//  [SOS] POWERING_ON -> INIT_ECHO_OFF
//  [SOS] INIT_ECHO_OFF -> INIT_TEXT_MODE
//  [SOS] INIT_TEXT_MODE -> PROBE_CSQ
//  [SOS] CSQ = 0
//  [SOS] PROBE_CSQ -> PROBE_CREG
//  [SOS] CREG stat = 0
//  [SOS] PROBE_CREG -> EVALUATE_SIGNAL
//  [SOS] Signal: NONE (CSQ=0, CREG=0)
//  [SOS] No signal — triggering BLE fallback
//  [SOS] EVALUATE_SIGNAL -> POWERING_OFF
//  [SOS] Modem power OFF
//  [BLE] Emergency broadcast: alert=0x01 lat=-338568 lon=1512153 seq=0 hop=0 crc=0x1A2B
//  [BLE] ADV started (250ms burst, non-connectable)
//
//  ---- UNIT B (Connected Relay) ----
//
//  [BLE] Scanner active (passive, continuous)
//  ...
//  [BLE] RX: devid=0xA1B2 alert=0x01 lat=-338568 lon=1512153 seq=0 hop=0 crc=0x1A2B RSSI=-52
//  [BLE] Decision: RELAY
//  [BLE] -> Relay callback: alert=0x01, lat=-33.8568, lon=151.2153
//  [SOS] IDLE -> POWERING_ON
//  [SOS] Modem power ON
//  [SOS] POWERING_ON -> INIT_ECHO_OFF
//  [SOS] INIT_ECHO_OFF -> INIT_TEXT_MODE
//  [SOS] INIT_TEXT_MODE -> PROBE_CSQ
//  [SOS] CSQ = 18
//  [SOS] PROBE_CSQ -> PROBE_CREG
//  [SOS] CREG stat = 1
//  [SOS] Signal: GOOD (CSQ=18, CREG=1)
//  [SOS] EVALUATE_SIGNAL -> SMS_CMD
//  [SOS] SMS body sent (79 bytes), awaiting result
//  [SOS] *** SMS SENT SUCCESSFULLY ***
//  ...
//  [BLE] RX: devid=0xA1B2 alert=0x01 lat=-338568 lon=1512153 seq=0 hop=0 crc=0x1A2B RSSI=-54
//  [BLE] Decision: DROP_DUPLICATE
//  ...
//  [BLE] RX: devid=0xA1B2 alert=0x01 lat=-338568 lon=1512153 seq=0 hop=0 crc=0x1A2B RSSI=-58
//  [BLE] Decision: DROP_DUPLICATE
//
// ############################################################################
