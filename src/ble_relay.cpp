#include "ble_relay.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLEScan.h>
#include <cstdarg>
#include <cstdio>

// ============================================================================
//  Global instance pointer — required for static BLE scan callback routing
// ============================================================================
static BleRelay *g_ble_relay_instance = nullptr;

// ============================================================================
//  Cross-core spinlock for RX queue
// ============================================================================
static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
//  BLE Scan Callback Handler
// ============================================================================
class RelayScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override
    {
        if (!g_ble_relay_instance) return;
        if (!advertisedDevice.haveManufacturerData()) return;

        std::string mfr = advertisedDevice.getManufacturerData();

        // Minimum: 2 bytes company ID + 15 bytes payload
        if (mfr.length() < 2 + sizeof(EmergencyPayload)) return;

        // Check company ID (little-endian in manufacturer data)
        uint16_t cid = static_cast<uint8_t>(mfr[0])
                     | (static_cast<uint16_t>(static_cast<uint8_t>(mfr[1])) << 8);
        if (cid != BleRelayConfig::COMPANY_ID) return;

        // Extract payload
        EmergencyPayload payload;
        memcpy(&payload, mfr.data() + 2, sizeof(EmergencyPayload));

        g_ble_relay_instance->enqueue_rx(payload, advertisedDevice.getRSSI());
    }
};

static RelayScanCallbacks s_scan_callbacks;

// ============================================================================
//  BleRelay — Implementation
// ============================================================================

void BleRelay::begin(uint16_t device_id, const char *name)
{
    device_id_ = device_id;
    g_ble_relay_instance = this;

    BLEDevice::init(name);
    ble_ready_ = true;

    log("[BLE] Initialized as \"%s\" (devid=0x%04X)\n", name, device_id);
}

// ============================================================================
//  Broadcasting
// ============================================================================

void BleRelay::broadcast_emergency(uint8_t alert_code, float lat, float lon)
{
    if (!ble_ready_) return;

    // ---- Build payload -----------------------------------------------------
    EmergencyPayload p;
    memset(&p, 0, sizeof(p));
    p.device_id    = device_id_;
    p.alert_code   = alert_code;
    p.trunc_lat    = encode_coord(lat);
    p.trunc_lon    = encode_coord(lon);
    p.sequence_num = seq_num_++;
    p.hop_count    = 0;  // origin
    p.crc16        = compute_packet_crc(p);

    // ---- Build raw advertising frame ---------------------------------------
    uint8_t adv[BleRelayConfig::ADV_FRAME_MAX];
    size_t len = build_adv_data(p, adv, sizeof(adv));

    log("[BLE] Emergency broadcast: alert=0x%02X lat=%ld lon=%ld seq=%d hop=0 crc=0x%04X\n",
        alert_code, (long)p.trunc_lat, (long)p.trunc_lon,
        p.sequence_num, p.crc16);

    set_adv_data_and_start(adv, len, BleRelayConfig::BURST_ADV_UNITS);
    broadcasting_ = true;

    log("[BLE] ADV started (250ms burst, non-connectable)\n");
}

void BleRelay::broadcast_heartbeat()
{
    if (!ble_ready_) return;

    // Build a presence beacon (alert_code = 0x00)
    EmergencyPayload p;
    memset(&p, 0, sizeof(p));
    p.device_id    = device_id_;
    p.alert_code   = BleRelayConfig::ALERT_NONE;
    p.sequence_num = seq_num_++;
    p.hop_count    = 0;
    p.crc16        = compute_packet_crc(p);

    uint8_t adv[BleRelayConfig::ADV_FRAME_MAX];
    size_t len = build_adv_data(p, adv, sizeof(adv));

    set_adv_data_and_start(adv, len, BleRelayConfig::HEARTBEAT_ADV_UNITS);
    broadcasting_ = true;

    log("[BLE] Heartbeat ADV started (5000ms interval)\n");
}

void BleRelay::stop_broadcast()
{
    if (!ble_ready_ || !broadcasting_) return;

    BLEDevice::getAdvertising()->stop();
    broadcasting_ = false;
    log("[BLE] ADV stopped\n");
}

// ============================================================================
//  Scanning
// ============================================================================

void BleRelay::start_scanner()
{
    if (!ble_ready_) return;

    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&s_scan_callbacks, true); // true = duplicates
    scan->setActiveScan(false);   // Passive scan (no scan request)
    scan->setInterval(100);       // Scan interval (in 0.625ms units)
    scan->setWindow(99);          // Scan window (nearly continuous)
    scan->start(0, nullptr, false); // Scan indefinitely
    scanning_ = true;

    log("[BLE] Scanner active (passive, continuous)\n");
}

void BleRelay::stop_scanner()
{
    if (!ble_ready_ || !scanning_) return;

    BLEDevice::getScan()->stop();
    scanning_ = false;
    log("[BLE] Scanner stopped\n");
}

// ============================================================================
//  tick() — Main loop processing (runs on Arduino core)
// ============================================================================

void BleRelay::tick(uint32_t /* now_ms */)
{
    // Drain the RX queue
    while (true) {
        RxEntry entry;
        bool got = false;

        portENTER_CRITICAL(&s_rx_mux);
        if (rx_count_ > 0) {
            entry = rx_queue_[rx_head_];
            rx_head_ = (rx_head_ + 1) % RX_QUEUE_SIZE;
            rx_count_--;
            got = true;
        }
        portEXIT_CRITICAL(&s_rx_mux);

        if (!got) break;

        process_received(entry.payload, entry.rssi);
    }
}

// ============================================================================
//  enqueue_rx — Called from BLE scan callback (Bluedroid task, core 0)
// ============================================================================

void BleRelay::enqueue_rx(const EmergencyPayload &p, int rssi)
{
    portENTER_CRITICAL(&s_rx_mux);
    if (rx_count_ < RX_QUEUE_SIZE) {
        size_t idx = (rx_head_ + rx_count_) % RX_QUEUE_SIZE;
        rx_queue_[idx].payload = p;
        rx_queue_[idx].rssi    = rssi;
        rx_queue_[idx].valid   = true;
        rx_count_++;
    }
    // If queue full, packet is dropped (acceptable: dedup handles retries)
    portEXIT_CRITICAL(&s_rx_mux);
}

// ============================================================================
//  process_received — Evaluate relay decision on main loop
// ============================================================================

void BleRelay::process_received(const EmergencyPayload &p, int rssi)
{
    log("[BLE] RX: devid=0x%04X alert=0x%02X lat=%ld lon=%ld seq=%d hop=%d "
        "crc=0x%04X RSSI=%d\n",
        p.device_id, p.alert_code,
        (long)p.trunc_lat, (long)p.trunc_lon,
        p.sequence_num, p.hop_count, p.crc16, rssi);

    RelayDecision decision = evaluate_relay(p, dedup_);

    log("[BLE] Decision: %s\n", relay_decision_name(decision));

    if (decision == RelayDecision::RELAY) {
        // Mark as relayed and forward to callback
        EmergencyPayload relayed = p;
        relayed.hop_count = 1;

        log("[BLE] -> Relay callback: alert=0x%02X lat=%.4f lon=%.4f\n",
            relayed.alert_code,
            decode_coord(relayed.trunc_lat),
            decode_coord(relayed.trunc_lon));

        if (relay_cb_) {
            relay_cb_(relayed);
        }
    }
}

// ============================================================================
//  Internal: Set raw advertising data and start
// ============================================================================

void BleRelay::set_adv_data_and_start(const uint8_t *data, size_t len,
                                       uint16_t interval_units)
{
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->stop();

    // Use the ESP-IDF API directly for raw advertising data
    // (available through the Arduino framework on ESP32-S3)
    esp_ble_gap_config_adv_data_raw(const_cast<uint8_t *>(data),
                                     static_cast<uint32_t>(len));

    // Configure advertising parameters
    esp_ble_adv_params_t adv_params = {};
    adv_params.adv_int_min       = interval_units;
    adv_params.adv_int_max       = interval_units;
    adv_params.adv_type          = ADV_TYPE_NONCONN_IND;
    adv_params.own_addr_type     = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map       = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    esp_ble_gap_start_advertising(&adv_params);
}

// ============================================================================
//  Debug Logging
// ============================================================================

void BleRelay::log(const char *fmt, ...)
{
    if (!debug_) return;
    char buf[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    debug_->print(buf);
}
