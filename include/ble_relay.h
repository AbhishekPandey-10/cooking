#ifndef BLE_RELAY_H
#define BLE_RELAY_H

#include "ble_relay_types.h"
#include <Print.h>

// ============================================================================
//  BLE Relay Engine — Non-blocking scanner + advertiser
//
//  Broadcasting:
//    broadcast_emergency()  → 250 ms burst, non-connectable undirected
//    broadcast_heartbeat()  → 5000 ms idle presence beacon
//    stop_broadcast()       → silence
//
//  Receiving:
//    start_scanner()  → continuous passive BLE scan
//    tick(now_ms)     → process queued RX packets, evaluate relay decisions
//
//  Threading:
//    BLE scan callbacks run in the Bluedroid task (core 0).
//    tick() runs in the Arduino loop (core 1).
//    Cross-core data is guarded by a portMUX spinlock.
// ============================================================================

/// Callback invoked when a valid relay candidate is received.
/// Payload has hop_count set to 1 (marked as relayed).
using RelayCallback = void (*)(const EmergencyPayload &payload);

class BleRelay {
public:
    /// Initialize BLE stack.  Call once at boot.
    /// @param device_id  Unique 16-bit ID for this wearable.
    /// @param name       BLE device name (default "DIS-SOS").
    void begin(uint16_t device_id, const char *name = "DIS-SOS");

    // ---- Broadcasting ------------------------------------------------------

    /// Start emergency advertising (250 ms burst, non-connectable).
    /// Builds the packet, computes CRC, increments sequence_num.
    void broadcast_emergency(uint8_t alert_code, float lat, float lon);

    /// Switch to idle heartbeat advertising (5000 ms interval).
    void broadcast_heartbeat();

    /// Stop all advertising.
    void stop_broadcast();

    // ---- Scanning ----------------------------------------------------------

    /// Start continuous passive BLE scan.
    void start_scanner();

    /// Stop scanning.
    void stop_scanner();

    // ---- Main loop ---------------------------------------------------------

    /// Process queued received packets, evaluate relay decisions.
    /// Call from loop() with injected timestamp.
    void tick(uint32_t now_ms);

    // ---- Configuration -----------------------------------------------------

    void set_relay_callback(RelayCallback cb) { relay_cb_ = cb; }
    void set_debug(Print *dbg)                { debug_ = dbg; }

    // ---- Query -------------------------------------------------------------

    uint8_t         sequence_num()  const { return seq_num_; }
    uint16_t        device_id()     const { return device_id_; }
    bool            is_broadcasting() const { return broadcasting_; }
    bool            is_scanning()   const { return scanning_; }
    const DedupRing &dedup_ring()   const { return dedup_; }

    // ---- Internal (called from BLE scan callback context) ------------------
    void enqueue_rx(const EmergencyPayload &p, int rssi);

private:
    uint16_t      device_id_     = 0;
    uint8_t       seq_num_       = 0;
    bool          broadcasting_  = false;
    bool          scanning_      = false;
    bool          ble_ready_     = false;

    DedupRing     dedup_;
    RelayCallback relay_cb_      = nullptr;
    Print        *debug_         = nullptr;

    // ---- RX queue (BLE task → main loop, spinlock-guarded) -----------------
    static constexpr size_t RX_QUEUE_SIZE = 4;
    struct RxEntry {
        EmergencyPayload payload;
        int              rssi;
        bool             valid;
    };
    RxEntry       rx_queue_[RX_QUEUE_SIZE] = {};
    volatile size_t rx_head_  = 0;
    volatile size_t rx_tail_  = 0;
    volatile size_t rx_count_ = 0;

    // ---- Internal helpers --------------------------------------------------
    void process_received(const EmergencyPayload &p, int rssi);
    void set_adv_data_and_start(const uint8_t *data, size_t len,
                                 uint16_t interval_units);
    void log(const char *fmt, ...);
};

#endif // BLE_RELAY_H
