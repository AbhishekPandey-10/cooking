#ifndef BLE_RELAY_TYPES_H
#define BLE_RELAY_TYPES_H

#include <cstdint>
#include <cstddef>
#include <cstring>

// ============================================================================
//  Phase 10 — Connectionless Single-Hop BLE Advertising Relay Protocol
//
//  Packet format: legacy advertising frame (≤ 31 bytes)
//  ┌─────────┬───────────────────────────────┬────────────────────┐
//  │ AD Flags│ Mfr Specific Data             │ (optional padding) │
//  │ 3 bytes │ 19 bytes                       │ up to 9 bytes      │
//  └─────────┴───────────────────────────────┴────────────────────┘
//  Total used: 22 bytes.  Frame limit: 31 bytes.
// ============================================================================

// ============================================================================
//  Configuration Constants
// ============================================================================
namespace BleRelayConfig {
    constexpr uint16_t COMPANY_ID                = 0xFFFF;   // BT SIG test ID
    constexpr size_t   DEDUP_RING_CAPACITY       = 16;
    constexpr uint8_t  MAX_HOP_COUNT             = 1;        // Single-hop relay

    // Advertising intervals (milliseconds)
    constexpr uint32_t BURST_INTERVAL_MS         = 250;      // Active emergency
    constexpr uint32_t HEARTBEAT_INTERVAL_MS     = 5000;     // Idle presence

    // BLE advertising interval units (1 unit = 0.625 ms)
    constexpr uint16_t BURST_ADV_UNITS           = 400;      // 250 ms / 0.625
    constexpr uint16_t HEARTBEAT_ADV_UNITS       = 8000;     // 5000 ms / 0.625

    // AD structure constants
    constexpr size_t   ADV_FRAME_MAX             = 31;
    constexpr size_t   ADV_DATA_LEN              = 22;       // Flags(3) + Mfr(19)
    constexpr size_t   PAYLOAD_SIZE              = 15;
    constexpr size_t   MFR_AD_LEN                = 18;       // type(1)+cid(2)+payload(15)

    // Fixed-point coordinate multiplier
    constexpr float    COORD_SCALE               = 10000.0f;

    // Alert codes
    constexpr uint8_t  ALERT_NONE                = 0x00;
    constexpr uint8_t  ALERT_FALL                = 0x01;
    constexpr uint8_t  ALERT_CARDIAC             = 0x02;
    constexpr uint8_t  ALERT_HEAT_GAS            = 0x03;
}

// ============================================================================
//  Emergency Payload — 15 bytes, packed, no padding
// ============================================================================
struct __attribute__((packed)) EmergencyPayload {
    uint16_t device_id;      // 2 B — unique per wearable
    uint8_t  alert_code;     // 1 B — 0x01 Fall, 0x02 Cardiac, 0x03 Heat/Gas
    int32_t  trunc_lat;      // 4 B — latitude  × 10^4  (fixed-point)
    int32_t  trunc_lon;      // 4 B — longitude × 10^4  (fixed-point)
    uint8_t  sequence_num;   // 1 B — monotonic counter (wraps at 255)
    uint8_t  hop_count;      // 1 B — 0 = origin, 1 = relayed
    uint16_t crc16;          // 2 B — CRC-16-CCITT-FALSE over preceding 13 bytes
};

static_assert(sizeof(EmergencyPayload) == 15,
              "EmergencyPayload must be exactly 15 bytes packed");

// ============================================================================
//  CRC-16-CCITT-FALSE (poly=0x1021, init=0xFFFF)
// ============================================================================
inline uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

/// Compute CRC over the first 13 bytes (everything before the crc16 field).
inline uint16_t compute_packet_crc(const EmergencyPayload &p)
{
    return crc16_ccitt(reinterpret_cast<const uint8_t *>(&p),
                       offsetof(EmergencyPayload, crc16));
}

/// Validate CRC of a received packet.
inline bool validate_packet_crc(const EmergencyPayload &p)
{
    return p.crc16 == compute_packet_crc(p);
}

// ============================================================================
//  Fixed-Point Coordinate Encoding (4 decimal places ≈ 11 m resolution)
// ============================================================================
inline int32_t encode_coord(float coord)
{
    return static_cast<int32_t>(coord * BleRelayConfig::COORD_SCALE);
}

inline float decode_coord(int32_t fixed)
{
    return static_cast<float>(fixed) / BleRelayConfig::COORD_SCALE;
}

// ============================================================================
//  Relay Decision
// ============================================================================
enum class RelayDecision : uint8_t {
    RELAY,                // Valid origin packet — forward via cellular
    DROP_INVALID_CRC,     // CRC mismatch
    DROP_HOP_EXCEEDED,    // hop_count >= 1 — already relayed
    DROP_DUPLICATE        // (device_id, seq) already in ring buffer
};

// ============================================================================
//  Dedup Ring Buffer — 16-entry FIFO of {device_id, sequence_num}
// ============================================================================
struct DedupEntry {
    uint16_t device_id;
    uint8_t  sequence_num;
};

class DedupRing {
public:
    bool contains(uint16_t device_id, uint8_t seq) const
    {
        for (size_t i = 0; i < count_; i++) {
            size_t idx = (head_ + BleRelayConfig::DEDUP_RING_CAPACITY - 1 - i)
                         % BleRelayConfig::DEDUP_RING_CAPACITY;
            if (entries_[idx].device_id == device_id &&
                entries_[idx].sequence_num == seq) {
                return true;
            }
        }
        return false;
    }

    void insert(uint16_t device_id, uint8_t seq)
    {
        entries_[head_] = { device_id, seq };
        head_ = (head_ + 1) % BleRelayConfig::DEDUP_RING_CAPACITY;
        if (count_ < BleRelayConfig::DEDUP_RING_CAPACITY) count_++;
    }

    void clear()  { head_ = 0; count_ = 0; }
    size_t count() const { return count_; }

private:
    DedupEntry entries_[BleRelayConfig::DEDUP_RING_CAPACITY] = {};
    size_t head_  = 0;
    size_t count_ = 0;
};

// ============================================================================
//  Relay Decision Logic — pure, fully testable
// ============================================================================
inline RelayDecision evaluate_relay(const EmergencyPayload &p, DedupRing &ring)
{
    if (!validate_packet_crc(p))                     return RelayDecision::DROP_INVALID_CRC;
    if (p.hop_count >= BleRelayConfig::MAX_HOP_COUNT) return RelayDecision::DROP_HOP_EXCEEDED;
    if (ring.contains(p.device_id, p.sequence_num))  return RelayDecision::DROP_DUPLICATE;

    ring.insert(p.device_id, p.sequence_num);
    return RelayDecision::RELAY;
}

// ============================================================================
//  Raw Advertising Data Construction / Parsing
// ============================================================================

/// Build the complete 22-byte raw advertising frame.
/// Returns bytes written (22 on success, 0 on failure).
inline size_t build_adv_data(const EmergencyPayload &p, uint8_t *out, size_t cap)
{
    if (cap < BleRelayConfig::ADV_DATA_LEN) return 0;

    // AD Flags: Length=2, Type=0x01, Value=0x06
    out[0] = 0x02;
    out[1] = 0x01;
    out[2] = 0x06;

    // Manufacturer Specific Data: Length=18, Type=0xFF, CID=0xFFFF
    out[3] = BleRelayConfig::MFR_AD_LEN;
    out[4] = 0xFF;
    out[5] = static_cast<uint8_t>(BleRelayConfig::COMPANY_ID & 0xFF);
    out[6] = static_cast<uint8_t>((BleRelayConfig::COMPANY_ID >> 8) & 0xFF);

    // Payload (15 bytes)
    memcpy(&out[7], &p, sizeof(EmergencyPayload));

    return BleRelayConfig::ADV_DATA_LEN;
}

/// Parse raw advertising data, iterating AD structures to find our
/// manufacturer-specific payload.  Returns true if found and extracted.
inline bool parse_adv_data(const uint8_t *data, size_t len, EmergencyPayload *out)
{
    size_t i = 0;
    while (i + 1 < len) {
        uint8_t ad_len = data[i];
        if (ad_len == 0) break;
        if (i + 1 + ad_len > len) break;

        uint8_t ad_type = data[i + 1];
        if (ad_type == 0xFF && ad_len >= 1 + 2 + BleRelayConfig::PAYLOAD_SIZE) {
            uint16_t cid = data[i + 2]
                         | (static_cast<uint16_t>(data[i + 3]) << 8);
            if (cid == BleRelayConfig::COMPANY_ID) {
                memcpy(out, &data[i + 4], sizeof(EmergencyPayload));
                return true;
            }
        }
        i += 1 + ad_len;
    }
    return false;
}

/// Return a human-readable name for a relay decision.
inline const char *relay_decision_name(RelayDecision d)
{
    switch (d) {
    case RelayDecision::RELAY:             return "RELAY";
    case RelayDecision::DROP_INVALID_CRC:  return "DROP_INVALID_CRC";
    case RelayDecision::DROP_HOP_EXCEEDED: return "DROP_HOP_EXCEEDED";
    case RelayDecision::DROP_DUPLICATE:    return "DROP_DUPLICATE";
    default:                               return "???";
    }
}

#endif // BLE_RELAY_TYPES_H
