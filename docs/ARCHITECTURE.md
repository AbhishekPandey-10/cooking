# System Architecture & Concurrency Model

> Technical reference for developers working on the Disaster Health Companion firmware.

---

## Table of Contents

- [Execution Model](#execution-model)
- [Core Assignment](#core-assignment)
- [Cross-Core Data Sharing](#cross-core-data-sharing)
- [Timing Invariants](#timing-invariants)
- [I2C Bus Arbitration](#i2c-bus-arbitration)
- [SRAM Allocation Map](#sram-allocation-map)
- [SOS Escalation State Machine](#sos-escalation-state-machine)
- [BLE Relay Protocol](#ble-relay-protocol)
- [Anomaly Engine Lifecycle](#anomaly-engine-lifecycle)

---

## Execution Model

The firmware uses a **cooperative single-threaded model** on Core 1 (the Arduino core), with hardware timer ISRs and BLE stack callbacks as the only concurrent execution contexts.

```
                    ┌─────────────────────────────────────────┐
                    │         Core 1 — Arduino loop()         │
                    │                                         │
                    │  ┌─ scheduler_poll()                    │
                    │  │    Checks ISR flags, reads sensors   │
                    │  │    via I2C in main-loop context      │
                    │  │                                      │
                    │  ├─ feed_sensors_to_engines()            │
                    │  │    Routes data to SafetyEngine,      │
                    │  │    FeatureExtractor                  │
                    │  │                                      │
                    │  ├─ run_eval_tick() [1 Hz]              │
                    │  │    AnomalyEngine + Correlator        │
                    │  │                                      │
                    │  ├─ orchestrate_comms()                  │
                    │  │    DegradedSos::tick()               │
                    │  │    BleRelay::tick()                  │
                    │  │                                      │
                    │  └─ update_display() [2 Hz]             │
                    │       OLED via I2C                      │
                    └─────────────────────────────────────────┘

                    ┌─────────────────────────────────────────┐
                    │    Core 0 — Bluedroid BLE Task          │
                    │                                         │
                    │  BLE scan callback → enqueue_rx()       │
                    │  (portMUX spinlock, ~24 byte copy)      │
                    └─────────────────────────────────────────┘

                    ┌─────────────────────────────────────────┐
                    │    ISR Context (Hardware Timers)        │
                    │                                         │
                    │  Timer 0 ISR → s_flag_ppg = true (80Hz) │
                    │  Timer 1 ISR → s_flag_imu = true (100Hz)│
                    └─────────────────────────────────────────┘
```

### Concurrency Design Rationale

The analysis pipeline is intentionally **not** split into separate preemptive FreeRTOS tasks:

1. **Shared State Access**: All analysis modules operate on incoming sensor frames. A single-threaded pipeline on Core 1 eliminates mutex contention and lock inversion risks.
2. **Predictable Timing**: The worst-case loop execution time is ~1.2 ms (during a 2 Hz OLED frame update), well below the 10.0 ms deadline imposed by 100 Hz IMU sampling.
3. **Deterministic Testing**: Every state machine accepts an injected `uint32_t now_ms` timestamp, allowing 100% reproducible unit and regression testing without task scheduling jitter.
4. **Memory Footprint**: Eliminates multiple per-task FreeRTOS thread stacks (typically 4 KB each), conserving internal SRAM.

The Bluedroid BLE stack operates on Core 0 as managed by ESP-IDF; communication across cores is confined to a single spinlock-protected FIFO.

---

## Core Assignment

| Context | Execution Target | Priority | Responsibility |
|---|---|---|---|
| `loop()` | Core 1 | 1 (Arduino default) | Sensor feeds, safety engine, anomaly evaluation, modem AT FSM, OLED |
| Timer 0 ISR | Xtensa ISR | High (Hardware) | Sets `s_flag_ppg` at 80 Hz |
| Timer 1 ISR | Xtensa ISR | High (Hardware) | Sets `s_flag_imu` at 100 Hz |
| BLE scan callback | Core 0 | Bluedroid Task | Parses 0xFFFF manufacturer data, enqueues to `rx_queue_` |
| WiFi / BT Stack | Core 0 | System background | Physical BLE radio driver and GAP events |

---

## Cross-Core Data Sharing

| Shared Resource | Size | Writer | Reader | Synchronization Mechanism |
|---|---|---|---|---|
| `s_flag_ppg` | 1 B | Timer 0 ISR | `scheduler_poll()` (Core 1) | `volatile bool` (atomic single-byte access) |
| `s_flag_imu` | 1 B | Timer 1 ISR | `scheduler_poll()` (Core 1) | `volatile bool` (atomic single-byte access) |
| `BleRelay::rx_queue_[4]` | 96 B | BLE scan callback (Core 0) | `BleRelay::tick()` (Core 1) | `portMUX_TYPE` spinlock (`taskENTER_CRITICAL`) |
| `SensorFrame s_frame` | ~96 B | `scheduler_poll()` (Core 1) | `feed_sensors_to_engines()` | Core 1 exclusive (no concurrency) |
| Safety & Correlator State | ~1.1 KB | Core 1 only | Core 1 only | Core 1 exclusive (no concurrency) |

### BLE RX Queue Synchronization

```cpp
// Cross-core spinlock in ble_relay.cpp
static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;

// Core 0 (Bluedroid scan callback):
portENTER_CRITICAL(&s_rx_mux);
if (rx_count_ < RX_QUEUE_SIZE) {
    rx_queue_[idx].payload = payload;
    rx_queue_[idx].rssi    = rssi;
    rx_queue_[idx].valid   = true;
    rx_count_++;
}
portEXIT_CRITICAL(&s_rx_mux);

// Core 1 (BleRelay::tick()):
portENTER_CRITICAL(&s_rx_mux);
if (rx_count_ > 0) {
    entry = rx_queue_[rx_head_];
    rx_head_ = (rx_head_ + 1) % RX_QUEUE_SIZE;
    rx_count_--;
    got = true;
}
portEXIT_CRITICAL(&s_rx_mux);
```

The critical section copies a 20-byte struct (`EmergencyPayload` + `int rssi`) and completes in < 2 µs.

---

## Timing Invariants

### Sampling Cadence

| Parameter | Rate | Period | Driving Mechanism |
|---|---|---|---|
| MAX30102 PPG | 80 Hz | 12,500 µs | Hardware Timer 0 ISR (`MAX30102_SAMPLE_US`) |
| MPU6050 IMU | 100 Hz | 10,000 µs | Hardware Timer 1 ISR (`MPU6050_SAMPLE_US`) |
| MAX30205 Skin Temp | 0.05 Hz | 20,000 ms | Software check in `scheduler_poll()` (`MAX30205_SAMPLE_MS`) |
| BME280 + MQ135 Env | 0.02 Hz | 50,000 ms | Software check in `scheduler_poll()` (`ENV_SAMPLE_MS`) |
| Eval Tick | 1 Hz | 1,000 ms | Software timer in `loop()` (`EVAL_TICK_MS`) |
| OLED Refresh | 2 Hz | 500 ms | Software throttle in `loop()` (`DISPLAY_UPDATE_MS`) |

### Per-Loop Timing Budget

| Pipeline Step | Typical Execution | Worst-Case Execution | Notes |
|---|---|---|---|
| `scheduler_poll()` | ~200 µs | ~400 µs | I2C transaction when flags are set |
| `feed_sensors_to_engines()` | ~35 µs | ~60 µs | Peak detector EMA + rolling variance updates |
| `run_eval_tick()` (1 Hz) | ~0 µs | ~90 µs | 6-feature z-score distance + correlator tick |
| `orchestrate_comms()` | ~20 µs | ~45 µs | AT engine line parsing and BLE RX drain |
| `update_display()` (2 Hz) | ~0 µs | ~800 µs | 128×64 byte transfer over 400 kHz I2C |
| **Total Iteration Time** | **~260 µs** | **~1.4 ms** | **Deadline: 10.0 ms (100 Hz IMU)** |

---

## I2C Bus Arbitration

All 4 sensors and the OLED share a single 400 kHz Fast-Mode I2C bus (`I2C_CLOCK_HZ`). Contention and lockups are prevented through three mechanisms:

1. **Strictly Serial Bus Access**: All reads occur within the Core 1 main loop. No I2C call is ever attempted from an interrupt service routine.
2. **Display Rate Limiting**: SSD1306 buffer transfers (~800 µs) are capped at 2 Hz, reserving >98% of bus bandwidth for 80 Hz PPG and 100 Hz IMU polling.
3. **Hardware Bus Recovery (`i2c_bus_recover`)**: If an I2C transaction times out (e.g. slave holding SDA low):
   - Detaches the hardware I2C peripheral (`Wire.end()`).
   - Bit-bangs 9 clock pulses on SCL (5 µs half-period) to allow stuck slaves to release SDA.
   - Generates a manual STOP condition (SDA low-to-high transition while SCL is high).
   - Reinitializes `Wire.begin(8, 9, 400000)`.

---

## SRAM Allocation Map

Exact byte footprints of application structures (compiled on 32-bit Xtensa architecture):

```
ESP32-S3 Internal SRAM: 512 KB
  ├── System & Network Stacks:
  │   ├── FreeRTOS Kernel & Core 0/1 Idle Tasks: ~25 KB
  │   ├── Bluedroid BLE Stack & Controller:      ~65 KB
  │   ├── ESP-IDF Driver & Hal Buffers:          ~18 KB
  │   └── Arduino Core Runtime Heap:             ~30 KB
  │
  ├── Application Static State (~3.2 KB total):
  │   ├── SafetyEngine:                         ~744 B
  │   │   ├── PpgQualifier state:                 ~16 B
  │   │   ├── FallDetector (RollingVariance<100>):~428 B
  │   │   ├── Motion variance (RollingVariance<50>):~216 B
  │   │   ├── Alert array (8 × Alert [8B]):       ~64 B
  │   │   └── State & timing scalars:             ~20 B
  │   │
  │   ├── Correlator:                           ~280 B
  │   │   ├── Window circular buffer (45 × 4B):   ~180 B
  │   │   ├── Running category counters:           ~8 B
  │   │   ├── Trigger description buffer:         ~80 B
  │   │   └── State & baseline scalars:           ~12 B
  │   │
  │   ├── FeatureExtractor:                     ~950 B
  │   │   ├── PpgPeakDetector (EMA + 16 peaks):   ~96 B
  │   │   ├── Accel variance (RollingVariance<200>):~816 B
  │   │   └── Temperature slope & state scalars:  ~38 B
  │   │
  │   ├── AnomalyEngine:                         ~96 B
  │   │   ├── BaselineStats (sum[6], sum_sq[6], count): ~52 B
  │   │   └── Lifecycle & threshold scalars:      ~44 B
  │   │
  │   ├── DegradedSos:                          ~650 B
  │   │   ├── AtEngine (cmd[80], line[128], data[128]): ~356 B
  │   │   ├── SMS body buffer:                   ~180 B
  │   │   ├── SosPayload struct:                  ~68 B
  │   │   └── Phone buffer & FSM state:           ~46 B
  │   │
  │   ├── BleRelay:                             ~200 B
  │   │   ├── RxEntry queue (4 × 24B):            ~96 B
  │   │   ├── DedupRing (16 × 4B + indices):      ~72 B
  │   │   └── Broadcast & state variables:        ~32 B
  │   │
  │   └── SensorFrame & Timing Bookkeeping:     ~100 B
  │
  ├── Main Task Stack:                            8 KB
  └── Free Available SRAM:                     >350 KB
```

---

## SOS Escalation State Machine

```
                    ┌──────────────────┐
                    │   MONITORING     │
                    │  (normal ops)    │
                    └────────┬─────────┘
                             │
                   ┌─────────┴─────────┐
                   │                   │
              Path A               Path B
           (fall/SpO2<90)      (2-of-4 window)
                   │                   │
                   └────────┬──────────┘
                            │
                            ▼
               ┌────────────────────────┐
               │  DegradedSos::         │
               │  request_sos()         │
               └────────────┬───────────┘
                            │
                            ▼
               ┌────────────────────────┐
               │  POWERING_ON           │
               │  (4s capacitor charge) │
               └────────────┬───────────┘
                            │
                    ATE0 → OK → AT+CMGF=1 → OK
                            │
                            ▼
               ┌────────────────────────┐
               │  AT+CSQ → AT+CREG     │
               │  Signal Assessment     │
               └──┬─────────┬────────┬─┘
                  │         │        │
           CSQ≥15,CREG=1  5≤CSQ≤14  CSQ<5 or
           (GOOD)         (MARGINAL)  no CREG
                  │         │        (NONE)
                  │         │          │
                  ▼         ▼          ▼
            Full SMS   Compact SMS   Power OFF
           (6dp,text)  (4dp,hex)    GPIO14 HIGH
                  │         │          │
               OK─┤    ┌─ERROR─┐       │
                  │    │       │       ▼
              DONE_    │   BACKOFF   BLE burst
             SUCCESS   │  5s→15s→45s 250ms ADV
                       │       │       │
                       │    retry      │
                       │       │    120s SLEEP
                       │       │       │
                       └───────┤    WAKE → re-probe
                               │       │
                        3 retries      │
                        exhausted ─────┘
                               │
                               ▼
                          BLE FALLBACK
                         ble_fallback_needed()
```

---

## BLE Relay Protocol

### Legacy Frame Format (22 of 31 Bytes)

```
Byte  0     1     2   │  3     4     5     6   │  7-8    9    10-13  14-17  18   19   20-21
┌─────┬─────┬─────┬───┼──┬─────┬─────┬─────┬───┼──────┬─────┬──────┬──────┬────┬────┬──────┐
│0x02 │0x01 │0x06 │   │ 0x12│0xFF │0xFF │0xFF │   │devid │alert│ lat  │ lon  │seq │hop │ crc  │
│ len │flags│value│   │ len │ mfr │ CID │ CID │   │uint16│uint8│int32 │int32 │u8  │u8  │uint16│
└─────┴─────┴─────┘   └─────┴─────┴─────┴─────┘   └──────┴─────┴──────┴──────┴────┴────┴──────┘
 ◄── AD Flags (3B) ─►  ◄── Mfr Specific (19B) ──────────────────────────────────────────────►
```

- **Coordinates**: Fixed-point encoded ($10^{-4}$ precision $\approx 11\text{m}$ resolution).
- **CRC**: CRC-16-CCITT-FALSE ($x^{16}+x^{12}+x^5+1$, init $0\text{xFFFF}$) computed across bytes 7 through 19.

### Single-Hop Relay Decision Logic

```
Incoming BLE Frame with Company ID 0xFFFF
    │
    ├─ CRC-16-CCITT mismatch?          → DROP (corrupted frame)
    │
    ├─ hop_count ≥ 1?                  → DROP (prevents multi-hop broadcast storm)
    │
    ├─ {device_id, seq} in ring buffer? → DROP (suppresses duplicate packet)
    │
    └─ Valid Origin Packet (hop_count == 0):
         1. Store {device_id, sequence_num} in 16-entry ring buffer.
         2. Set hop_count = 1.
         3. Trigger relay callback → DegradedSos::request_sos().
         4. Halt further BLE re-broadcast (never re-transmit over BLE).
```

---

## Anomaly Engine Lifecycle

```
Boot (t = 0)
  │
  ▼
┌──────────┐  60 minutes of data  ┌──────────┐  Motion variance > 2.0  ┌───────────┐
│ DORMANT  │─────────────────────→│  ACTIVE  │────────────────────────→│ SUSPENDED │
│          │                      │          │                         │ (5 min)   │
│ Collect  │                      │ Z-score  │                         │ Score     │
│ baseline │                      │ distance │                         │ dampened  │
│ sum &    │                      │ scoring  │                         │ by 50%    │
│ sum_sq   │                      │          │                         │           │
└──────────┘                      └──────────┘←────────────────────────└───────────┘
                                                Low motion for 5 min (cooldown)

DORMANT State:
  - Continuously accumulates sum[6] and sum_sq[6] for valid feature vectors.
  - Outputs neutral confidence modifier (0.5).
  - At t = 60 min, computes mean and standard deviation per feature, and calibrates θ = μ_score + 3σ_score.

ACTIVE State:
  - Computes squared Euclidean distance in z-score space: score = (1/N) × Σ((fᵢ − μᵢ)/σᵢ)².
  - If score > θ: anomaly detected, outputting confidence boost (> 0.5).
  - Handles up to 2 missing/corrupted features per vector via baseline mean substitution.

SUSPENDED State:
  - Triggered when accelerometer variance exceeds 2.0 (m/s²)² (e.g. running or physical labor).
  - Dampens calculated score by 50% to prevent false positives from exercise physiology.
  - Automatically transitions back to ACTIVE after 5 minutes of quiet motion.
```
