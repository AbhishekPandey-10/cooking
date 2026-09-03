# 🩺 Disaster Health Companion — ESP32-S3

> **An autonomous wearable health monitor and emergency beacon for disaster first responders, built on ESP32-S3 with deterministic safety logic, multi-sensor anomaly detection, degraded-network SMS escalation, and connectionless BLE mesh relay.**

---

## Overview

The Disaster Health Companion is a bare-metal embedded system designed for environments where cellular infrastructure is damaged or intermittent — earthquakes, floods, building collapses, wildfires. It continuously monitors the wearer's vital signs using five biometric and environmental sensors (four on a shared 400 kHz I2C bus, one analog gas sensor on ADC), evaluates physiological danger through a three-tier analysis pipeline (deterministic thresholds → cross-sensor correlation → statistical anomaly detection), and autonomously dispatches SOS alerts through the best available channel: cellular SMS when signal exists, or a connectionless BLE advertising relay when it doesn't.

### Key Design Principles

- **Zero dynamic allocation** — all buffers are fixed-size, stack-allocated, and bounded at compile time (~3.2 KB total module state).
- **Deterministic timing** — all state machines accept `uint32_t now_ms` injection. No internal `millis()` or `delay()` calls in any analysis or communication module.
- **Fail-safe degradation** — every layer assumes the layer below it can fail. I2C bus recovery (9 SCL bit-bang pulses + manual STOP), modem power cycling (SI2301DS P-MOSFET), BLE fallback, and sensor-loss masking are built in.
- **Testable by design** — all safety-critical logic (fall detection, heat index, SpO2 thresholds, correlator, anomaly engine, CRC-16, dedup ring buffer) is pure C++ with zero hardware dependencies, validated by 180 unit tests.

---

## Hardware

### Bill of Materials

| Component | Role | Interface | Bus / Rail |
|---|---|---|---|
| **ESP32-S3 DevKit** | Microcontroller (Xtensa dual-core LX7) | — | 3.3V / 5V |
| **MAX30102** | PPG / SpO2 / Heart Rate | I2C (`0x57`) | 3.3V |
| **MPU6050** | 6-axis IMU (fall detection) | I2C (`0x68`) | 3.3V |
| **MAX30205** | Clinical skin temperature (±0.1°C) | I2C (`0x48`) | 3.3V |
| **BME280** | Ambient temp / humidity / pressure | I2C (`0x76`) | 3.3V |
| **MQ135** | Gas / air quality sensor | Analog ADC1_CH0 (GPIO 1) | 5V Heater / 3.3V ADC |
| **SSD1306** | 128×64 OLED display | I2C (`0x3C`) | 3.3V |
| **SIM800L** | GSM/GPRS modem (SMS) | UART1 (115200 baud) | VBAT (3.7V–4.2V LiPo) |
| **SI2301DS** | P-MOSFET power gate for modem | GPIO 14 (Gate) | High-side on LiPo rail |
| **2200 µF Cap** | Low-ESR decoupling for SIM800L VBAT | — | Across modem VBAT/GND |

### Pinout

```
ESP32-S3 DevKit
  ├── GPIO 8  ──── I2C SDA (400 kHz Fast-Mode: MAX30102, MPU6050, MAX30205, BME280, SSD1306)
  ├── GPIO 9  ──── I2C SCL
  ├── GPIO 1  ──── MQ135 analog output (ADC1_CH0, 12-bit 0–4095)
  ├── GPIO 14 ──── SI2301DS P-FET gate (LOW = modem ON, HIGH = modem OFF)
  ├── GPIO 17 ──── SIM800L TX (ESP32 TX → modem RXD)
  └── GPIO 18 ──── SIM800L RX (modem TXD → ESP32 RX)

Power Rails
  ├── 3.3V Rail   ── ESP32-S3 VCC, I2C pull-ups, MAX30102, MPU6050, MAX30205, BME280, SSD1306
  ├── 5V Rail     ── MQ135 heater element (VBUS)
  └── LiPo VBAT   ── SI2301DS Source → Drain → SIM800L VBAT (with 2200µF buffer cap)
```

---

## Architecture

The firmware is organized as a pipeline of decoupled modules, each with feed/query/tick interfaces and zero hardware coupling in the analysis layers.

```
┌──────────────────── Sensor Layer ─────────────────────┐
│  MAX30102   MPU6050   MAX30205   BME280   MQ135       │
│   80 Hz     100 Hz    0.05 Hz   0.02 Hz  0.02 Hz     │
└──────────────────┬────────────────────────────────────┘
                   │
                   ▼
┌──────────── Tier 1: Deterministic Safety ─────────────┐
│  SafetyEngine                                         │
│  ├── PpgQualifier   (buffer-fill masking, contact)    │
│  ├── FallDetector   (4-state FSM: freefall→impact)    │
│  ├── SpO2 thresholds (92% sustained, 90% immediate)   │
│  └── HeatIndex      (NWS 9-term Rothfusz polynomial)  │
└──────────────────┬────────────────────────────────────┘
                   │
                   ▼
┌──────────── Tier 2: Cross-Sensor Correlation ─────────┐
│  Correlator                                           │
│  ├── Path A: hard bypass (fall OR SpO2 < 90%)         │
│  └── Path B: 45s sliding window, 2-of-4 coincidence   │
│       ├── Optical  (SpO2 90–92%)                      │
│       ├── Heat     (HI > 38°C)                        │
│       ├── Gas      (MQ135 Δ > 150 ADC counts)         │
│       └── Thermal  (skin temp < 30°C or > 38.5°C)     │
└──────────────────┬────────────────────────────────────┘
                   │
                   ▼
┌──────────── Tier 3: Statistical Anomaly Detection ────┐
│  FeatureExtractor → AnomalyEngine                     │
│  ├── 6-feature vector (HR, RMSSD, SpO2, skin temp,    │
│  │    skin temp slope, accel variance)                 │
│  ├── 60-min cold-start baseline collection             │
│  ├── Z-score distance anomaly scoring                  │
│  └── Confidence modifier (0.0 suppress ↔ 1.0 boost)   │
└──────────────────┬────────────────────────────────────┘
                   │
                   ▼
┌──────────── Communication Layer ──────────────────────┐
│  DegradedSos (Phase 7.5)                              │
│  ├── AT+CSQ / AT+CREG signal probing                  │
│  ├── STATE_GOOD:     full SMS (6-decimal coords)       │
│  ├── STATE_MARGINAL: compact SMS + exponential backoff │
│  └── STATE_NONE:     P-MOSFET modem cut → BLE fallback│
│                                                        │
│  BleRelay (Phase 10)                                  │
│  ├── 22-byte raw legacy advertising frame              │
│  ├── CRC-16-CCITT integrity                            │
│  ├── 16-entry dedup ring buffer                        │
│  └── Single-hop relay (no re-broadcast storms)         │
└───────────────────────────────────────────────────────┘
```

---

## Modules

### Phase 1 — Sensor Management & Bus Resilience

| File | Description |
|---|---|
| `include/config.h` | Pin assignments, I2C addresses, sampling rates, and hardware thresholds. |
| `include/sensor_types.h` | Pure data structures (`PpgReading`, `ImuReading`, `TempReading`, `EnvReading`, `SensorFrame`) with zero hardware dependencies. |
| `include/sensor_manager.h` | Individual sensor read functions (`read_ppg`, `read_imu`, `read_skin_temp`, `read_environment`) decoupled from scheduling. |
| `src/sensor_manager.cpp` | I2C device initialization with per-sensor error reporting. MQ135 warm-up blackout gating. SSD1306 OLED driver. |
| `include/i2c_recovery.h` | I2C bus recovery API. |
| `src/i2c_recovery.cpp` | Detaches the I2C peripheral (`Wire.end()`), bit-bangs 9 SCL clock pulses to release a stuck slave, issues a manual STOP condition (SDA low→high while SCL high), then reinitializes `Wire.begin()`. |
| `include/sampling_scheduler.h` | Timer-driven multi-rate scheduler API. |
| `src/sampling_scheduler.cpp` | Two hardware timer ISRs (80 Hz PPG, 100 Hz IMU) set volatile flags; `scheduler_poll()` checks flags and performs I2C reads in the main loop context. Software timers for 20s/50s low-rate sensors. |

### Phase 2–3 — Tier 1 Deterministic Safety Engine

| File | Description |
|---|---|
| `include/safety_types.h` | `AlertSeverity`, `AlertType`, `PpgState`, `FallState` enums. `RollingVariance<N>` templated online variance calculator. Tunable thresholds in `SafetyConfig` namespace. |
| `include/safety_engine.h` | Feed-Query-Rebuild orchestrator. Accepts raw sensor values via `feed_ppg()`, `feed_imu()`, `feed_environment()`. Rebuilds the active alert array after each feed. |
| `src/safety_engine.cpp` | Alert suppression hierarchy: PPG not VALID → suppress SpO2 alerts. Motion artifact → suppress + flag. SpO2 < 90% → immediate CRITICAL. SpO2 < 92% sustained 30s → WARNING. Fall confirmed → CRITICAL. Heat index thresholds. |
| `include/fall_detector.h` | 4-state FSM: `IDLE → FREEFALL_DETECTED → IMPACT_DETECTED → FALL_CONFIRMED`. |
| `src/fall_detector.cpp` | Freefall (\|a\| < 0.2g) → impact (\|a\| > 3.0g within 300ms) → immobility (variance < 0.05 for 5s). |
| `include/heat_index.h` | NWS Rothfusz polynomial API. |
| `src/heat_index.cpp` | Full 9-term NWS regression polynomial (Celsius input/output, internal °F conversion). Steadman linear fallback for T < 26.7°C or RH < 40%. Low-RH and high-RH band adjustments. |
| `include/ppg_qualify.h` | PPG signal quality state machine API. |
| `src/ppg_qualify.cpp` | States: `CALIBRATING` (< 100 samples in ring buffer) → `VALID`. Transitions to `CONTACT_LOST` when IR < 5000 counts. Automatic re-calibration on contact loss recovery. |

### Phase 4.5 — Cross-Sensor Correlation Engine

| File | Description |
|---|---|
| `include/correlator.h` | Dual-path decision engine with configurable constants in `CorrelatorConfig` namespace. |
| `src/correlator.cpp` | **Path A**: fall detected OR SpO2 < 90% → immediate `HARD_BYPASS` (zero delay). **Path B**: 45-second sliding window (circular buffer, 1 Hz tick) tracking 4 boolean flags. O(1) per-tick via running counters. MQ135 delta threshold: 150 ADC counts (~5–7× above noise floor). `last_trigger_description()` returns human-readable flag breakdown for logging. |

### Phase 5–6 — Anomaly Detection Engine

| File | Description |
|---|---|
| `include/feature_extract.h` | `FeatureVector` (6 features) + streaming `PpgPeakDetector` with EMA baseline and hysteresis-based peak detection. |
| `src/feature_extract.cpp` | Peak detector: O(1) per sample, ~96 bytes. HR from mean inter-peak interval. RMSSD (HRV) from successive IPI differences. Skin temp slope from consecutive readings (°C/min). Accel variance via `RollingVariance<200>`. All features have physiological sanity bounds. |
| `include/anomaly_engine.h` | Cold-start state machine: `DORMANT → ACTIVE → SUSPENDED`. `BaselineStats` running sum/sum_sq. |
| `src/anomaly_engine.cpp` | 60-minute dormancy accumulates per-feature mean and standard deviation. Anomaly threshold θ calibrated at μ + 3σ of the score distribution observed during dormancy. Z-score distance: `score = (1/N) × Σ((fᵢ − μᵢ)/σᵢ)²`. Post-exercise motion cooldown dampens score by 50% for 5 minutes. NaN substitution: up to 2 missing features replaced with baseline mean. |

### Phase 7.5 — Degraded-Network SOS Escalation

| File | Description |
|---|---|
| `include/degraded_sos.h` | Non-blocking AT engine + 16-state SOS machine. All thresholds in `SosConfig` namespace. |
| `src/degraded_sos.cpp` | **AT Engine**: line accumulator with echo suppression (ATE0), garbage character filtering, CME/CMS ERROR parsing, SMS prompt (`>`) detection, and configurable per-command timeouts. **SOS State Machine**: `POWERING_ON` (4s cap charge) → `INIT_ECHO_OFF` → `INIT_TEXT_MODE` → `PROBE_CSQ` → `PROBE_CREG` → `EVALUATE_SIGNAL` → SMS dispatch or BLE fallback. Exponential backoff: 5s → 15s → 45s. Sleep/wake: 120s power-off cycle. **Power Gating**: SI2301DS P-MOSFET on GPIO 14 (LOW = ON, HIGH = OFF). |

### Phase 10 — BLE Advertising Relay Protocol

| File | Description |
|---|---|
| `include/ble_relay_types.h` | Pure C++ packet types, CRC-16-CCITT, `DedupRing`, `evaluate_relay()`, AD frame construction/parsing. Zero BLE dependencies. |
| `include/ble_relay.h` | `BleRelay` class with cross-core RX queue (4 entries, `portMUX` spinlock). |
| `src/ble_relay.cpp` | Arduino BLE library for scan callbacks. ESP-IDF `esp_ble_gap_config_adv_data_raw()` for raw non-connectable advertising. 22-byte legacy ADV frame (3B flags + 19B manufacturer-specific). |

### Main Integration

| File | Description |
|---|---|
| `src/main.cpp` | Production firmware wiring all modules. ISR-flag-driven sensor polling → safety/feature feeds at native rates → 1 Hz eval tick → SOS escalation with automatic cellular→BLE fallback → 2 Hz OLED throttle. Single-threaded on Core 1 (only BLE scan callback on Core 0). |

---

## Building & Flashing

### Prerequisites

- [PlatformIO Core](https://platformio.org/install/cli) or [PlatformIO IDE](https://platformio.org/install/ide) (VS Code extension)
- USB cable for ESP32-S3 DevKit

### Build

```bash
# Clone the repository
git clone <repository-url>
cd "The proj"

# Build the firmware
pio run

# Build + flash
pio run --target upload

# Monitor serial output (115200 baud)
pio device monitor --baud 115200
```

### Configuration

Edit `include/config.h` for hardware pin assignments and sampling rates. Edit the `SosConfig`, `CorrelatorConfig`, `BleRelayConfig`, and `AnomalyConfig` namespaces in their respective headers for tunable thresholds.

Key configuration points:

| What | Where | Default |
|---|---|---|
| I2C pins | `config.h` | SDA=8, SCL=9 (400 kHz) |
| MQ135 ADC pin | `config.h` | GPIO 1 |
| Modem UART pins | `main.cpp` → `MainConfig` | TX=17, RX=18 |
| Modem power pin | `degraded_sos.h` → `SosConfig` | GPIO 14 |
| Emergency phone number | `main.cpp` → `setup()` | `+911234567890` |
| BLE device ID | `main.cpp` → `MainConfig` | `0xA1B2` |
| SpO2 thresholds | `safety_types.h` → `SafetyConfig` | 92% warn, 90% critical |
| Fall detection timing | `safety_types.h` → `SafetyConfig` | 300ms impact, 5s immobility |
| Correlator window | `correlator.h` → `CorrelatorConfig` | 45s, 2-of-4 |
| Backoff intervals | `degraded_sos.h` → `SosConfig` | 5s, 15s, 45s |
| Anomaly dormancy | `anomaly_engine.h` → `AnomalyConfig` | 60 minutes |

---

## Testing

### Unit Tests

All safety-critical logic has unit tests using the [Unity](https://github.com/ThrowTheSwitch/Unity) framework, runnable on the ESP32-S3 target or desktop environment via PlatformIO.

```bash
# Run all 180 tests across all 6 suites
pio test

# Run a specific suite
pio test --filter test_safety
pio test --filter test_correlator
pio test --filter test_anomaly
pio test --filter test_degraded_sos
pio test --filter test_ble_relay
pio test --filter test_sensors
```

### Test Coverage (180 Total Tests)

| Suite | Path | Tests | Coverage |
|---|---|---|---|
| `test_safety` | `test/test_safety/test_safety_engine.cpp` | 39 | Rolling variance, Steadman & Rothfusz heat index, PPG qualification, fall FSM transitions, SpO2 sustained/critical thresholds, alert suppression |
| `test_correlator` | `test/test_correlator/test_correlator.cpp` | 35 | Path A fall/SpO2 hard bypass, Path B 2-of-4 coincidence, window expiry, MQ135 delta tracking, window reset behavior |
| `test_anomaly` | `test/test_anomaly/test_anomaly_engine.cpp` | 26 | Streaming PPG peak detection, baseline statistics, cold-start lifecycle (DORMANT→ACTIVE→SUSPENDED), motion cooldown dampening, NaN substitution |
| `test_degraded_sos` | `test/test_degraded_sos/test_degraded_sos.cpp` | 35 | AT command framing (OK/ERROR/CME/CMS/prompt/timeout), echo suppression, garbage filtering, CSQ/CREG parsing, signal classification, backoff timing, sleep/wake, mock UART |
| `test_ble_relay` | `test/test_ble_relay/test_ble_relay.cpp` | 36 | CRC-16-CCITT ("123456789" vector), packed struct layout, fixed-point coords, raw AD framing, dedup ring eviction, single-hop relay decisions |
| `test_sensors` | `test/test_sensors/test_sensor_reads.cpp` | 9 | Data structure zero-initialization, MQ135 warmup blackout timing, 3D acceleration vector magnitude, sensor scale conversion |

### Bench Testing (Dual-Device BLE Relay)

See the complete dual-device walkthrough in `test/test_ble_relay/test_ble_relay.cpp` and `docs/TESTING.md`. Requires two ESP32-S3 boards and RF-absorbing foil.

---

## Project Structure

```
The proj/
├── platformio.ini              # PlatformIO build configuration
├── README.md                   # Project overview and guide
├── LICENSE                     # MIT License
│
├── include/                    # Header files (public API)
│   ├── config.h                # Hardware pins, I2C addresses, sampling rates
│   ├── sensor_types.h          # Pure data structures (no HW deps)
│   ├── sensor_manager.h        # Sensor init + individual read functions
│   ├── i2c_recovery.h          # I2C bus recovery API
│   ├── sampling_scheduler.h    # Timer-driven multi-rate scheduler
│   ├── safety_types.h          # Alert types, enums, RollingVariance<N>
│   ├── safety_engine.h         # Tier 1 safety orchestrator
│   ├── fall_detector.h         # Fall detection FSM
│   ├── heat_index.h            # NWS Rothfusz heat index
│   ├── ppg_qualify.h           # PPG signal quality state machine
│   ├── correlator.h            # Phase 4.5 cross-sensor correlator
│   ├── feature_extract.h       # Streaming feature extraction
│   ├── anomaly_engine.h        # Phase 5-6 anomaly detection
│   ├── degraded_sos.h          # Phase 7.5 SOS + AT engine
│   ├── ble_relay_types.h       # Phase 10 packet types (pure C++)
│   └── ble_relay.h             # Phase 10 BLE advertiser/scanner
│
├── src/                        # Implementation files
│   ├── main.cpp                # Production firmware (all phases integrated)
│   ├── sensor_manager.cpp      # Sensor I2C drivers + OLED
│   ├── i2c_recovery.cpp        # Bus recovery (9-clock bit-bang)
│   ├── sampling_scheduler.cpp  # HW timer ISRs + SW timer polling
│   ├── safety_engine.cpp       # Alert evaluation + suppression
│   ├── fall_detector.cpp       # 4-state fall FSM
│   ├── heat_index.cpp          # 9-term NWS polynomial
│   ├── ppg_qualify.cpp         # Buffer-fill + contact gating
│   ├── correlator.cpp          # Dual-path 45s engine
│   ├── feature_extract.cpp     # Peak detector, HR, RMSSD, slope
│   ├── anomaly_engine.cpp      # Cold-start baseline + z-score
│   ├── degraded_sos.cpp        # AT state machine + SMS + power gating
│   └── ble_relay.cpp           # BLE ADV + scan + relay
│
├── test/                       # Unit tests (Unity framework)
│   ├── test_safety/            # Safety engine tests (39 tests)
│   ├── test_correlator/        # Correlator tests (35 tests)
│   ├── test_anomaly/           # Anomaly engine tests (26 tests)
│   ├── test_degraded_sos/      # SOS engine + mock UART tests (35 tests)
│   ├── test_ble_relay/         # BLE packet + dedup tests (36 tests)
│   └── test_sensors/           # I2C data structures + warmup tests (9 tests)
│
└── docs/                       # Detailed documentation
    ├── ARCHITECTURE.md         # System architecture + concurrency model
    ├── HARDWARE.md             # Wiring guide + BOM + power analysis
    └── TESTING.md              # Full test catalogue + bench protocols
```

---

## Resource Budget

| Resource | Usage | Available | Margin |
|---|---|---|---|
| **SRAM** | ~3.2 KB (module state) | 512 KB | >99% free |
| **Flash** | ~200 KB (est. with BLE stack) | 8 MB / 16 MB | >97% free |
| **I2C bandwidth** | ~1.2 ms/loop worst case | 10 ms budget (100 Hz) | >8× |
| **Loop time** | 0.3 ms typical, 1.2 ms peak | 10 ms budget | >8× |
| **BLE ADV frame** | 22 bytes | 31 bytes max | 9 bytes spare |

---

## Safety Design Philosophy

This system is a **monitoring aid**, not a certified medical device. The multi-tier architecture is designed to minimize both false negatives (missed emergencies) and false positives (alarm fatigue):

1. **Tier 1 (Deterministic)** catches unambiguous emergencies — falls, extreme SpO2 drops — with zero latency and zero ML dependency.
2. **Tier 2 (Correlation)** requires corroborating evidence from multiple independent sensors before triggering a warning, reducing single-sensor false alarms.
3. **Tier 3 (Anomaly)** learns the wearer's personal baseline over 60 minutes, detecting physiological drift to modulate alert confidence.
4. **Communication** degrades gracefully from full-text SMS → compact SMS with exponential retry → BLE mesh relay, ensuring alerts reach help even in total infrastructure collapse.

---

## License

This project is licensed under the MIT License - see the [LICENSE](file:///w:/The%20proj/LICENSE) file for details.
