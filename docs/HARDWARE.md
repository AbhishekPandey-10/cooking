# Hardware Setup & Wiring Guide

> Complete assembly reference and electrical specification for the Disaster Health Companion wearable.

---

## Bill of Materials

| # | Component | Part Number / Module | Qty | Power Rail | Interface | Notes |
|---|---|---|---|---|---|---|
| 1 | ESP32-S3 DevKit | ESP32-S3-DevKitC-1 (N16R8) | 1 | 5V / LiPo | USB / UART | Xtensa dual-core LX7, 512KB SRAM |
| 2 | MAX30102 Breakout | GY-MAX30102 | 1 | 3.3V | I2C (`0x57`) | PPG / SpO2, onboard optical window |
| 3 | MPU6050 Breakout | GY-521 | 1 | 3.3V | I2C (`0x68`) | 6-axis IMU (±8g, ±1000°/s) |
| 4 | MAX30205 Breakout | CJMCU-30205 | 1 | 3.3V | I2C (`0x48`) | Clinical human body temp (±0.1°C) |
| 5 | BME280 Breakout | GY-BME280 | 1 | 3.3V | I2C (`0x76`) | Ambient temp / humidity / pressure |
| 6 | MQ135 Module | Waveshare / Standard MQ135 | 1 | 5V (heater) | Analog (GPIO 1) | Air quality / smoke proxy, continuous drive |
| 7 | SSD1306 OLED | 0.96" 128×64 Monochrome | 1 | 3.3V | I2C (`0x3C`) | Visual vital/alert/calibration display |
| 8 | SIM800L Module | SIM800L EVB / Core board | 1 | VBAT (3.7V–4.2V) | UART1 (115200) | Quad-band GSM/GPRS, 2A peak pulses |
| 9 | P-MOSFET | SI2301DS (SOT-23) | 1 | LiPo rail | GPIO 14 gate | High-side power gate (Rds(on) ~100mΩ) |
| 10 | Decoupling Cap | 2200 µF 6.3V / 10V Low-ESR | 1 | VBAT to GND | Across modem | Buffers 2A GSM burst transmission |
| 11 | Pull-up Resistors | 4.7 kΩ 0.125W | 2 | 3.3V | I2C SDA / SCL | Bus pull-ups (verify parallel net ≥ 2.2kΩ) |
| 12 | GSM Antenna | Quad-band IPEX / SMA | 1 | — | u.FL on modem | Required for network registration |
| 13 | SIM Card | Nano SIM | 1 | — | SIM socket | Active 2G/GSM network with SMS |
| 14 | LiPo Battery | 3.7V 1000mAh–3000mAh | 1 | LiPo (+) to (-) | JST connector | Main system power source |

---

## Wiring Diagram

### I2C Bus (400 kHz Fast-Mode)

All four I2C sensors and the OLED display share a single bus on GPIO 8 (SDA) and GPIO 9 (SCL).

```
ESP32-S3                  I2C Bus (400 kHz Fast-Mode, 3.3V)
┌──────┐                  ┌─────────────────────────────────────────┐
│ GPIO8├──── SDA ────┬───┤ MAX30102 (0x57)                         │
│      │             │   ├─────────────────────────────────────────┤
│ GPIO9├──── SCL ────┼───┤ MPU6050  (0x68)                         │
│      │             │   ├─────────────────────────────────────────┤
│      │        4.7kΩ│   │ MAX30205 (0x48)                         │
│      │      ┌──┤├──┤   ├─────────────────────────────────────────┤
│ 3.3V ├──────┤      │   │ BME280   (0x76)                         │
│      │      ├──┤├──┤   ├─────────────────────────────────────────┤
│      │      │ 4.7kΩ│   │ SSD1306  (0x3C)                         │
│      │      │      │   └─────────────────────────────────────────┘
│      │      └──────┘
│      │
│      │   Note: Measure parallel resistance between SDA/SCL and 3.3V.
│      │   Breakout modules often have onboard 10k or 4.7k pull-ups.
│      │   Ensure effective bus resistance stays between 2.2 kΩ and 4.7 kΩ.
└──────┘
```

### MQ135 Gas Sensor (Analog ADC1_CH0)

```
ESP32-S3                   MQ135 Module
┌────────┐                 ┌────────────┐
│  GPIO1 ├───── AOUT ──────┤ Analog Out │ (0–3.3V signal into ADC1_CH0)
│        │                 │            │
│  5V    ├───── VCC  ──────┤ VCC (5V)   │ (Heater requires 5V from USB/5V rail)
│ (VBUS) │                 │            │
│  GND   ├───── GND  ──────┤ GND        │
└────────┘                 └────────────┘

Critical Notes:
1. The MQ135 internal SnO2 heating coil draws ~150 mA and requires 5.0V ± 0.2V.
   Powering from 3.3V leaves the sensor under-temperature and uncalibrated.
2. The analog output voltage must not exceed 3.3V. Standard breakout modules with
   a load resistor divider provide 0–3.3V into high-impedance ADC inputs.
3. The heater must be powered continuously (no duty-cycling). A 90-second warmup
   blackout is enforced by firmware before readings are considered valid.
```

### SIM800L Modem & SI2301DS High-Side Power Gate

> [!CAUTION]
> **DO NOT connect SIM800L VBAT to the ESP32-S3 3.3V rail!**  
> The SIM800L operating voltage is **3.4V – 4.4V** (recommended: 3.8V – 4.2V). During GSM transmission bursts, current spikes reach **2.0 Amperes**. Attempting to power the modem from a 3.3V LDO will undervolt the modem and trigger brownout resets on the ESP32.

```
LiPo Battery (+)
  [3.7V–4.2V]
      │
      ├─── Source (S)
      │    ┌───────────┐
      │    │ SI2301DS  │
      │    │ P-MOSFET  │
GPIO 14 ──┤ Gate (G)  │
           │           │
           │ Drain (D) ┴──────┐
           └───────────┘      │
                              ├─── (+) 2200µF Low-ESR Cap
                              │    (buffering 2A TX bursts)
                              ├─── VBAT [SIM800L Module]
                              │
GND ──────────────────────────┴─── GND  [SIM800L Module]

ESP32-S3
  GPIO 17 ──────── TXD ──────────► RXD  [SIM800L UART]
  GPIO 18 ◄─────── RXD ─────────── TXD  [SIM800L UART]
```

#### Power Gating Control Logic
- **GPIO 14 = LOW (0V)**: Gate pulled down. $|V_{gs}| \approx 3.7\text{V} - 4.2\text{V}$, MOSFET conducts fully ($R_{ds(on)} \approx 0.1\ \Omega$), powering SIM800L VBAT. Firmware enforces a 4000ms pause for capacitor charging and baseband boot before AT commands.
- **GPIO 14 = HIGH (3.3V)**: Gate voltage raised to cut off the MOSFET, cutting modem power completely during BLE fallback or sleep cycles.
  *(Note: For battery voltages > 3.8V, an NPN transistor or small N-FET driving the P-FET gate with a pullup to LiPo (+) provides complete cut-off if $V_{gs(th)}$ leakage occurs).*

---

## I2C Address Map

| Address | Device | Function | Detection / Verification |
|---|---|---|---|
| `0x3C` | SSD1306 | 128×64 OLED Display | `sensors_init()` I2C probe |
| `0x48` | MAX30205 | Clinical Skin Temperature | `sensors_init()` configuration register |
| `0x57` | MAX30102 | PPG / SpO2 Optical Sensor | `sensors_init()` Part ID register (`0x15`) |
| `0x68` | MPU6050 | 6-Axis Accelerometer & Gyro | `sensors_init()` WHO_AM_I register (`0x68`) |
| `0x76` | BME280 | Barometric Climate & Humidity | `sensors_init()` Chip ID register (`0x60`) |

### Address Conflict Check
All 5 I2C devices occupy distinct, non-overlapping addresses on the 7-bit bus. No address multiplexer (e.g. TCA9548A) is required.

---

## Sensor Placement Notes

### MAX30102 (PPG / SpO2)
- **Location**: Inner wrist (radial artery) or fingertip clip.
- **Contact**: Direct contact with clean optical aperture. An IR count < 5000 indicates contact loss, automatically suppressing SpO2 alerting and re-triggering the 100-sample calibration window upon restoration.
- **Occlusion**: Shield sensor edges from direct sunlight to prevent ambient light photodiode saturation.

### MPU6050 (IMU)
- **Location**: Securely fastened to rigid chassis/casing.
- **Orientation**: Z-axis perpendicular to wearer's wrist surface. Fall detection uses total Euclidean acceleration magnitude $\|a\| = \sqrt{a_x^2 + a_y^2 + a_z^2}$, making it invariant to static orientation.

### MAX30205 (Skin Temperature)
- **Location**: Conductive thermal contact tab pressed against inner skin surface.
- **Thermal Response**: Requires ~30 seconds of stable contact to reach thermal equilibrium with epidermis.

### BME280 (Climate)
- **Location**: Vented to outside air through a dust/water resistant membrane (e.g. Gore-Tex vent).
- **Isolation**: Thermally isolated from ESP32-S3 chip heat and battery pack to avoid temperature bias.

### MQ135 (Gas Sensor)
- **Location**: Ambient air intake slot.
- **Thermal Management**: Heating element operates at ~150°C internally. Ensure enclosure plastic does not touch the metal sensor gauze.

---

## Assembly Checklist

- [ ] Connect LiPo battery (+) to SI2301DS Source pin (NOT 3.3V rail).
- [ ] Connect SI2301DS Drain to SIM800L VBAT pin with 2200µF low-ESR capacitor in parallel.
- [ ] Connect SI2301DS Gate to ESP32-S3 GPIO 14.
- [ ] Connect SIM800L TXD to ESP32-S3 GPIO 18 (RX), SIM800L RXD to ESP32-S3 GPIO 17 (TX).
- [ ] Connect MQ135 VCC to 5V (USB VBUS rail), AOUT to GPIO 1.
- [ ] Connect all I2C peripherals (MAX30102, MPU6050, MAX30205, BME280, SSD1306) to 3.3V rail.
- [ ] Tie I2C SDA to GPIO 8, SCL to GPIO 9.
- [ ] Solder 4.7 kΩ pull-ups on SDA and SCL to 3.3V (measure total bus pullup: must be between 2.2kΩ and 4.7kΩ).
- [ ] Common all GND connections across battery, ESP32, sensors, and modem.
- [ ] Boot and verify Serial output: `[MAIN] All sensors online (mask 0x1F)`.

---

## Power Considerations

### Current Consumption Breakdown

| Subsystem | Active State | Low-Power / Idle | Notes |
|---|---|---|---|
| ESP32-S3 (240MHz) | 80–120 mA | 15–25 mA (Modem sleep) | Dual-core CPU, BLE scan continuous |
| MAX30102 | ~1.2 mA | < 1 µA | 80 Hz sample rate, 50mA LED pulse |
| MPU6050 | 3.8 mA | 5 µA | 100 Hz continuous sampling |
| MAX30205 | 0.6 mA | 1.6 µA | Sampled once per 20 seconds |
| BME280 | 0.1 mA | 0.1 µA | Forced mode every 50 seconds |
| SSD1306 OLED | 10–18 mA | < 10 µA | 2 Hz update rate, inverted display off |
| MQ135 Sensor | ~150 mA | ~150 mA | Continuous heating required |
| SIM800L (Idle) | 15–20 mA | 1 mA (Sleep) | Power gated when idle |
| SIM800L (Transmitting) | 200–450 mA | 2.0 A peak pulses | Handled by 2200µF capacitor |

### Battery Life Estimates
- **With 1200 mAh LiPo**: ~4.5 hours of continuous operation (governed primarily by the 150mA MQ135 heater and ESP32 active core).
- **With 2600 mAh 18650 Cell**: ~10 hours of continuous field operation.
