# 👤 PERSON 1: Comprehensive Presentation & Study Guide
## Biometrics, Sensor Intelligence & 3-Tier Multi-Sensor Safety Engine

> **Your Role in the Presentation:**  
> You explain the physical wearable, what sensors are on it, how the hardware bus recovers from crashes, and how our 3-tier algorithm analyzes the human body to catch genuine emergencies without ever crying wolf.
> 
> **Estimated Speaking Time:** 7–9 Minutes  
> **Preparation Time Needed:** ~45–60 Minutes to read and rehearse.

---

## 📑 Table of Contents
1. [The Big Picture (What Are We Building & Why?)](#1-the-big-picture)
2. [The Hardware & The 5 Sensors (Explained Simply)](#2-the-hardware--the-5-sensors)
3. [The Bus Problem & Our 9-Clock Recovery Trick](#3-the-bus-problem--our-9-clock-recovery-trick)
4. [Tier 1: Deterministic Safety Engine (Instant Reflexes)](#4-tier-1-deterministic-safety-engine)
5. [Tier 2: Cross-Sensor Correlator (The 45-Second Coincidence Rule)](#5-tier-2-cross-sensor-correlator)
6. [Tier 3: Personal Baseline Anomaly Engine (The Smart Math)](#6-tier-3-personal-baseline-anomaly-engine)
7. [Word-for-Word Speaking Script (Your Exact Words)](#7-word-for-word-speaking-script)
8. [Patent Claims You Need to Defend](#8-patent-claims-you-need-to-defend)
9. [Tough Mentor Questions & Your Exact Answers](#9-tough-mentor-questions--your-exact-answers)

---

## 1. The Big Picture

### Why does this exist?
Imagine an earthquake, building collapse, or massive forest fire:
- Dust, smoke, extreme heat, and rubble are everywhere.
- Firefighters and search-and-rescue teams are working under extreme physical stress.
- Cellular towers are destroyed or intermittent.
- Responders can fall, inhale smoke, suffer heat stroke, or experience cardiac collapse.

### Why do existing smartwatches (Apple Watch, Garmin) fail here?
1. **They need a smartphone or stable cell network**: If the phone is crushed or cell towers are down, they can't send help.
2. **False Alarm Nightmare**: When a firefighter climbs over rubble or uses a sledgehammer, an Apple Watch thinks they fell or have an abnormal heart rate. If a device cries wolf 10 times, the responder takes it off and throws it away.
3. **They crash or leak memory**: Consumer wearables run complex operating systems with dynamic memory (`malloc`). In life-and-death equipment, a memory crash is unacceptable. Our system uses **zero dynamic memory** (only ~3.2 KB total compile-time static memory).

---

## 2. The Hardware & The 5 Sensors

Our brain is the **ESP32-S3** microcontroller (dual-core 32-bit processor running at 240 MHz with 512 KB internal SRAM).

We have **5 sensors** connected to the ESP32-S3:

| Sensor | Real-World Job | How It Connects | Why It's There |
|---|---|---|---|
| **MAX30102** | Optical Pulse Oximeter | I2C (`0x57`) | Shines Red & Infrared light through skin to measure **Heart Rate** and **Blood Oxygen ($\text{SpO}_2$)**. |
| **MPU6050** | 6-Axis IMU (Motion Sensor) | I2C (`0x68`) | Measures acceleration ($a_x, a_y, a_z$) and rotation. Detects **freefall, violent impact, and immobility**. |
| **MAX30205** | Clinical Skin Thermometer | I2C (`0x48`) | Measures body temperature touching the skin to an accuracy of $\pm 0.1^\circ\text{C}$ (detects hypothermia & fever). |
| **BME280** | Ambient Climate Sensor | I2C (`0x76`) | Measures surrounding air temperature, humidity, and barometric pressure. Used to calculate **Heat Index**. |
| **MQ135** | Gas & Air Quality Sensor | Analog (`GPIO 1`) | Heated semiconductor sensor that detects toxic combustion products: smoke, $CO_2$, benzene, ammonia. |
| **SSD1306** | 0.96" OLED Screen | I2C (`0x3C`) | Displays heart rate, warnings, and system status to the wearer. |

### Sampling Rates (How Fast We Read Them):
- **PPG (MAX30102)**: **80 times per second (80 Hz)** — needs fast optical sampling to see individual heartbeats.
- **IMU (MPU6050)**: **100 times per second (100 Hz)** — needs fast sampling to capture sudden falls and impacts.
- **Skin Temp (MAX30205)**: Once every 20 seconds (body temperature changes very slowly).
- **Climate & Gas (BME280 / MQ135)**: Once every 50 seconds.

---

## 3. The Bus Problem & Our 9-Clock Recovery Trick

### What is I2C?
I2C is a 2-wire communication highway (`SDA` for data, `SCL` for clock). All 4 digital sensors and the OLED screen share this same pair of wires.

### The Real-World Disaster Problem:
If a sensor gets knocked, drops voltage, or gets corrupted by electrical noise, its internal chip can get stuck while driving the `SDA` line LOW. When `SDA` is stuck LOW, the entire highway is blocked. No sensor can talk to the microcontroller! Standard Arduino code freezes forever when this happens.

### Our Solution (`i2c_recovery.cpp`):
We built an autonomous recovery algorithm:
1. If 3 consecutive I2C reads fail, the ESP32 unplugs the I2C driver (`Wire.end()`).
2. It takes manual control of the `SCL` wire and **bit-bangs 9 clock pulses** (5 µs half-period).
   - *Why 9 pulses?* In the I2C standard, a data byte is 8 bits plus 1 ACK bit. Sending 9 pulses forces any stuck slave chip to finish its internal cycle and release the line!
3. It creates a manual **STOP condition** (pulls SDA from LOW to HIGH while SCL is HIGH).
4. Reinitializes `Wire.begin()` cleanly. The device recovers in < 1 millisecond without restarting!

---

## 4. Tier 1: Deterministic Safety Engine

This is our "reflex system." It has **zero delay** and runs on strict, deterministic medical thresholds.

```
Incoming Sensor Data
        │
        ├─► [Optical Quality Gate] ──► Motion > 0.3? ──► SUPPRESS SpO2 ALERTS (Avoid false alarm!)
        │
        ├─► [4-State Fall FSM] ────► Freefall (<0.2g) ──► Impact (>3.0g) ──► Immobile (5s) ──► ALARM!
        │
        └─► [Rothfusz Heat Index] ─► 9-Term Polynomial ──► HI ≥ 40°C: Warning / HI ≥ 54°C: Danger
```

### A. Optical Signal Quality & Motion Gating (`ppg_qualify.cpp`)
* **The Problem:** When you run or move your hand, the optical sensor slides against your skin. The light leaks, and the sensor falsely outputs "SpO2 = 82%" (which looks like severe suffocation).
* **Our Solution:**
  1. *Skin Contact Gating*: If raw Infrared (IR) light amplitude $< 5000$ counts, the sensor isn't touching skin. State = `CONTACT_LOST`. All SpO2 alerts are suppressed.
  2. *Buffer Filling*: When contact returns, it enters `CALIBRATING` for 100 samples before trusting any vitals.
  3. *Motion Artifact Rejection*: We compute the **rolling variance of acceleration** over a 0.5-second window (50 samples at 100 Hz):
     $$\text{Var}(a) = \frac{1}{N}\sum a^2 - \left(\frac{1}{N}\sum a\right)^2$$
     If $\text{Var}(a) > 0.3\,(\text{m/s}^2)^2$, we know the user is moving their hand! The state shifts to `MOTION_ARTIFACT`. The OLED shows *"Keep still for vitals"*, and SpO2 alerts are **blocked**.

### B. 4-State Fall Detection State Machine (`fall_detector.cpp`)
A person falling doesn't just experience high acceleration; they experience a distinct sequence of physics:
1. **`IDLE`**: Normal movement ($1.0\,\text{g}$ gravity).
2. **`FREEFALL_DETECTED`**: Total acceleration drops below **$0.2\,\text{g}$** ($\|a\| = \sqrt{a_x^2 + a_y^2 + a_z^2} < 1.96\,\text{m/s}^2$). Starts a 300 ms timer.
3. **`IMPACT_DETECTED`**: Within 300 ms of freefall, acceleration spikes above **$3.0\,\text{g}$** (the body hitting the ground).
4. **`FALL_CONFIRMED`**: This is our critical filter! Many people drop their bag or jump down stairs (impact) but then keep walking. We require **5 full seconds of sustained immobility** ($\text{Var}(a) < 0.05\,(\text{m/s}^2)^2$). Only if they stay motionless does the alarm sound!
   - *Self-Recovery Safeguard*: If the responder moves continuously for 15 seconds after impact, the FSM drops back to `IDLE`.

### C. NWS Heat Index Engine (`heat_index.cpp`)
Air temperature alone doesn't tell you how dangerous the environment is—humidity stops sweat from evaporating.
We implement the full **National Weather Service (NWS) 9-term Rothfusz polynomial regression**:
$$HI = c_1 + c_2 T + c_3 R + c_4 T R + c_5 T^2 + c_6 R^2 + c_7 T^2 R + c_8 T R^2 + c_9 T^2 R^2$$
- $T$ = Temp in $^\circ\text{F}$, $R$ = Relative Humidity in $\%$.
- Output converted back to Celsius:
  - $\ge 40^\circ\text{C}$: NWS **Heat Stress Warning**.
  - $\ge 54^\circ\text{C}$: NWS **Extreme Heat Danger** (heat stroke imminent).

---

## 5. Tier 2: Cross-Sensor Correlator (`correlator.cpp`)

*Why do we need this?*  
In a disaster, a single sensor might drift or pick up dust. If an alarm fired every time one sensor spiked, rescuers would suffer **alarm fatigue**.

We built a **Dual-Path Correlator**:

```
                       ┌───────────────────────────────┐
                       │       Incoming Metrics        │
                       └───────────────┬───────────────┘
                                       │
                ┌──────────────────────┴──────────────────────┐
                ▼                                             ▼
       [ PATH A: Hard Bypass ]                     [ PATH B: Corroborator ]
  Confirmed Fall  OR  SpO2 < 90%              45-Second Sliding Window (1 Hz)
                │                                             │
                ▼                                             ▼
      INSTANT CRITICAL ALARM!                     Are ≥ 2 of 4 Flags Active?
      (Zero delay, top priority)                  1. Optical: SpO2 90–92%
                                                  2. Heat: Heat Index > 38°C
                                                  3. Gas: MQ135 Δ > 150 counts
                                                  4. Thermal: Skin <30°C or >38.5°C
                                                              │
                                                              ▼
                                                   YES ──► PATH B ALARM!
```

### Path A: Hard Bypass (Zero Latency)
If the victim suffered a **Confirmed Fall** OR their **$\text{SpO}_2 < 90\%$**, we do not wait. This is an immediate, life-threatening emergency. It fires instantly!

### Path B: 45-Second Coincidence Window
For borderline symptoms, we use a 45-second circular buffer updated at 1 Hz ($O(1)$ constant time) tracking 4 flags:
1. **Optical Flag**: $\text{SpO}_2$ is in the borderline danger zone ($90\% \le \text{SpO}_2 \le 92\%$).
2. **Heat Flag**: Heat Index $> 38.0^\circ\text{C}$.
3. **Gas Flag**: MQ135 gas derivative $|\Delta| > 150$ ADC counts from the previous reading. (A slow increase is just weather; a sharp spike of 150 counts means a sudden plume of toxic smoke!).
4. **Thermal Flag**: Skin temperature is outside healthy limits ($< 30.0^\circ\text{C}$ hypothermia or $> 38.5^\circ\text{C}$ fever).

**The Decision Rule**: If **2 or more of these 4 flags** occur within the 45-second window, Path B triggers an alarm! One strange reading is ignored; two independent sensors agreeing triggers the alarm.

---

## 6. Tier 3: Personal Baseline Anomaly Engine (`anomaly_engine.cpp`)

*Why do we need AI / Statistical Anomaly Detection?*  
A trained firefighter might have a resting heart rate of 50 BPM. An older civilian might have a resting heart rate of 80 BPM. A fixed number cannot fit everyone.

We extract a **6-Feature Vector** ($f_0$ to $f_5$) every second:
1. $f_0$: **Heart Rate** (BPM) from optical peak detection.
2. $f_1$: **RMSSD** (Root Mean Square of Successive Differences) — measures Heart Rate Variability (HRV), which detects physiological stress and exhaustion.
3. $f_2$: **$\text{SpO}_2$** (Blood oxygen percentage).
4. $f_3$: **Skin Temperature** ($^\circ\text{C}$).
5. $f_4$: **Skin Temperature Slope** ($^\circ\text{C}/\text{min}$ across consecutive readings).
6. $f_5$: **Accelerometer Variance** (physical body movement).

### The Lifecycle:
1. **`DORMANT` State (First 60 Minutes)**:
   - When the device is first turned on, it doesn't sound any anomaly alarms.
   - It quietly records the user's baseline: tracks running sum $\sum f_i$ and sum of squares $\sum f_i^2$.
   - At 60 minutes, it calculates the personal mean ($\mu_i$) and standard deviation ($\sigma_i$) for all 6 features.
   - It calibrates the anomaly threshold:
     $$\theta = \mu_{\text{score}} + 3\sigma_{\text{score}}$$
     (3-sigma statistical confidence: only deviations exceeding 99.7% of normal baseline will trigger).
2. **`ACTIVE` State**:
   - Computes the normalized **Z-score squared distance**:
     $$\text{Score} = \frac{1}{N}\sum_{i=1}^N \left(\frac{f_i - \mu_i}{\sigma_i}\right)^2$$
3. **The 5-Minute Motion Cooldown Safeguard**:
   - If the user was just running or hauling equipment (accelerometer variance $> 2.0$), their heart rate will naturally be high.
   - The engine detects this high motion and **dampens the anomaly score by 50% for 5 minutes** during recovery, completely preventing false alarms from normal exercise!
4. **Missing-Feature Substitution**:
   - If up to 2 features are lost (e.g. optical sensor slips off skin), it automatically substitutes the user's learned baseline mean $\mu_i$, allowing the algorithm to keep functioning without crashing.

---

## 7. Word-for-Word Speaking Script

> **Tip:** Read this aloud 2–3 times. Speak clearly, at a conversational pace.

### [Slide 1 / Intro]
"Good afternoon. I am presenting the first half of the **Disaster Health Companion**, focusing on our wearable sensor architecture and multi-tier physiological safety engine.

In a disaster—such as an earthquake or structural collapse—first responders face lethal hazards: toxic gas, extreme heat, building falls, and physical exhaustion. Commercial smartwatches fail here because they rely on connected smartphones, generate high false alarms during heavy physical exertion, and suffer from operating system crashes. Our bare-metal ESP32-S3 firmware uses strictly **zero dynamic heap allocation**, running entirely within 3.2 KB of static memory."

### [Slide 2 / Sensors & I2C Recovery]
"Our hardware integrates five sensors: optical PPG for heart rate and SpO2, a 6-axis IMU for fall detection, clinical skin temperature accurate to 0.1 degrees Celsius, an environmental climate sensor for heat index, and an analog MQ135 for toxic smoke.

Because sensor wires can glitch in harsh environments, we built an autonomous I2C bus recovery routine. If three consecutive sensor reads fail, our code detaches the I2C peripheral, bit-bangs 9 clock pulses on the SCL line to force any stuck sensor to release the bus, generates a manual STOP condition, and reinitializes. This unfreezes the bus in under one millisecond without restarting the microcontroller."

### [Slide 3 / Tier 1 Safety: Motion Gating & Fall FSM]
"Our safety logic operates in three hierarchical tiers. Tier 1 is our deterministic reflex engine.

Optical PPG sensors are notoriously vulnerable to motion artifacts—moving your hand can cause false low-SpO2 readings. We solve this by continuously calculating rolling acceleration variance. If motion variance exceeds 0.3, the system shifts into `MOTION_ARTIFACT` state, displays 'Keep still for vitals', and actively suppresses SpO2 alert generation to prevent false alarms.

For fall detection, we implemented an orientation-invariant 4-state finite state machine. It detects freefall below 0.2g, verifies high-g impact above 3.0g within 300 milliseconds, and strictly requires five full seconds of immobility before confirming a fall. If the wearer recovers and walks within 15 seconds, the alarm is automatically cancelled."

### [Slide 4 / Tier 2 Correlator & Tier 3 Anomaly Engine]
"To prevent alarm fatigue from single-sensor anomalies, Tier 2 is our Cross-Sensor Correlator. Path A is an immediate hard bypass for confirmed falls or SpO2 dropping below 90%. Path B uses a 45-second sliding window that tracks four physiological flags: borderline SpO2, heat index above 38 degrees, thermal limits, and a sharp gas derivative above 150 counts. Crucially, it requires coincidence—at least two flags within 45 seconds—before triggering an alert.

Finally, Tier 3 is our Unsupervised Anomaly Engine. It personalizes thresholds by collecting a 60-minute baseline across six physiological features. It computes distance in z-score space and sets the threshold at mu plus 3-sigma. Uniquely, if the responder engaged in heavy physical labor, the system automatically dampens the anomaly score by 50% for five minutes to account for post-exercise recovery.

Now, my partner will present how these emergency alerts are reliably transmitted through collapsed communication networks."

---

## 8. Patent Claims You Need to Defend

When the mentor asks what parts of your half are patentable, highlight these two:

### Claim Concept 1: Motion-Gated Optical Quality State Machine
* **Novelty:** Dynamically coupling IMU-derived acceleration variance ($\text{Var}(a) > 0.3$) to optical photodiode threshold gating, which selectively suppresses oxygen desaturation alerts while preserving heart rate interval buffers and auto-triggering calibration resets upon contact restoration.

### Claim Concept 2: Hierarchical Dual-Path Cross-Sensor Temporal Coincidence Engine
* **Novelty:** The combination of a zero-latency hard bypass for unambiguous life threats with an $O(1)$ 45-second sliding coincidence window requiring $\ge 2$ cross-domain flags (gas derivative, ambient heat index, clinical skin temp, borderline SpO2), combined with an unsupervised z-score anomaly detector featuring post-exercise motion-variance score dampening.

---

## 9. Tough Mentor Questions & Your Exact Answers

### Q1: "Why did you choose a 5-second immobility window for fall detection instead of 2 seconds or 10 seconds?"
> **Your Answer:**  
> "Clinical fall detection literature shows that 2 seconds is too short—people often pause for 2 seconds after stumbling or sitting down quickly. Conversely, 10 seconds creates an unnecessary delay when dispatching rescue teams. 5 seconds represents the sweet spot in clinical trauma research for identifying post-impact disorientation while rejecting voluntary recovery pauses."

### Q2: "Why use a statistical z-score instead of an ML model like a Convolutional Neural Network or Random Forest?"
> **Your Answer:**  
> "Standard ML models require floating-point matrix libraries, large flash memory, and dynamic memory allocation that can fragment the heap and crash in real-time embedded safety systems. Furthermore, an offline-trained model cannot adapt to an individual's personal resting physiology. Our z-score distance runs in $O(N)$ constant time, consumes only ~96 bytes of SRAM, trains directly on the wearer's personal resting baseline in 60 minutes, and allows deterministic safeguards like our 5-minute exercise cooldown dampening."

### Q3: "What happens if a sensor breaks completely during a mission?"
> **Your Answer:**  
> "In Tier 1, each sensor feed is decoupled. If the PPG fails, fall detection and heat index remain fully active. In Tier 2, the correlator tracks flags independently; if one sensor is invalid, it simply cannot raise its flag, but the remaining sensors can still trigger. In Tier 3, our anomaly engine features missing-feature substitution: if up to two features are unavailable, it substitutes the wearer's learned baseline mean ($\mu_i$), allowing the engine to continue monitoring without mathematical singularities or crashes."
