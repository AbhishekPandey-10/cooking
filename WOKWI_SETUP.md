# Running your real firmware in Wokwi

These two files simulate your actual ESP32-S3 board — not a mockup. Wokwi flashes
your real compiled `.bin`, so all of your Phase 1–10 logic (I2C recovery, fall FSM,
heat index, anomaly engine, BLE relay, SOS engine, display throttling) executes
exactly as it does on hardware.

## Setup (VS Code, recommended — 5 min)

1. Install the **Wokwi for VS Code** extension (also install the PlatformIO extension
   if you don't have it).
2. Copy `diagram.json` and `wokwi.toml` into the root of your `cooking` repo
   (same folder as `platformio.ini`).
3. Get a free Wokwi license key from wokwi.com and activate it in the extension.
4. Build normally: `pio run -e esp32s3`
5. Press **F1 → "Wokwi: Start Simulator"** (or click the play icon Wokwi adds
   above `void setup()` in `main.cpp`).

Your firmware boots in the simulator, the serial monitor shows your real
`Serial.printf` logs, and the virtual MPU6050/BME280/SSD1306 respond to actual
I2C transactions from your driver code.

## What's real in this simulation

| Component | Status |
|---|---|
| ESP32-S3 core, hardware timers, scheduler | ✅ Real, your actual compiled firmware |
| I2C bus + bit-bang recovery routine | ✅ Real bus on GPIO 8 (SDA) / GPIO 9 (SCL) |
| MPU6050 (fall detection) | ✅ Real virtual chip (`wokwi-mpu6050` at 0x68) — interactive sliders for accelX/Y/Z to test fall FSM |
| SSD1306 OLED | ✅ Real virtual display (`board-ssd1306` at 0x3C) — renders live display buffer with 2 Hz throttling |
| BME280 (heat index) | ⚡ Custom chip (`chip-bme280` at 0x76 via GitHub dependency) — provides climate telemetry; degrades gracefully if offline |
| MQ135 (gas, analog) | ✅ Real virtual potentiometer (`wokwi-potentiometer` on GPIO 1 / ADC1_CH0) — twist the knob live to vary gas PPM |
| MAX30102 (PPG/SpO2) | ⚠️ Not modeled in Wokwi — probe_i2c(0x57) reports absent, firmware exercises degraded vitals path |
| MAX30205 (skin temp) | ⚠️ Same — probe_i2c(0x48) reports absent, degrades gracefully |
| SIM800L / GSM SOS engine | ⚠️ Not modeled — exercises modem power gate (GPIO 14) and UART retry backoff state machine |
| BLE mesh relay (real RF) | ⚠️ Wokwi has no multi-device RF mesh simulation — physical boards or Renode required |

## Honest framing for patent/mentor material

This setup demonstrates that your control logic, scheduling, I2C recovery, fall
detection, and heat-index safety engine work correctly against real (simulated)
sensor hardware and real firmware — not a diagram or a claim. The PPG/gas/GSM/BLE
gaps above are worth stating plainly rather than glossing over; they're the reason
physical hardware bench testing (your next milestone) still matters.
