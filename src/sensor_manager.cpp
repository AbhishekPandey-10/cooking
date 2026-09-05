#include "sensor_manager.h"
#include "config.h"
#include "i2c_recovery.h"

#include <Arduino.h>
#include <Wire.h>

// SparkFun MAX3010x library defines I2C_BUFFER_LENGTH 32 without an #ifndef guard,
// conflicting with ESP32 Wire.h (128). Undefine it first to avoid redefinition warning.
#undef I2C_BUFFER_LENGTH
#include <MAX30105.h>              // SparkFun MAX3010x library
#include <MPU6050.h>               // ElectronicCats MPU6050
#include <Protocentral_MAX30205.h> // Protocentral MAX30205
#include <Adafruit_BME280.h>       // Adafruit BME280
#include <Adafruit_SSD1306.h>      // Adafruit SSD1306

// ============================================================================
//  Static driver instances (file-scoped, not visible outside this TU)
// ============================================================================
static MAX30105         s_ppg;
static MPU6050          s_imu;
static MAX30205         s_temp;
static Adafruit_BME280  s_bme;
static Adafruit_SSD1306 s_oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// Track which sensors are actually online
static uint8_t s_live_mask = 0;

// MQ135 warm-up tracking (boot timestamp captured in sensors_init)
static uint32_t s_boot_ms = 0;

// ============================================================================
//  Helper: I2C device probe
// ============================================================================
static bool probe_i2c(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

// ============================================================================
//  Helper: wrapped I2C read with automatic recovery
//  Returns true if the transaction succeeded (possibly after one recovery).
// ============================================================================
static bool safe_i2c_transaction(uint8_t addr)
{
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) return true;

    // Attempt recovery once
    Serial.printf("[SENSOR] I2C error %d on 0x%02X — attempting recovery\r\n",
                  err, addr);
    if (i2c_bus_recover()) {
        Wire.beginTransmission(addr);
        return (Wire.endTransmission() == 0);
    }
    return false;
}

// ============================================================================
//  Initialisation
// ============================================================================
uint8_t sensors_init()
{
    s_boot_ms = millis();
    s_live_mask = 0;

    // ---- I2C bus -----------------------------------------------------------
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
    Wire.setTimeOut(50);  // 50 ms transaction timeout
    Serial.println("[SENSOR] I2C bus initialised");

    // ---- MAX30102 (PPG / SpO2) ---------------------------------------------
    if (probe_i2c(MAX30102_ADDR)) {
        if (s_ppg.begin(Wire, I2C_SPEED_FAST, MAX30102_ADDR)) {
            // Configure for PPG:  LED power, sample rate, pulse width
            s_ppg.setup(0x1F,   // LED brightness (power level)
                        4,      // sample average
                        2,      // LED mode: Red + IR
                        400,    // sample rate (sensor internal)
                        411,    // pulse width (18-bit resolution)
                        4096);  // ADC range
            s_live_mask |= SENSOR_MAX30102;
            Serial.println("[SENSOR] MAX30102 OK");
        } else {
            Serial.println("[SENSOR] MAX30102 begin() failed");
        }
    } else {
        Serial.println("[SENSOR] MAX30102 not found at 0x57");
    }

    // ---- MPU6050 (IMU) -----------------------------------------------------
    if (probe_i2c(MPU6050_ADDR)) {
        s_imu.initialize();
        if (s_imu.testConnection()) {
            s_imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // ±4 g
            s_imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // ±500 °/s
            s_imu.setDLPFMode(MPU6050_DLPF_BW_42);             // 42 Hz LPF
            s_live_mask |= SENSOR_MPU6050;
            Serial.println("[SENSOR] MPU6050  OK");
        } else {
            Serial.println("[SENSOR] MPU6050 connection test failed");
        }
    } else {
        Serial.println("[SENSOR] MPU6050  not found at 0x68");
    }

    // ---- MAX30205 (skin temp) ----------------------------------------------
    if (probe_i2c(MAX30205_ADDR)) {
        s_temp.begin();
        s_live_mask |= SENSOR_MAX30205;
        Serial.println("[SENSOR] MAX30205 OK");
    } else {
        Serial.println("[SENSOR] MAX30205 not found at 0x48");
    }

    // ---- BME280 (climate) --------------------------------------------------
    if (probe_i2c(BME280_ADDR)) {
        if (s_bme.begin(BME280_ADDR, &Wire)) {
            // Weather-monitoring mode: low power, infrequent reads
            s_bme.setSampling(Adafruit_BME280::MODE_FORCED,
                              Adafruit_BME280::SAMPLING_X1,   // temp
                              Adafruit_BME280::SAMPLING_X1,   // pressure
                              Adafruit_BME280::SAMPLING_X1,   // humidity
                              Adafruit_BME280::FILTER_OFF,
                              Adafruit_BME280::STANDBY_MS_1000);
            s_live_mask |= SENSOR_BME280;
            Serial.println("[SENSOR] BME280   OK");
        } else {
            Serial.println("[SENSOR] BME280 begin() failed");
        }
    } else {
        Serial.println("[SENSOR] BME280   not found at 0x76");
    }

    // ---- MQ135 (analog gas sensor — always on) -----------------------------
    //  The heater is powered continuously via Vcc; we just configure the ADC.
    analogReadResolution(MQ135_ADC_RESOLUTION);
    pinMode(MQ135_ANALOG_PIN, INPUT);
    Serial.printf("[SENSOR] MQ135 ADC ready — blackout for %d s\r\n",
                  MQ135_WARMUP_MS / 1000);

    // ---- SSD1306 OLED display ----------------------------------------------
    if (probe_i2c(SSD1306_ADDR)) {
        if (s_oled.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDR)) {
            s_oled.clearDisplay();
            s_oled.setTextSize(1);
            s_oled.setTextColor(SSD1306_WHITE);
            s_oled.setCursor(0, 0);
            s_oled.println("Disaster Health Mon");
            s_oled.println("Initialising...");
            s_oled.display();
            s_live_mask |= SENSOR_SSD1306;
            Serial.println("[SENSOR] SSD1306  OK");
        } else {
            Serial.println("[SENSOR] SSD1306 begin() failed");
        }
    } else {
        Serial.println("[SENSOR] SSD1306  not found at 0x3C");
    }

    Serial.printf("[SENSOR] Init complete — mask 0x%02X (%d/5 devices)\n",
                  s_live_mask, __builtin_popcount(s_live_mask));
    return s_live_mask;
}

// ============================================================================
//  Individual Read Functions
// ============================================================================

bool read_ppg(PpgReading &out)
{
    if (!(s_live_mask & SENSOR_MAX30102)) return false;
    if (!safe_i2c_transaction(MAX30102_ADDR)) return false;

    out.ir  = s_ppg.getIR();
    out.red = s_ppg.getRed();
    out.timestamp_ms = millis();
    return true;
}

bool read_imu(ImuReading &out)
{
    if (!(s_live_mask & SENSOR_MPU6050)) return false;
    if (!safe_i2c_transaction(MPU6050_ADDR)) return false;

    int16_t ax, ay, az, gx, gy, gz;
    s_imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw → physical units (±4 g = 8192 LSB/g, ±500 °/s = 65.5 LSB/°/s)
    constexpr float ACCEL_SCALE = 9.80665f / 8192.0f;
    constexpr float GYRO_SCALE  = 1.0f / 65.5f;

    out.ax = ax * ACCEL_SCALE;
    out.ay = ay * ACCEL_SCALE;
    out.az = az * ACCEL_SCALE;
    out.gx = gx * GYRO_SCALE;
    out.gy = gy * GYRO_SCALE;
    out.gz = gz * GYRO_SCALE;
    out.timestamp_ms = millis();
    return true;
}

bool read_skin_temp(TempReading &out)
{
    if (!(s_live_mask & SENSOR_MAX30205)) return false;
    if (!safe_i2c_transaction(MAX30205_ADDR)) return false;

    out.skin_temp_c  = s_temp.getTemperature();
    out.timestamp_ms = millis();
    return true;
}

bool read_environment(EnvReading &out)
{
    bool ok = false;

    // ---- BME280 (forced-mode: trigger a reading, then read) ----------------
    if (s_live_mask & SENSOR_BME280) {
        if (safe_i2c_transaction(BME280_ADDR)) {
            s_bme.takeForcedMeasurement();
            out.bme_temp_c   = s_bme.readTemperature();
            out.bme_humidity = s_bme.readHumidity();
            out.bme_pressure = s_bme.readPressure() / 100.0f;  // Pa → hPa
            ok = true;
        }
    }

    // ---- MQ135 (analog — always available, but blackout-gated) -------------
    out.mq135_raw   = analogRead(MQ135_ANALOG_PIN);
    out.mq135_valid = (millis() - s_boot_ms) >= MQ135_WARMUP_MS;

    out.timestamp_ms = millis();
    return ok;  // reflects BME280 success; MQ135 always fills raw value
}

// ============================================================================
//  Display Update
// ============================================================================
void display_update(const SensorFrame &frame)
{
    if (!(s_live_mask & SENSOR_SSD1306)) return;

    s_oled.clearDisplay();
    s_oled.setCursor(0, 0);

    // Line 1: PPG
    s_oled.printf("IR:%lu R:%lu\n",
                  (unsigned long)frame.ppg.ir,
                  (unsigned long)frame.ppg.red);

    // Line 2: IMU accel magnitude
    float amag = sqrtf(frame.imu.ax * frame.imu.ax +
                       frame.imu.ay * frame.imu.ay +
                       frame.imu.az * frame.imu.az);
    s_oled.printf("Accel: %.1f m/s2\n", amag);

    // Line 3: Skin temperature
    s_oled.printf("Skin: %.1f C\n", frame.temp.skin_temp_c);

    // Line 4: Environment
    s_oled.printf("T:%.1fC H:%.0f%%\n",
                  frame.env.bme_temp_c, frame.env.bme_humidity);

    // Line 5: Pressure
    s_oled.printf("P:%.0f hPa\n", frame.env.bme_pressure);

    // Line 6: Gas
    if (frame.env.mq135_valid) {
        s_oled.printf("Gas: %d", frame.env.mq135_raw);
    } else {
        s_oled.printf("Gas: warming up");
    }

    s_oled.display();
}
