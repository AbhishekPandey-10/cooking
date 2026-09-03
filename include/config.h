#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
//  Hardware Pin Assignments
// ============================================================================
#define I2C_SDA_PIN           8
#define I2C_SCL_PIN           9
#define I2C_CLOCK_HZ          400000   // 400 kHz Fast-Mode

#define MQ135_ANALOG_PIN      1        // ADC1_CH0 (GPIO 1)

// ============================================================================
//  I2C Device Addresses
// ============================================================================
#define MAX30102_ADDR         0x57
#define MPU6050_ADDR          0x68
#define MAX30205_ADDR         0x48
#define BME280_ADDR           0x76
#define SSD1306_ADDR          0x3C

// ============================================================================
//  Sampling Intervals & Rates
// ============================================================================

// High-rate sensors (hardware-timer driven, microseconds)
#define MAX30102_SAMPLE_US    12500    // 80 Hz  (range: 50–100 Hz)
#define MPU6050_SAMPLE_US     10000    // 100 Hz

// Low-rate sensors (software-timer driven, milliseconds)
#define MAX30205_SAMPLE_MS    20000    // every 20 s
#define ENV_SAMPLE_MS         50000    // BME280 + MQ135 every 50 s

// ============================================================================
//  MQ135 Gas Sensor
// ============================================================================
#define MQ135_WARMUP_MS       90000    // 90-second heater stabilization blackout
#define MQ135_ADC_RESOLUTION  12       // 12-bit ADC on ESP32-S3 (0–4095)

// ============================================================================
//  I2C Recovery
// ============================================================================
#define I2C_RECOVERY_SCL_TOGGLES  9
#define I2C_RECOVERY_DELAY_US     5    // half-period of bit-bang clock

// ============================================================================
//  Display
// ============================================================================
#define OLED_WIDTH            128
#define OLED_HEIGHT           64
#define OLED_RESET            -1       // no HW reset pin

#endif // CONFIG_H
