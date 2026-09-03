#include "i2c_recovery.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2c.h>    // ESP-IDF driver — needed to detach peripheral

// ============================================================================
//  I2C Bus Recovery Implementation
//
//  Sequence:
//    1. Detach the I2C peripheral so GPIOs are free for bit-banging.
//    2. Configure SCL as open-drain output, SDA as open-drain input.
//    3. Toggle SCL 9 times — a stuck slave will eventually release SDA.
//    4. Generate a STOP condition: drive SDA LOW, raise SCL, then raise SDA.
//    5. Re-init Wire so the I2C peripheral is re-attached.
// ============================================================================

bool i2c_bus_recover()
{
    Serial.println("[I2C] Bus recovery started");

    // ---- Step 1: Detach the I2C peripheral from the GPIO matrix ------------
    Wire.end();

    // ---- Step 2: Configure pins for manual control -------------------------
    //  Open-drain + external pull-up is the I2C electrical standard.
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);

    // Ensure SCL starts HIGH (idle)
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(I2C_RECOVERY_DELAY_US);

    // ---- Step 3: Bit-bang SCL 9 times --------------------------------------
    //  If a slave is holding SDA low mid-byte, clocking SCL lets it finish
    //  shifting out remaining bits + the ACK, after which it releases SDA.
    for (int i = 0; i < I2C_RECOVERY_SCL_TOGGLES; i++) {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(I2C_RECOVERY_DELAY_US);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(I2C_RECOVERY_DELAY_US);

        // If SDA went high the slave has released — we can stop early
        if (digitalRead(I2C_SDA_PIN) == HIGH) {
            Serial.printf("[I2C] SDA released after %d clock(s)\n", i + 1);
            break;
        }
    }

    // ---- Step 4: Manual STOP condition  (SDA low→high while SCL is HIGH) ---
    //  SDA LOW
    pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(I2C_RECOVERY_DELAY_US);

    //  SCL HIGH  (should already be high, but make sure)
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(I2C_RECOVERY_DELAY_US);

    //  SDA HIGH  — this rising edge while SCL is high IS the STOP condition
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(I2C_RECOVERY_DELAY_US);

    // ---- Step 5: Re-initialise the Wire peripheral -------------------------
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

    // ---- Verify: both lines should idle HIGH -------------------------------
    bool sda_ok = digitalRead(I2C_SDA_PIN) == HIGH;
    bool scl_ok = digitalRead(I2C_SCL_PIN) == HIGH;

    if (sda_ok && scl_ok) {
        Serial.println("[I2C] Bus recovery successful — lines idle HIGH");
    } else {
        Serial.printf("[I2C] Bus recovery FAILED  SDA=%d  SCL=%d\n",
                      digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));
    }

    return sda_ok && scl_ok;
}
