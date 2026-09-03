#ifndef I2C_RECOVERY_H
#define I2C_RECOVERY_H

// ============================================================================
//  I2C Bus Recovery
//
//  If an I2C transaction times out (e.g. a slave holds SDA low), this routine:
//    1. Detaches the ESP32-S3 I2C peripheral from the GPIO matrix.
//    2. Bit-bangs SCL 9 times so a stuck slave can complete its byte/ACK.
//    3. Generates a manual STOP condition (SDA low→high while SCL is high).
//    4. Re-initialises the Wire peripheral via Wire.begin().
//
//  Call this from any sensor read that returns an I2C error.
// ============================================================================

/// Attempt a full I2C bus recovery.  Returns true if the bus looks healthy
/// after recovery (SDA and SCL both read HIGH).
bool i2c_bus_recover();

#endif // I2C_RECOVERY_H
