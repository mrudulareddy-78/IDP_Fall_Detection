// ============================================================
// mpu6500.h — MPU6500 IMU Driver
// ============================================================
#pragma once
#include <Wire.h>

#define MPU_ADDR       0x68   // AD0 tied to GND

#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT   0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG     0x1A
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_ACCEL_OUT  0x3B

// ±2g  → 16384 LSB/g
// ±250 → 131   LSB/dps
#define ACCEL_SENS  16384.0f
#define GYRO_SENS   131.0f

static void _mpu_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

static bool mpu_init() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 1, true);
    uint8_t id = Wire.read();
    Serial.printf("  WHO_AM_I = 0x%02X ", id);

    // MPU6500 = 0x70, variant = 0x71, MPU6050 = 0x68
    if (id != 0x70 && id != 0x71 && id != 0x68) {
        Serial.println("← NOT recognised");
        return false;
    }
    Serial.println("← OK");

    _mpu_write(REG_PWR_MGMT,   0x00);  // wake up
    delay(100);
    // 1000 / (1 + 9) = 100 Hz
    _mpu_write(REG_SMPLRT_DIV, 9);
    // DLPF 44 Hz bandwidth
    _mpu_write(REG_CONFIG,     0x03);
    // Gyro ±250 dps
    _mpu_write(REG_GYRO_CFG,   0x00);
    // Accel ±2g
    _mpu_write(REG_ACCEL_CFG,  0x00);
    delay(100);
    return true;
}

static void mpu_read(float* ax, float* ay, float* az,
                     float* gx, float* gy, float* gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_OUT);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t rax = (Wire.read()<<8)|Wire.read();
    int16_t ray = (Wire.read()<<8)|Wire.read();
    int16_t raz = (Wire.read()<<8)|Wire.read();
    Wire.read(); Wire.read();  // skip temp
    int16_t rgx = (Wire.read()<<8)|Wire.read();
    int16_t rgy = (Wire.read()<<8)|Wire.read();
    int16_t rgz = (Wire.read()<<8)|Wire.read();

    *ax = rax / ACCEL_SENS;
    *ay = ray / ACCEL_SENS;
    *az = raz / ACCEL_SENS;
    *gx = rgx / GYRO_SENS;
    *gy = rgy / GYRO_SENS;
    *gz = rgz / GYRO_SENS;
}
