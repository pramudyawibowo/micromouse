#include "imu.h"
#include "config.h"

bool IMU::available = false;
uint8_t IMU::activeAddr = MPU6050_I2C_ADDR;
float IMU::yaw = 0.0f;
float IMU::gyroZ = 0.0f;
float IMU::gyroZOffset = 0.0f;
uint32_t IMU::lastUpdateUs = 0;

void IMU::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(activeAddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t IMU::readRegister(uint8_t reg) {
    Wire.beginTransmission(activeAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)activeAddr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool IMU::init() {
    Wire.setSCL(PIN_I2C_SCL);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.begin();
    Wire.setClock(100000); // 100kHz standard mode untuk kestabilan probing awal

    // Cek respons I2C di alamat 0x68 (AD0=GND) atau 0x69 (AD0=VCC)
    activeAddr = MPU6050_I2C_ADDR;
    Wire.beginTransmission(activeAddr);
    if (Wire.endTransmission() != 0) {
        activeAddr = MPU6050_I2C_ADDR_ALT; // Coba 0x69
        Wire.beginTransmission(activeAddr);
        if (Wire.endTransmission() != 0) {
            available = false;
            return false; // Hardware MPU tidak merespons di bus I2C
        }
    }

    // Bangunkan MPU6050 / MPU6500 dari sleep mode: tulis 0 ke PWR_MGMT_1 (0x6B)
    writeRegister(0x6B, 0x00);
    delay(20);

    // Konfigurasi Clock source Auto-Select (PLL Gyro Z lebih stabil daripada internal 8MHz RC)
    writeRegister(0x6B, 0x01);
    delay(10);

    // DLPF Config: 0x1A -> 0x03 (DLPF_CFG = 3: ~42Hz bandwidth)
    writeRegister(0x1A, 0x03);

    // Gyro Config: 0x1B -> 0x08 (±500 dps full scale -> sensitivity 65.5 LSB/(deg/s))
    writeRegister(0x1B, 0x08);

    Wire.setClock(400000); // Naikkan ke 400kHz Fast I2C untuk pembacaan real-time

    available = true;
    lastUpdateUs = micros();
    calibrate();
    return true;
}

void IMU::calibrate() {
    if (!available) return;

    int32_t sum = 0;
    const int samples = 150;
    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(activeAddr);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)activeAddr, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t gzRaw = (Wire.read() << 8) | Wire.read();
            sum += gzRaw;
        }
        delay(2);
    }
    gyroZOffset = (float)sum / (float)samples;
    yaw = 0.0f;
    lastUpdateUs = micros();
}

void IMU::update() {
    if (!available) return;

    uint32_t now = micros();
    // Hitung dt dengan benar bahkan saat micros() overflow (~70 menit)
    // Unsigned subtraction secara otomatis menangani wraparound pada uint32_t
    float dt = (float)(now - lastUpdateUs) * 1e-6f;
    lastUpdateUs = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f; // Protect against extreme values

    Wire.beginTransmission(activeAddr);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)activeAddr, (uint8_t)2);

    if (Wire.available() >= 2) {
        int16_t gzRaw = (Wire.read() << 8) | Wire.read();
        // ±500 dps -> 65.5 LSB per deg/s
        float rawRate = ((float)gzRaw - gyroZOffset) / 65.5f;

        // Deadband filter kecil untuk redam noise getaran statis
        if (fabs(rawRate) < 0.12f) {
            rawRate = 0.0f;
        }

        gyroZ = rawRate;
        yaw += gyroZ * dt;

        // Wrap yaw to -180..+180
        while (yaw > 180.0f) yaw -= 360.0f;
        while (yaw < -180.0f) yaw += 360.0f;
    }
}

void IMU::resetYaw() {
    yaw = 0.0f;
    lastUpdateUs = micros();
}

float IMU::getYaw() {
    return yaw;
}

float IMU::getGyroZ() {
    return gyroZ;
}

bool IMU::isAvailable() {
    return available;
}
