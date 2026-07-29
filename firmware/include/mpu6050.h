#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class MPU6050 {
public:
    MPU6050() : gyro_offset_x(0), gyro_offset_y(0), gyro_offset_z(0),
                pitch(0), roll(0), yaw(0), gyro_z_deg_s(0) {}

    bool init() {
        Wire.begin();
        Wire.setClock(400000); // 400kHz fast mode

        // Wake up
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x6B); // PWR_MGMT_1
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) return false;

        // Configure Gyro (±500°/s) -> 65.5 LSB/(°/s)
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x1B); // GYRO_CONFIG
        Wire.write(0x08);
        Wire.endTransmission();

        // Configure Accel (±8g) -> 4096 LSB/g
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x1C); // ACCEL_CONFIG
        Wire.write(0x10);
        Wire.endTransmission();
        
        // Digital Low Pass Filter (42Hz)
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x1A); // CONFIG
        Wire.write(0x03);
        Wire.endTransmission();

        return true;
    }

    void calibrate(int samples = 500) {
        long sum_gx = 0, sum_gy = 0, sum_gz = 0;
        for (int i = 0; i < samples; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            readRaw(ax, ay, az, gx, gy, gz);
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            delay(3);
        }
        gyro_offset_x = (float)sum_gx / samples;
        gyro_offset_y = (float)sum_gy / samples;
        gyro_offset_z = (float)sum_gz / samples;
    }

    void read(float dt) {
        int16_t ax, ay, az, gx, gy, gz;
        readRaw(ax, ay, az, gx, gy, gz);

        // Convert raw to deg/s
        float gx_deg = (gx - gyro_offset_x) / 65.5f;
        float gy_deg = (gy - gyro_offset_y) / 65.5f;
        gyro_z_deg_s = (gz - gyro_offset_z) / 65.5f;

        // Convert accel to g's
        float ax_g = ax / 4096.0f;
        float ay_g = ay / 4096.0f;
        float az_g = az / 4096.0f;

        // Accelerometer angles
        float accel_pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0f / PI;
        float accel_roll = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;

        // Complementary filter
        const float alpha = 0.98f;
        pitch = alpha * (pitch + gx_deg * dt) + (1.0f - alpha) * accel_pitch;
        roll = alpha * (roll + gy_deg * dt) + (1.0f - alpha) * accel_roll;
    }

    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }
    float getYawRate() const { return gyro_z_deg_s; }

private:
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    float pitch, roll, yaw;
    float gyro_z_deg_s;

    void readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x3B); // ACCEL_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14, (uint8_t)true);
        
        if (Wire.available() == 14) {
            ax = Wire.read() << 8 | Wire.read();
            ay = Wire.read() << 8 | Wire.read();
            az = Wire.read() << 8 | Wire.read();
            Wire.read(); Wire.read(); // Skip temp
            gx = Wire.read() << 8 | Wire.read();
            gy = Wire.read() << 8 | Wire.read();
            gz = Wire.read() << 8 | Wire.read();
        }
    }
};
