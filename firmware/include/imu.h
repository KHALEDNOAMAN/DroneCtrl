#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "attitude_ekf.h"

/**
 * MPU6050 driver and attitude estimation.
 *
 * This is deliberately a thin layer: it owns the I2C transactions, the scaling
 * from raw counts to physical units, and the gyro calibration, and then hands
 * the numbers to AttitudeEKF. Every line of estimation maths lives in
 * attitude_ekf.h, which includes no Arduino header and is therefore covered by
 * the host tests in firmware/test and flown by the harness in firmware/sil.
 *
 * Replaces the previous complementary filter. Two things were wrong with it
 * beyond the filter choice itself, and both are worth naming because the
 * second hid the first:
 *
 *   1. Roll and pitch were transposed. The angle computed from atan2(-ax, ...)
 *      is pitch, and it was being stored as roll; the gyro integration had the
 *      same swap, using the roll rate to propagate pitch.
 *   2. The mixer's roll sign was inverted relative to the standard convention.
 *
 * Either one alone would have made the airframe uncontrollable. Together they
 * cancelled, which is why the code looked like it worked. Both are fixed here
 * and in mixer_math.h, and there is now a unit test on each so they cannot
 * drift back independently.
 *
 * Axis convention, stated once and used everywhere: x forward, y right, z up
 * through the top of the airframe. Positive roll is right side down, positive
 * pitch is nose up. That matches the accelerometer at rest reading +1 g on z.
 */
class IMU {
public:
    bool init() {
        Wire.begin();
        Wire.setClock(400000); // 400 kHz fast mode

        // Wake up
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x6B); // PWR_MGMT_1
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) return false;

        // Gyro +-500 deg/s -> 65.5 LSB per deg/s
        writeReg(0x1B, 0x08);
        // Accel +-8 g -> 4096 LSB per g
        writeReg(0x1C, 0x10);
        // On-chip low pass at 42 Hz. Well above the ~6 rad/s attitude loop and
        // below the frame's own vibration, so it removes prop noise without
        // adding phase lag where the controller would feel it.
        writeReg(0x1A, 0x03);

        return true;
    }

    /**
     * Measure and remove the gyro's zero offset. Must be run with the airframe
     * still. The EKF estimates bias in flight as well, but starting from a
     * measured offset means it begins near the answer rather than converging
     * through the first seconds of a flight.
     */
    void calibrate(int samples = 500) {
        long sum_gx = 0, sum_gy = 0, sum_gz = 0;
        long sum_ax = 0, sum_ay = 0, sum_az = 0;
        for (int i = 0; i < samples; ++i) {
            int16_t ax, ay, az, gx, gy, gz;
            readRaw(ax, ay, az, gx, gy, gz);
            sum_gx += gx; sum_gy += gy; sum_gz += gz;
            sum_ax += ax; sum_ay += ay; sum_az += az;
            delay(3);
        }
        gyro_offset_x_ = (float)sum_gx / samples;
        gyro_offset_y_ = (float)sum_gy / samples;
        gyro_offset_z_ = (float)sum_gz / samples;

        // Seed attitude from the averaged accelerometer, which is far quieter
        // than any single sample.
        ekf_.reset();
        ekf_.initializeFromAccel((float)sum_ax / samples / 4096.0f,
                                 (float)sum_ay / samples / 4096.0f,
                                 (float)sum_az / samples / 4096.0f);
    }

    /** One sensor read plus one estimator step. */
    void update(float dt) {
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        readRaw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

        constexpr float kLsbPerDegPerSec = 65.5f;
        constexpr float kLsbPerG = 4096.0f;
        constexpr float kDegToRad = 0.01745329252f;

        p_ = (gx_raw - gyro_offset_x_) / kLsbPerDegPerSec * kDegToRad;
        q_ = (gy_raw - gyro_offset_y_) / kLsbPerDegPerSec * kDegToRad;
        r_ = (gz_raw - gyro_offset_z_) / kLsbPerDegPerSec * kDegToRad;

        const float ax = ax_raw / kLsbPerG;
        const float ay = ay_raw / kLsbPerG;
        const float az = az_raw / kLsbPerG;

        ekf_.predict(p_, q_, r_, dt);
        ekf_.updateAccel(ax, ay, az);
    }

    float getRoll() const { return ekf_.getRollDeg(); }
    float getPitch() const { return ekf_.getPitchDeg(); }
    /** Yaw rate in deg/s, straight off the gyro. Yaw is flown as a rate. */
    float getYawRate() const { return r_ * 57.2957795f; }

    float getBiasP() const { return ekf_.getBiasP(); }
    float getBiasQ() const { return ekf_.getBiasQ(); }
    float getRollSigma() const { return ekf_.getRollSigma(); }
    float getInnovation(int axis) const { return ekf_.getInnovation(axis); }

    /**
     * True when the estimator has gone non-finite. Wired straight into the
     * flight state machine, which treats it as unrecoverable: if attitude is
     * unknown then every motor command after this point is a guess.
     */
    bool isDiverged() const { return ekf_.isDiverged(); }

private:
    AttitudeEKF ekf_;
    float gyro_offset_x_ = 0, gyro_offset_y_ = 0, gyro_offset_z_ = 0;
    float p_ = 0, q_ = 0, r_ = 0; // bias-corrected body rates, rad/s

    void writeReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    void readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                 int16_t& gx, int16_t& gy, int16_t& gz) {
        ax = ay = az = 0;
        gx = gy = gz = 0;

        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(0x3B); // ACCEL_XOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14, (uint8_t)true);

        if (Wire.available() == 14) {
            ax = Wire.read() << 8 | Wire.read();
            ay = Wire.read() << 8 | Wire.read();
            az = Wire.read() << 8 | Wire.read();
            Wire.read(); Wire.read(); // temperature, unused
            gx = Wire.read() << 8 | Wire.read();
            gy = Wire.read() << 8 | Wire.read();
            gz = Wire.read() << 8 | Wire.read();
        }
        // A short read leaves every output at zero. The EKF skips a zero-length
        // accelerometer vector rather than dividing by it, so a dropped I2C
        // transaction costs one update instead of corrupting the filter.
    }
};
