#pragma once

/**
 * Extended Kalman filter for roll and pitch, with online gyro bias estimation.
 *
 * Why this replaces the complementary filter
 * ------------------------------------------
 * A complementary filter blends gyro and accelerometer with one fixed weight.
 * That weight is a permanent compromise: high enough to reject accelerometer
 * noise means slow correction of gyro drift, and low enough to track drift
 * means every bump in the airframe leaks into the attitude estimate. It also
 * has nowhere to put gyro bias, so a warm drifting gyro is corrected only
 * indirectly, through the same fixed blend.
 *
 * An EKF carries a covariance, so the blend is computed every step from how
 * much the filter currently trusts each source, and it carries bias as part of
 * the state, so drift is estimated rather than fought.
 *
 * State (4)
 * ---------
 *   x = [ roll, pitch, bias_p, bias_q ]        radians, radians/second
 *
 * Bias is modelled as a random walk. Yaw is deliberately absent: a single
 * accelerometer observes gravity, and gravity says nothing about heading, so a
 * yaw state here would be unobservable and would drift without bound. Yaw on
 * this airframe is flown as a rate, straight off the gyro. Adding a
 * magnetometer is what would make a yaw state observable.
 *
 * Process model
 * -------------
 * Euler-angle kinematics driven by bias-corrected body rates:
 *
 *   roll_dot  = p' + q' sin(roll) tan(pitch) + r cos(roll) tan(pitch)
 *   pitch_dot = q' cos(roll) - r sin(roll)
 *   bias_dot  = 0
 *
 * where p' = p - bias_p and q' = q - bias_q.
 *
 * Measurement model
 * -----------------
 * The accelerometer is treated as an observation of the gravity vector in body
 * axes rather than being pre-converted into angles:
 *
 *   h(x) = g * [ -sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch) ]
 *
 * Feeding the raw vector through the real nonlinear model is better than
 * computing atan2 angles first. Pre-computing angles bakes in a linearisation
 * the filter never sees, and it loses the information carried by the vector's
 * length, which is what the accelerometer trust check below is built on.
 *
 * Accelerometer trust
 * -------------------
 * The measurement model assumes the only specific force is gravity. That is
 * false whenever the airframe accelerates, which on a quadcopter is most of
 * the time. The length of the measured vector is the available evidence: when
 * it departs from 1 g, something other than gravity is being measured, so R is
 * inflated in proportion and the update is de-weighted rather than dropped.
 * Dropping it outright would throw away the partial information in a mildly
 * disturbed sample and would make the filter's behaviour discontinuous.
 *
 * Implementation notes
 * --------------------
 * No dynamic allocation and no library dependency, so this compiles for the
 * ATmega328 as well as the ESP32 and links into the host test and SIL builds
 * unchanged. Every matrix is a fixed-size float array. The largest temporary
 * is the 4x3 gain, so the whole update runs in roughly 200 bytes of stack.
 */

#include <math.h>

class AttitudeEKF {
public:
    /** Standard gravity, in the same units the accelerometer is fed in (g). */
    static constexpr float kGravity = 1.0f;

    struct Tuning {
        /** Gyro white noise, (rad/s)^2. Raise to trust the accelerometer more. */
        float gyro_noise = 4.0e-4f;
        /** Bias random-walk noise, (rad/s^2)^2. Sets how fast bias is tracked. */
        float bias_noise = 1.0e-7f;
        /** Accelerometer noise, g^2, at rest. */
        float accel_noise = 4.0e-2f;
        /**
         * How hard to distrust the accelerometer per g of departure from 1 g.
         * R is scaled by (1 + k * |‖a‖ - 1|)^2, so k = 0 disables the check.
         */
        float accel_reject_gain = 40.0f;
        /** Hard ceiling on |pitch| used to keep tan(pitch) finite, radians. */
        float max_pitch = 1.45f; // ~83 degrees
    };

    AttitudeEKF() { reset(); }

    explicit AttitudeEKF(const Tuning& tuning) : tuning_(tuning) { reset(); }

    void setTuning(const Tuning& tuning) { tuning_ = tuning; }

    /** Zero the state and reset covariance to the initial uncertainty. */
    void reset() {
        for (int i = 0; i < 4; ++i) {
            x_[i] = 0.0f;
            for (int j = 0; j < 4; ++j) P_[i][j] = 0.0f;
        }
        // Angles start unknown to within a large angle; bias starts unknown to
        // within a few degrees per second. Both shrink within a second of data.
        P_[0][0] = 1.0f;
        P_[1][1] = 1.0f;
        P_[2][2] = 1.0e-2f;
        P_[3][3] = 1.0e-2f;
    }

    /**
     * Seed roll and pitch from one accelerometer sample, so the filter starts
     * near the truth instead of converging from zero. Worth calling once the
     * airframe is known to be still, straight after gyro calibration.
     */
    void initializeFromAccel(float ax, float ay, float az) {
        const float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 1.0e-3f) return;
        x_[0] = atan2f(ay, az);
        x_[1] = atan2f(-ax, sqrtf(ay * ay + az * az));
        P_[0][0] = 1.0e-2f;
        P_[1][1] = 1.0e-2f;
    }

    /**
     * Propagate the state forward with the measured body rates.
     *
     * @param p,q,r body roll, pitch and yaw rates in radians/second
     * @param dt    step in seconds
     */
    void predict(float p, float q, float r, float dt) {
        if (dt <= 0.0f) return;

        const float roll = x_[0];
        const float pitch = clampPitch(x_[1]);
        const float pc = p - x_[2];
        const float qc = q - x_[3];

        const float sr = sinf(roll), cr = cosf(roll);
        const float tp = tanf(pitch);
        const float cp = cosf(pitch);
        // cos(pitch) is bounded away from zero by clampPitch, so sec^2 is finite.
        const float sec2 = 1.0f / (cp * cp);

        const float roll_dot = pc + qc * sr * tp + r * cr * tp;
        const float pitch_dot = qc * cr - r * sr;

        // Jacobian of the process model. Rows are the state derivatives, columns
        // the states they are differentiated against.
        float F[4][4] = {{0}};
        F[0][0] = qc * cr * tp - r * sr * tp;
        F[0][1] = (qc * sr + r * cr) * sec2;
        F[0][2] = -1.0f;
        F[0][3] = -sr * tp;
        F[1][0] = -qc * sr - r * cr;
        F[1][1] = 0.0f;
        F[1][2] = 0.0f;
        F[1][3] = -cr;
        // Bias rows stay zero: a random walk has no deterministic drift.

        x_[0] = roll + roll_dot * dt;
        x_[1] = clampPitch(pitch + pitch_dot * dt);
        x_[0] = wrapPi(x_[0]);

        // Discrete transition, first order: Phi = I + F dt. At 250 Hz the
        // neglected term is O(dt^2) = 1.6e-5, far below the process noise.
        float Phi[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                Phi[i][j] = (i == j ? 1.0f : 0.0f) + F[i][j] * dt;
            }
        }

        // P = Phi P Phi^T + Q
        float PhiP[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += Phi[i][k] * P_[k][j];
                PhiP[i][j] = s;
            }
        }
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += PhiP[i][k] * Phi[j][k];
                P_[i][j] = s;
            }
        }

        // Gyro noise enters through the angle states, bias noise through its own.
        const float qg = tuning_.gyro_noise * dt;
        const float qb = tuning_.bias_noise * dt;
        P_[0][0] += qg;
        P_[1][1] += qg;
        P_[2][2] += qb;
        P_[3][3] += qb;

        symmetrize();
    }

    /**
     * Correct with one accelerometer sample.
     *
     * @param ax,ay,az specific force in body axes, in g
     */
    void updateAccel(float ax, float ay, float az) {
        const float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 1.0e-3f) return; // free fall or a dead sensor, nothing to use

        const float roll = x_[0];
        const float pitch = clampPitch(x_[1]);
        const float sr = sinf(roll), cr = cosf(roll);
        const float sp = sinf(pitch), cp = cosf(pitch);
        const float g = kGravity;

        // Predicted gravity vector in body axes.
        const float h[3] = {-g * sp, g * sr * cp, g * cr * cp};

        // H = dh/dx. The bias columns are zero: the accelerometer cannot see
        // gyro bias directly. It is still estimated, through the correlation
        // that P builds up between the angle and bias states during predict.
        float H[3][4] = {{0}};
        H[0][0] = 0.0f;       H[0][1] = -g * cp;
        H[1][0] = g * cr * cp; H[1][1] = -g * sr * sp;
        H[2][0] = -g * sr * cp; H[2][1] = -g * cr * sp;

        // Inflate R when the measured magnitude says this is not just gravity.
        const float excess = fabsf(norm - g);
        const float scale = 1.0f + tuning_.accel_reject_gain * excess;
        const float r_acc = tuning_.accel_noise * scale * scale;

        // S = H P H^T + R
        float HP[3][4];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += H[i][k] * P_[k][j];
                HP[i][j] = s;
            }
        }
        float S[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += HP[i][k] * H[j][k];
                S[i][j] = s + (i == j ? r_acc : 0.0f);
            }
        }

        float Sinv[3][3];
        if (!invert3x3(S, Sinv)) return; // singular, skip rather than corrupt P

        // K = P H^T S^-1
        float PHt[4][3];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += P_[i][k] * H[j][k];
                PHt[i][j] = s;
            }
        }
        float K[4][3];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 3; ++k) s += PHt[i][k] * Sinv[k][j];
                K[i][j] = s;
            }
        }

        const float y[3] = {ax - h[0], ay - h[1], az - h[2]};
        for (int i = 0; i < 3; ++i) innovation_[i] = y[i];

        for (int i = 0; i < 4; ++i) {
            float d = 0.0f;
            for (int j = 0; j < 3; ++j) d += K[i][j] * y[j];
            x_[i] += d;
        }
        x_[0] = wrapPi(x_[0]);
        x_[1] = clampPitch(x_[1]);

        // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T. The textbook short form
        // (I-KH)P is cheaper but loses symmetry and positive-definiteness to
        // rounding, which on 32-bit floats at 250 Hz shows up as a filter that
        // quietly stops correcting after a few minutes.
        float IKH[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 3; ++k) s += K[i][k] * H[k][j];
                IKH[i][j] = (i == j ? 1.0f : 0.0f) - s;
            }
        }
        float T[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += IKH[i][k] * P_[k][j];
                T[i][j] = s;
            }
        }
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += T[i][k] * IKH[j][k];
                float krk = 0.0f;
                for (int k = 0; k < 3; ++k) krk += K[i][k] * r_acc * K[j][k];
                P_[i][j] = s + krk;
            }
        }

        symmetrize();
    }

    float getRollRad() const { return x_[0]; }
    float getPitchRad() const { return x_[1]; }
    float getRollDeg() const { return x_[0] * 57.2957795f; }
    float getPitchDeg() const { return x_[1] * 57.2957795f; }

    /** Estimated gyro bias, radians/second. */
    float getBiasP() const { return x_[2]; }
    float getBiasQ() const { return x_[3]; }

    /** One-sigma uncertainty in roll and pitch, radians. */
    float getRollSigma() const { return sqrtf(P_[0][0] > 0.0f ? P_[0][0] : 0.0f); }
    float getPitchSigma() const { return sqrtf(P_[1][1] > 0.0f ? P_[1][1] : 0.0f); }

    /**
     * Last accelerometer residual, in g. Flight-test convention: a healthy
     * filter leaves this zero-mean and inside the predicted sigma, so logging
     * it is how a tuning problem is told apart from a sensor problem.
     */
    float getInnovation(int axis) const {
        return (axis >= 0 && axis < 3) ? innovation_[axis] : 0.0f;
    }

    /** True if any state or covariance entry has gone non-finite. */
    bool isDiverged() const {
        for (int i = 0; i < 4; ++i) {
            if (!isfinite(x_[i])) return true;
            for (int j = 0; j < 4; ++j) {
                if (!isfinite(P_[i][j])) return true;
            }
        }
        return false;
    }

private:
    Tuning tuning_;
    float x_[4];
    float P_[4][4];
    float innovation_[3] = {0.0f, 0.0f, 0.0f};

    float clampPitch(float pitch) const {
        if (pitch > tuning_.max_pitch) return tuning_.max_pitch;
        if (pitch < -tuning_.max_pitch) return -tuning_.max_pitch;
        return pitch;
    }

    static float wrapPi(float a) {
        const float two_pi = 6.283185307f;
        while (a > 3.141592654f) a -= two_pi;
        while (a < -3.141592654f) a += two_pi;
        return a;
    }

    /** Force P symmetric. Rounding makes the two triangles drift apart. */
    void symmetrize() {
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                const float m = 0.5f * (P_[i][j] + P_[j][i]);
                P_[i][j] = m;
                P_[j][i] = m;
            }
        }
    }

    static bool invert3x3(const float m[3][3], float out[3][3]) {
        const float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        const float c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        const float c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        const float det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
        if (fabsf(det) < 1.0e-12f) return false;
        const float inv = 1.0f / det;
        out[0][0] = c00 * inv;
        out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
        out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
        out[1][0] = c01 * inv;
        out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
        out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
        out[2][0] = c02 * inv;
        out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
        out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
        return true;
    }
};
