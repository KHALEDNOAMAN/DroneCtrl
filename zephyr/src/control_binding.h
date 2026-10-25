/*
 * The single seam between Zephyr and the existing DroneCtrl core.
 *
 * main.c is C and owns the platform: timing, I2C, PWM, watchdog. The core is
 * header-only C++. This header is the extern "C" boundary between them.
 *
 * The port uses the PURE classes directly and leaves the two Arduino-bound ones
 * behind:
 *
 *   used as-is   AttitudeEKF         attitude_ekf.h
 *                PIDController       pid_controller.h
 *                mixer::mixX         mixer_math.h
 *                FlightStateMachine  flight_state.h
 *
 *   replaced     IMU         imu.h owns Wire register reads AND wraps the EKF.
 *                            Zephyr reads the MPU6050 over i2c_dt_spec and
 *                            drives AttitudeEKF directly.
 *                MotorMixer  motor_mixer.h computes the mix and then writes
 *                            Arduino pins. Only the write is Arduino-bound,
 *                            and mixer_math.h already holds the arithmetic, so
 *                            the Zephyr build calls mixX and writes PWM itself.
 *
 * That split is why this port is a weekend and not a rewrite: the separation
 * between mixer_math.h and motor_mixer.h was already made.
 */

#ifndef DC_CONTROL_BINDING_H
#define DC_CONTROL_BINDING_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mirrors ESC_MIN in firmware/include/config.h. */
#define DC_ESC_DISARMED_US 1000

struct dc_imu_sample {
	float gyro_dps[3];   /* p, q, r in degrees per second */
	float accel_g[3];    /* x, y, z in g */
};

struct dc_motor_outputs {
	uint16_t pulse_us[4];   /* front-left, front-right, back-left, back-right */
	bool motors_enabled;
	bool authority_limited;
};

/* Configure the MPU6050 over I2C. Returns 0, or a negative errno. */
int dc_imu_init(const struct i2c_dt_spec *bus);

/* One burst read of all six axes. Returns 0 on success. */
int dc_imu_read(const struct i2c_dt_spec *bus, struct dc_imu_sample *out);

/* Construct the estimator, the three PIDs and the state machine. */
void dc_control_init(void);

/* One control step at a fixed dt. Always writes a defined output, including on
 * the failure path, because main.c writes the ESCs unconditionally. */
void dc_control_step(const struct dc_imu_sample *imu, bool imu_valid,
		     float dt, uint32_t now_ms, struct dc_motor_outputs *out);

#ifdef __cplusplus
}
#endif

#endif /* DC_CONTROL_BINDING_H */
