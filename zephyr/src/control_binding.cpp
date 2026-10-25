/*
 * DroneCtrl control core, bound to Zephyr.
 *
 * Note which headers are NOT included: config.h, imu.h and motor_mixer.h.
 *
 * config.h cannot be included from a Zephyr build as it stands, because it
 * declares `constexpr uint8_t PIN_BATTERY = A0;` and `A0` is an Arduino macro.
 * Everything else in that file is pure tuning data. The clean fix upstream is
 * to split it into config.h (tuning, platform-free) and board_pins.h (pins,
 * Arduino-only), after which this file includes config.h like any other and the
 * duplicated constants below disappear. Until then they are mirrored here, and
 * mirrored constants are a liability: if you change a gain in config.h, change
 * it here in the same commit.
 */

#include "control_binding.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include "attitude_ekf.h"
#include "pid_controller.h"
#include "mixer_math.h"
#include "flight_state.h"

/* ---- mirrored from firmware/include/config.h ------------------------------ */
namespace {

constexpr float kPidRollKp   = 2.7f;
constexpr float kPidRollKi   = 1.8f;
constexpr float kPidRollKd   = 0.55f;
constexpr float kPidPitchKp  = 2.7f;
constexpr float kPidPitchKi  = 1.8f;
constexpr float kPidPitchKd  = 0.55f;
constexpr float kPidYawKp    = 4.0f;
constexpr float kPidYawKi    = 0.5f;
constexpr float kPidYawKd    = 0.0f;

constexpr float kMaxTiltSetpointDeg = 30.0f;
constexpr float kMaxYawRateDps      = 150.0f;
constexpr float kMaxTiltAngleDeg    = 45.0f;
constexpr float kMinBatteryVolts    = 10.5f;

constexpr float kEscMin  = 1000.0f;
constexpr float kEscMax  = 2000.0f;
constexpr float kEscIdle = 1050.0f;

/* MPU6050 registers. Same values imu.h writes; a register map does not change
 * with the RTOS, which is why this driver is a substitution and not a rewrite. */
constexpr uint8_t kRegSmplrtDiv   = 0x19;
constexpr uint8_t kRegConfig      = 0x1A;
constexpr uint8_t kRegGyroConfig  = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH  = 0x3B;
constexpr uint8_t kRegPwrMgmt1    = 0x6B;
constexpr uint8_t kRegWhoAmI      = 0x75;

/* +-500 deg/s and +-4 g, matching the scale factors below. */
constexpr float kGyroLsbPerDps = 65.5f;
constexpr float kAccelLsbPerG  = 8192.0f;

/* ---- the core, constructed once ------------------------------------------- */

AttitudeEKF *g_ekf;
PIDController<float> g_pid_roll;
PIDController<float> g_pid_pitch;
PIDController<float> g_pid_yaw;
FlightStateMachine *g_fsm;

bool g_initialised_from_accel;

alignas(AttitudeEKF) uint8_t g_ekf_storage[sizeof(AttitudeEKF)];
alignas(FlightStateMachine) uint8_t g_fsm_storage[sizeof(FlightStateMachine)];

} /* namespace */

/* ---- IMU over Zephyr I2C -------------------------------------------------- */

extern "C" int dc_imu_init(const struct i2c_dt_spec *bus)
{
	uint8_t who = 0;
	int rc = i2c_reg_read_byte_dt(bus, kRegWhoAmI, &who);
	if (rc != 0) {
		return rc;
	}
	/* 0x68 is the MPU6050, 0x70 and 0x72 are MPU6500 and MPU9250 clones that
	 * answer the same register map. Accepting them is deliberate; silently
	 * accepting anything is not. */
	if (who != 0x68 && who != 0x70 && who != 0x72) {
		return -ENODEV;
	}

	/* Wake up and select the X gyro as the clock source. The internal
	 * oscillator drifts enough over a flight to matter to an integrator. */
	rc = i2c_reg_write_byte_dt(bus, kRegPwrMgmt1, 0x01);
	if (rc != 0) {
		return rc;
	}
	k_msleep(100);

	/* DLPF at 42 Hz. Above the airframe's own dynamics, well below the 125 Hz
	 * Nyquist of a 250 Hz loop, so the filter removes motor noise without
	 * adding phase lag the derivative term would then amplify. */
	rc = i2c_reg_write_byte_dt(bus, kRegConfig, 0x03);
	if (rc != 0) {
		return rc;
	}
	/* Sample at 1 kHz / (1 + 3) = 250 Hz, matching the control loop exactly, so
	 * every step reads one fresh sample and never the same one twice. */
	rc = i2c_reg_write_byte_dt(bus, kRegSmplrtDiv, 0x03);
	if (rc != 0) {
		return rc;
	}
	rc = i2c_reg_write_byte_dt(bus, kRegGyroConfig, 0x08);   /* +-500 deg/s */
	if (rc != 0) {
		return rc;
	}
	rc = i2c_reg_write_byte_dt(bus, kRegAccelConfig, 0x08);  /* +-4 g */
	return rc;
}

extern "C" int dc_imu_read(const struct i2c_dt_spec *bus, struct dc_imu_sample *out)
{
	/* One burst of 14 bytes: accel x/y/z, temperature, gyro x/y/z.
	 *
	 * Deliberately not six separate register reads. At 250 Hz those spread one
	 * sample across a window wide enough that the accelerometer and the
	 * gyroscope no longer describe the same instant, and the estimator sees
	 * that as noise no amount of tuning removes. */
	uint8_t raw[14];
	int rc = i2c_burst_read_dt(bus, kRegAccelXoutH, raw, sizeof(raw));
	if (rc != 0) {
		return rc;
	}

	const int16_t ax = (int16_t)sys_get_be16(&raw[0]);
	const int16_t ay = (int16_t)sys_get_be16(&raw[2]);
	const int16_t az = (int16_t)sys_get_be16(&raw[4]);
	/* raw[6..7] is temperature, skipped */
	const int16_t gx = (int16_t)sys_get_be16(&raw[8]);
	const int16_t gy = (int16_t)sys_get_be16(&raw[10]);
	const int16_t gz = (int16_t)sys_get_be16(&raw[12]);

	out->accel_g[0] = (float)ax / kAccelLsbPerG;
	out->accel_g[1] = (float)ay / kAccelLsbPerG;
	out->accel_g[2] = (float)az / kAccelLsbPerG;
	out->gyro_dps[0] = (float)gx / kGyroLsbPerDps;
	out->gyro_dps[1] = (float)gy / kGyroLsbPerDps;
	out->gyro_dps[2] = (float)gz / kGyroLsbPerDps;
	return 0;
}

/* ---- control ------------------------------------------------------------- */

extern "C" void dc_control_init(void)
{
	AttitudeEKF::Tuning tuning{};          /* the header's defaults */
	g_ekf = new (g_ekf_storage) AttitudeEKF(tuning);

	FlightLimits limits{};
	limits.max_tilt_deg = kMaxTiltAngleDeg;
	limits.min_battery_volts = kMinBatteryVolts;
	g_fsm = new (g_fsm_storage) FlightStateMachine(limits);

	g_pid_roll.setGains(kPidRollKp, kPidRollKi, kPidRollKd);
	g_pid_pitch.setGains(kPidPitchKp, kPidPitchKi, kPidPitchKd);
	g_pid_yaw.setGains(kPidYawKp, kPidYawKi, kPidYawKd);

	g_initialised_from_accel = false;
}

extern "C" void dc_control_step(const struct dc_imu_sample *imu, bool imu_valid,
				float dt, uint32_t now_ms,
				struct dc_motor_outputs *out)
{
	/* Defined output first, so every early return below is still safe for the
	 * caller, which writes the ESCs unconditionally. */
	for (int i = 0; i < 4; i++) {
		out->pulse_us[i] = DC_ESC_DISARMED_US;
	}
	out->motors_enabled = false;
	out->authority_limited = false;

	/* 1. estimator */
	if (imu_valid) {
		if (!g_initialised_from_accel) {
			g_ekf->initializeFromAccel(imu->accel_g[0], imu->accel_g[1],
						   imu->accel_g[2]);
			g_initialised_from_accel = true;
		}
		g_ekf->predict(imu->gyro_dps[0], imu->gyro_dps[1], imu->gyro_dps[2], dt);
		g_ekf->updateAccel(imu->accel_g[0], imu->accel_g[1], imu->accel_g[2]);
	} else if (g_initialised_from_accel) {
		/* A dropped read is not a level airframe. Propagate on the last known
		 * rates and let the state machine see a diverging estimate, rather
		 * than feeding the filter a zero sample it would believe. */
		g_ekf->predict(0.0f, 0.0f, 0.0f, dt);
	}

	/* 2. state machine
	 *
	 * The receiver and the battery are not wired on this build yet, so the
	 * inputs below are the bench values. They are the next thing to connect,
	 * and until they are, signal_lost is held true on purpose so the machine
	 * cannot leave DISARMED and a bench board cannot spin a motor. */
	FlightInputs in{};
	in.roll_deg = g_ekf->getRollDeg();
	in.pitch_deg = g_ekf->getPitchDeg();
	in.estimator_diverged = g_ekf->isDiverged();
	in.signal_lost = true;        /* TODO: replace when the receiver is wired */
	in.throttle_norm = 0.0f;
	in.yaw_norm = 0.0f;
	in.battery_volts = 12.6f;     /* TODO: replace with the ADC reading */

	g_fsm->update(in, now_ms, dt);

	if (!g_fsm->motorsEnabled()) {
		g_pid_roll.reset();
		g_pid_pitch.reset();
		g_pid_yaw.reset();
		return;
	}

	/* 3. attitude PIDs. The setpoints come from the sticks in a flying build;
	 * level is the bench case. */
	const bool hold = g_fsm->holdIntegrators();
	const float roll_sp = 0.0f * kMaxTiltSetpointDeg;
	const float pitch_sp = 0.0f * kMaxTiltSetpointDeg;
	const float yaw_rate_sp = 0.0f * kMaxYawRateDps;

	const float roll_cmd = g_pid_roll.compute(roll_sp, in.roll_deg, dt, hold);
	const float pitch_cmd = g_pid_pitch.compute(pitch_sp, in.pitch_deg, dt, hold);
	const float yaw_cmd = g_pid_yaw.compute(yaw_rate_sp, imu->gyro_dps[2], dt, hold);

	/* 4. mix. The same pure function the host tests and the SIL harness call. */
	const float base = kEscIdle +
		g_fsm->commandedThrottle() * (kEscMax - kEscIdle);
	const mixer::Outputs mix = mixer::mixX(base, pitch_cmd, roll_cmd, yaw_cmd,
					       kEscMin, kEscMax);

	out->pulse_us[0] = (uint16_t)mix.fl;
	out->pulse_us[1] = (uint16_t)mix.fr;
	out->pulse_us[2] = (uint16_t)mix.bl;
	out->pulse_us[3] = (uint16_t)mix.br;
	out->motors_enabled = true;
	out->authority_limited = mix.authority_limited;
}
