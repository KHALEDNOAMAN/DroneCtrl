/*
 * DroneCtrl on Zephyr: the 250 Hz control thread.
 *
 * This file is the platform side. It owns timing, the I2C transfers, the four
 * PWM channels and the watchdog. It contains no control law, because that
 * already exists in firmware/include and is already covered by the host tests.
 *
 * The timing design is the part worth reading.
 *
 * A superloop with a delay at the bottom accumulates drift: the iteration takes
 * however long it takes and the delay adds to that rather than absorbing it, so
 * the real period is 4 ms plus the work and it moves whenever the work moves. A
 * PID that divides by a nominal dt it is not actually running at is being
 * quietly mistuned by its own scheduler, and the gains in config.h were derived
 * for 250 Hz exactly.
 *
 * So the period comes from a k_timer, which fires on an absolute schedule, and
 * the thread blocks on a semaphore the timer gives. An overrun leaves the next
 * tick already pending, so the thread runs again immediately rather than
 * sliding, and the overrun is counted and reported instead of hidden.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "control_binding.h"

LOG_MODULE_REGISTER(dronectrl, LOG_LEVEL_INF);

/* LOOP_FREQUENCY_HZ in firmware/include/config.h. Same rate as the ESP32 build,
 * so the derived gains carry over unchanged. */
#define CONTROL_HZ        250
#define CONTROL_PERIOD_US (1000000 / CONTROL_HZ)

#define CONTROL_STACK_SIZE 4096
/* Cooperative, so nothing preempts a step once it has started. A preempted
 * mixer write is a partial actuator update. */
#define CONTROL_PRIORITY   K_PRIO_COOP(4)

K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
static struct k_thread control_thread;

static K_SEM_DEFINE(tick_sem, 0, 1);
static struct k_timer control_timer;

static atomic_t overruns;
static atomic_t ticks;

static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(imu));

static const struct pwm_dt_spec esc[4] = {
	PWM_DT_SPEC_GET(DT_NODELABEL(esc0)),   /* front left  */
	PWM_DT_SPEC_GET(DT_NODELABEL(esc1)),   /* front right */
	PWM_DT_SPEC_GET(DT_NODELABEL(esc2)),   /* back left   */
	PWM_DT_SPEC_GET(DT_NODELABEL(esc3)),   /* back right  */
};

#if DT_HAS_CHOSEN(zephyr_watchdog)
static const struct device *const wdt = DEVICE_DT_GET(DT_CHOSEN(zephyr_watchdog));
static int wdt_channel = -1;
#endif

static void on_tick(struct k_timer *t)
{
	ARG_UNUSED(t);
	atomic_inc(&ticks);
	/* k_sem_give on a semaphore already at its limit of 1 is a no-op, which is
	 * exactly the behaviour wanted: the previous step had not finished, so the
	 * tick is recorded as an overrun and no backlog builds. */
	if (k_sem_count_get(&tick_sem) > 0) {
		atomic_inc(&overruns);
	}
	k_sem_give(&tick_sem);
}

static void control_main(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	const float dt = 1.0f / (float)CONTROL_HZ;
	uint32_t report_at = CONTROL_HZ;

	for (;;) {
		k_sem_take(&tick_sem, K_FOREVER);

		struct dc_imu_sample imu;
		const int rc = dc_imu_read(&imu_i2c, &imu);

		struct dc_motor_outputs out;
		dc_control_step(&imu, rc == 0, dt, k_uptime_get_32(), &out);

		/* Written last and together, so the four ESCs always see one coherent
		 * mixer solution. */
		for (int i = 0; i < 4; i++) {
			(void)pwm_set_pulse_dt(&esc[i], PWM_USEC(out.pulse_us[i]));
		}

#if DT_HAS_CHOSEN(zephyr_watchdog)
		/* Fed only after a complete step. A watchdog fed from a timer callback
		 * proves the board is powered, not that the loop is running, which is
		 * the only thing worth proving. */
		if (wdt_channel >= 0) {
			(void)wdt_feed(wdt, wdt_channel);
		}
#endif

		if (--report_at == 0) {
			report_at = CONTROL_HZ;
			const uint32_t o = (uint32_t)atomic_clear(&overruns);
			const uint32_t n = (uint32_t)atomic_clear(&ticks);
			if (o > 0) {
				LOG_WRN("control overruns: %u of %u ticks", o, n);
			} else {
				LOG_INF("control: %u ticks, no overrun", n);
			}
		}
	}
}

int main(void)
{
	LOG_INF("DroneCtrl on Zephyr, %d Hz control loop", CONTROL_HZ);

	if (!device_is_ready(imu_i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}
	for (int i = 0; i < 4; i++) {
		if (!pwm_is_ready_dt(&esc[i])) {
			LOG_ERR("ESC %d PWM not ready", i);
			return -ENODEV;
		}
	}

	/* Hold every ESC at its disarmed pulse BEFORE anything else runs. Motors
	 * that spin during bring-up are the failure that costs fingers. */
	for (int i = 0; i < 4; i++) {
		(void)pwm_set_pulse_dt(&esc[i], PWM_USEC(DC_ESC_DISARMED_US));
	}

	const int rc = dc_imu_init(&imu_i2c);
	if (rc != 0) {
		LOG_ERR("IMU init failed (%d)", rc);
		return rc;
	}
	dc_control_init();

#if DT_HAS_CHOSEN(zephyr_watchdog)
	if (device_is_ready(wdt)) {
		struct wdt_timeout_cfg cfg = {
			/* Four missed control steps: tight enough to catch a hung loop
			 * quickly, loose enough that one slow step is not a reset. */
			.window.min = 0,
			.window.max = (4 * 1000) / CONTROL_HZ,
			.callback = NULL,
			.flags = WDT_FLAG_RESET_SOC,
		};
		wdt_channel = wdt_install_timeout(wdt, &cfg);
		if (wdt_channel >= 0) {
			(void)wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
			LOG_INF("watchdog armed at %u ms", cfg.window.max);
		}
	}
#endif

	k_thread_create(&control_thread, control_stack, CONTROL_STACK_SIZE,
			control_main, NULL, NULL, NULL,
			CONTROL_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&control_thread, "control");

	k_timer_init(&control_timer, on_tick, NULL);
	k_timer_start(&control_timer, K_USEC(CONTROL_PERIOD_US),
		      K_USEC(CONTROL_PERIOD_US));

	return 0;
}
