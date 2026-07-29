#include <Arduino.h>
#include "config.h"
#include "mpu6050.h"
#include "receiver.h"
#include "motor_mixer.h"
#include "pid_controller.h"
#include "battery.h"
#include "safety.h"

MPU6050 mpu;
Receiver rx;
MotorMixer mixer;
Battery battery;
Safety safety(battery, rx, mpu, mixer);

// PIDs: Kp, Ki, Kd, max_integral, max_output
PIDController<float> pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 100.0f, 400.0f);
PIDController<float> pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 100.0f, 400.0f);
PIDController<float> pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 100.0f, 400.0f);

unsigned long prev_loop_time = 0;
unsigned long prev_debug_time = 0;

void setup() {
    Serial.begin(115200);
    
    // Init components
    if (!mpu.init()) {
        Serial.println("MPU6050 init failed!");
        while (1) { delay(10); }
    }
    
    Serial.println("Calibrating MPU... Keep drone level.");
    mpu.calibrate();
    Serial.println("Calibration complete.");

    rx.init();
    mixer.init();
    battery.init();

    prev_loop_time = micros();
}

void loop() {
    unsigned long current_time = micros();
    if (current_time - prev_loop_time >= LOOP_INTERVAL_US) {
        prev_loop_time = current_time;

        // 1. Read Sensors
        mpu.read(DT);
        rx.readChannels();
        
        // Update battery at lower rate
        static int bat_divider = 0;
        if (++bat_divider > 50) {
            battery.update();
            bat_divider = 0;
        }

        // 2. Safety Check
        SafetyStatus status = safety.checkAll();

        // 3. Control Logic
        if (mixer.isArmed() && status == SafetyStatus::OK && rx.getThrottle() > 0.05f) {
            // Setpoints from RC (Max 30 deg tilt)
            float sp_roll = rx.getRoll() * 30.0f;
            float sp_pitch = rx.getPitch() * 30.0f;
            float sp_yaw_rate = rx.getYaw() * MAX_YAW_RATE;

            // PID compute
            float out_roll = pidRoll.compute(sp_roll, mpu.getRoll(), DT);
            float out_pitch = pidPitch.compute(sp_pitch, mpu.getPitch(), DT);
            float out_yaw = pidYaw.compute(sp_yaw_rate, mpu.getYawRate(), DT); 

            mixer.setThrottle(rx.getThrottle(), out_pitch, out_roll, out_yaw);
        } else {
            pidRoll.reset();
            pidPitch.reset();
            pidYaw.reset();
            if (!mixer.isArmed()) {
                mixer.setThrottle(0, 0, 0, 0); // Ensures motors are off
            }
        }
        
        // 4. Debug output at 50Hz
        if (millis() - prev_debug_time > 20) {
            prev_debug_time = millis();
            /*
            Serial.print("R: "); Serial.print(mpu.getRoll());
            Serial.print(" P: "); Serial.print(mpu.getPitch());
            Serial.print(" Armed: "); Serial.print(mixer.isArmed());
            Serial.print(" V: "); Serial.println(battery.getVoltage());
            */
        }
    }
}
