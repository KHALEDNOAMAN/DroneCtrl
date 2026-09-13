export class PIDController {
  private kp: number;
  private ki: number;
  private kd: number;

  private outputMin: number;
  private outputMax: number;
  private integralMax: number;

  /**
   * Low-pass time constant for the derivative term, in seconds. Raw derivatives
   * amplify the step-to-step noise a real MPU6050 produces, and in a rate loop
   * they also feed the controller's own last output straight back into its D
   * term, which oscillates at the loop rate. The firmware filters for the same
   * reason; 0 disables it.
   */
  private derivativeTau: number;
  private filteredDerivative = 0;

  private integral: number = 0;
  private prevError: number = 0;
  private lastMeasurement: number = 0;
  private primed = false;

  constructor(
    kp: number,
    ki: number,
    kd: number,
    outputMin: number,
    outputMax: number,
    integralMax: number,
    derivativeTau: number = 0.02,
  ) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.outputMin = outputMin;
    this.outputMax = outputMax;
    this.integralMax = integralMax;
    this.derivativeTau = derivativeTau;
  }

  setGains(kp: number, ki: number, kd: number) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  reset() {
    this.integral = 0;
    this.prevError = 0;
    this.lastMeasurement = 0;
    this.filteredDerivative = 0;
    this.primed = false;
  }

  compute(setpoint: number, measurement: number, dt: number): number {
    if (dt <= 0) return 0;

    const error = setpoint - measurement;

    // Proportional
    const p = this.kp * error;

    // Integral with anti-windup
    this.integral += error * dt;
    this.integral = Math.max(-this.integralMax, Math.min(this.integralMax, this.integral));
    const i = this.ki * this.integral;

    // Derivative on measurement, so a setpoint step does not produce a kick.
    // The first call has no previous sample, so skip it rather than
    // differentiating against a zero that was never measured.
    let derivative = 0;
    if (this.primed) {
      derivative = (measurement - this.lastMeasurement) / dt;
    }

    if (this.derivativeTau > 0) {
      const alpha = dt / (this.derivativeTau + dt);
      this.filteredDerivative += alpha * (derivative - this.filteredDerivative);
    } else {
      this.filteredDerivative = derivative;
    }
    const d = -this.kd * this.filteredDerivative;

    this.lastMeasurement = measurement;
    this.prevError = error;
    this.primed = true;

    const output = p + i + d;
    return Math.max(this.outputMin, Math.min(this.outputMax, output));
  }
}
