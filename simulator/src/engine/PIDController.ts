export class PIDController {
  private kp: number;
  private ki: number;
  private kd: number;
  
  private outputMin: number;
  private outputMax: number;
  private integralMax: number;
  
  private integral: number = 0;
  private prevError: number = 0;
  private lastMeasurement: number = 0;

  constructor(kp: number, ki: number, kd: number, outputMin: number, outputMax: number, integralMax: number) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.outputMin = outputMin;
    this.outputMax = outputMax;
    this.integralMax = integralMax;
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
    
    // Derivative (on measurement to avoid derivative kick on setpoint change)
    const derivative = (measurement - this.lastMeasurement) / dt;
    const d = -this.kd * derivative;
    
    this.lastMeasurement = measurement;
    this.prevError = error;
    
    let output = p + i + d;
    return Math.max(this.outputMin, Math.min(this.outputMax, output));
  }
}
