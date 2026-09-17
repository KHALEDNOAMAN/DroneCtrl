function c = pid_init(kp, ki, kd, max_integral, max_output, derivative_tau)
% PID_INIT  Matches firmware/include/pid_controller.h.
  if nargin < 6
    derivative_tau = 0.02;
  end
  c.kp = kp; c.ki = ki; c.kd = kd;
  c.max_integral = max_integral;
  c.max_output = max_output;
  c.derivative_tau = derivative_tau;
  c.integral = 0;
  c.prev_measurement = 0;
  c.filtered_derivative = 0;
  c.primed = false;
  c.saturated = false;
end
