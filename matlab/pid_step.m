function [c, out] = pid_step(c, setpoint, measurement, dt, hold_integral)
% PID_STEP  One control step, derivative on measurement.
%
% hold_integral is raised by the caller when the mixer could not deliver last
% step's torque. Integrating against an actuator already at its limit only
% builds a correction that has to be unwound, which is what turns a brief
% saturation into an overshoot once it clears.

  if nargin < 5
    hold_integral = false;
  end
  out = 0;
  if dt <= 0
    return;
  end

  err = setpoint - measurement;

  if ~hold_integral
    c.integral = c.integral + err * dt;
    c.integral = min(max(c.integral, -c.max_integral), c.max_integral);
  end

  derivative = 0;
  if c.primed
    derivative = (measurement - c.prev_measurement) / dt;
  else
    % First call. prev_measurement is not a real previous sample yet, so
    % differencing against it would produce a spike from a standing start.
    c.primed = true;
  end

  if c.derivative_tau > 0
    % A time constant, not a fixed smoothing factor, so the cutoff does not
    % move when the loop rate does.
    alpha = dt / (c.derivative_tau + dt);
    c.filtered_derivative = c.filtered_derivative + alpha * (derivative - c.filtered_derivative);
  else
    c.filtered_derivative = derivative;
  end

  c.prev_measurement = measurement;

  out = c.kp * err + c.ki * c.integral - c.kd * c.filtered_derivative;
  c.saturated = abs(out) > c.max_output;
  out = min(max(out, -c.max_output), c.max_output);
end
