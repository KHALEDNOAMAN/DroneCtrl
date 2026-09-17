function e = ekf_predict(e, p_meas, q_meas, r_meas, dt)
% EKF_PREDICT  Propagate with the measured body rates.

  if dt <= 0
    return;
  end

  roll  = e.x(1);
  pitch = min(max(e.x(2), -e.max_pitch), e.max_pitch);
  pc = p_meas - e.x(3);
  qc = q_meas - e.x(4);

  sr = sin(roll); cr = cos(roll);
  tp = tan(pitch); cp = cos(pitch);
  sec2 = 1 / cp^2;   % finite, because pitch is clamped short of vertical

  roll_dot  = pc + qc * sr * tp + r_meas * cr * tp;
  pitch_dot = qc * cr - r_meas * sr;

  % Jacobian of the process model.
  F = zeros(4,4);
  F(1,1) = qc * cr * tp - r_meas * sr * tp;
  F(1,2) = (qc * sr + r_meas * cr) * sec2;
  F(1,3) = -1;
  F(1,4) = -sr * tp;
  F(2,1) = -qc * sr - r_meas * cr;
  F(2,4) = -cr;
  % Bias rows stay zero: a random walk has no deterministic drift.

  e.x(1) = wrap_pi(roll + roll_dot * dt);
  e.x(2) = min(max(pitch + pitch_dot * dt, -e.max_pitch), e.max_pitch);

  % Discrete transition, first order. At 250 Hz the neglected term is of order
  % dt^2 = 1.6e-5, far below the process noise.
  Phi = eye(4) + F * dt;
  Q = diag([e.gyro_noise * dt, e.gyro_noise * dt, ...
            e.bias_noise * dt, e.bias_noise * dt]);

  e.P = Phi * e.P * Phi' + Q;
  e.P = 0.5 * (e.P + e.P');   % rounding pulls the two triangles apart
end

function a = wrap_pi(a)
  while a > pi
    a = a - 2*pi;
  end
  while a < -pi
    a = a + 2*pi;
  end
end
