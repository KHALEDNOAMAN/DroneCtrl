function e = ekf_update_accel(e, ax, ay, az)
% EKF_UPDATE_ACCEL  Correct with one accelerometer sample, in g.
%
% The accelerometer is treated as an observation of the gravity vector in body
% axes rather than being pre-converted into angles. Converting first would
% bake in a linearisation the filter never sees, and would throw away the
% vector's length, which is what the trust check below is built on.

  n = sqrt(ax^2 + ay^2 + az^2);
  if n < 1e-3
    return;   % free fall or a dead sensor, nothing to use
  end

  roll  = e.x(1);
  pitch = min(max(e.x(2), -e.max_pitch), e.max_pitch);
  sr = sin(roll); cr = cos(roll);
  sp = sin(pitch); cp = cos(pitch);
  g = e.gravity;

  h = [-g*sp; g*sr*cp; g*cr*cp];

  % The bias columns are zero: the accelerometer cannot see gyro bias
  % directly. It is still estimated, through the correlation P builds up
  % between the angle and bias states during predict.
  H = zeros(3,4);
  H(1,2) = -g*cp;
  H(2,1) =  g*cr*cp;  H(2,2) = -g*sr*sp;
  H(3,1) = -g*sr*cp;  H(3,2) = -g*cr*sp;

  % The model assumes the only specific force is gravity, which on a
  % quadcopter is false most of the time. The measured magnitude is the
  % evidence available, so R is inflated in proportion rather than the sample
  % being dropped, which would discard the partial information in a mildly
  % disturbed reading and make the behaviour discontinuous.
  scale = 1 + e.accel_reject_gain * abs(n - g);
  R = eye(3) * (e.accel_noise * scale^2);

  S = H * e.P * H' + R;
  if abs(det(S)) < 1e-12
    return;
  end
  K = e.P * H' / S;

  y = [ax; ay; az] - h;
  e.innovation = y;

  e.x = e.x + K * y;
  e.x(1) = wrap_pi(e.x(1));
  e.x(2) = min(max(e.x(2), -e.max_pitch), e.max_pitch);

  % Joseph form. The short (I-KH)P loses symmetry and positive-definiteness to
  % rounding, which on 32-bit floats at 250 Hz shows up as a filter that
  % quietly stops correcting after a few minutes.
  IKH = eye(4) - K * H;
  e.P = IKH * e.P * IKH' + K * R * K';
  e.P = 0.5 * (e.P + e.P');
end

function a = wrap_pi(a)
  while a > pi
    a = a - 2*pi;
  end
  while a < -pi
    a = a + 2*pi;
  end
end
