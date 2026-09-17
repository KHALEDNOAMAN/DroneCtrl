function e = ekf_seed(e, ax, ay, az)
% EKF_SEED  Start from one accelerometer sample rather than from zero.
% Worth doing once the airframe is known to be still, straight after gyro
% calibration, so the filter begins near the answer.

  n = sqrt(ax^2 + ay^2 + az^2);
  if n < 1e-3
    return;
  end
  e.x(1) = atan2(ay, az);
  e.x(2) = atan2(-ax, sqrt(ay^2 + az^2));
  e.P(1,1) = 1e-2;
  e.P(2,2) = 1e-2;
end
