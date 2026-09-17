function e = ekf_init()
% EKF_INIT  Four-state attitude filter.
%
% State: [roll; pitch; bias_p; bias_q] in radians and radians per second.
% The same filter as firmware/include/attitude_ekf.h, written in the matrix
% notation the algorithm is actually stated in.
%
% The C++ version hand-rolls every product into fixed-size arrays because it
% has to run on an ATmega328 with 2 kB of RAM and no matrix library. This one
% does not, so it reads the way the derivation does. Having both is the point:
% the readable one is where the maths is checked, the hand-rolled one is what
% flies, and verify_mil_sil.m is what proves they still agree.
%
% Yaw is deliberately not a state. One accelerometer observes gravity, and
% gravity says nothing about heading, so a yaw state here would be
% unobservable and would drift without bound.

  e.x = [0; 0; 0; 0];
  e.P = diag([1.0, 1.0, 1e-2, 1e-2]);

  e.gyro_noise = 4.0e-4;        % (rad/s)^2
  e.bias_noise = 1.0e-7;        % (rad/s^2)^2
  e.accel_noise = 4.0e-2;       % g^2
  e.accel_reject_gain = 40.0;   % per g of departure from 1 g
  e.max_pitch = 1.45;           % rad, keeps tan(pitch) finite
  e.gravity = 1.0;              % the accelerometer is fed in g
  e.innovation = [0; 0; 0];
end
