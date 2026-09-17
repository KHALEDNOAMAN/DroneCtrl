function p = quad_init()
% QUAD_INIT  Plant state and airframe parameters.
%
% These numbers are the ones in firmware/sil/sil_flight.cpp. They describe a
% 1 kg quadcopter with a 0.15 m arm and 6 N motors, which is a 450 class
% airframe. Change them and the gains in tune_gains.m change with them, which
% is the point of deriving the gains rather than guessing.

  p.cfg = struct( ...
    'mass',        1.0,    ...  % kg
    'arm',         0.15,   ...  % m, centre to motor
    'inertia_xx',  0.011,  ...  % kg m^2
    'inertia_yy',  0.011,  ...
    'inertia_zz',  0.021,  ...
    'max_thrust',  6.0,    ...  % N per motor
    'motor_tau',   0.03,   ...  % s, ESC and propeller spin-up
    'rot_damping', 0.02,   ...  % N m per rad/s
    'yaw_torque_k',0.02,   ...  % N m per N of differential thrust
    'lin_drag',    0.35,   ...  % N per m/s
    'g',           9.81,   ...
    'esc_min',     1000,   ...
    'esc_max',     2000,   ...
    'esc_idle',    1050);

  p.roll = 0; p.pitch = 0; p.yaw = 0;
  p.p = 0; p.q = 0; p.r = 0;
  p.x = 0; p.y = 0; p.z = 0;
  p.vx = 0; p.vy = 0; p.vz = 0;
  p.thrust = [0 0 0 0];
  p.motor_health = [1 1 1 1];
end
