function p = quad_step(p, pwm, wind_torque_roll, dt)
% QUAD_STEP  Advance the quadcopter plant by one timestep.
%
% Deliberately the same model as firmware/sil/sil_flight.cpp, parameter for
% parameter, because the point of having it twice is to check the two against
% each other. verify_mil_sil.m is what does that check; if this file and the
% C++ plant drift apart, that script fails.
%
%   p    plant state struct, from quad_init()
%   pwm  [FL FR BL BR] ESC commands in microseconds
%   wind_torque_roll  external roll disturbance, N m
%   dt   timestep, seconds
%
% Frame convention, the same one the estimator and the mixer use:
%   x forward, y right, z up. Positive roll is right side down, positive pitch
%   is nose up.

  c = p.cfg;

  % ESC command to commanded thrust, then a first order lag toward it. The lag
  % is the ESC and the propeller's own inertia, and leaving it out is how a
  % model ends up endorsing gains that oscillate on real hardware.
  u = (pwm - c.esc_min) / (c.esc_max - c.esc_min);
  u(u < 0) = 0;
  commanded = c.max_thrust * u .* p.motor_health;
  a = dt / (c.motor_tau + dt);
  p.thrust = p.thrust + a * (commanded - p.thrust);

  t_fl = p.thrust(1); t_fr = p.thrust(2);
  t_bl = p.thrust(3); t_br = p.thrust(4);
  total = sum(p.thrust);

  % On an X frame every motor sits at 45 degrees to both axes, so its moment
  % arm about either one is the geometric arm times cos(45).
  arm_eff = c.arm * 0.70710678;

  tau_roll  = arm_eff * ((t_fl + t_bl) - (t_fr + t_br)) - c.rot_damping * p.p + wind_torque_roll;
  tau_pitch = arm_eff * ((t_fl + t_fr) - (t_bl + t_br)) - c.rot_damping * p.q;
  tau_yaw   = c.yaw_torque_k * ((t_fr + t_bl) - (t_fl + t_br)) - c.rot_damping * p.r;

  p.p = p.p + (tau_roll  / c.inertia_xx) * dt;
  p.q = p.q + (tau_pitch / c.inertia_yy) * dt;
  p.r = p.r + (tau_yaw   / c.inertia_zz) * dt;

  % Euler kinematics from body rates.
  sr = sin(p.roll); cr = cos(p.roll);
  sp = sin(p.pitch); cp = cos(p.pitch); tp = tan(p.pitch);

  p.roll  = p.roll  + (p.p + p.q * sr * tp + p.r * cr * tp) * dt;
  p.pitch = p.pitch + (p.q * cr - p.r * sr) * dt;
  if cp == 0
    cp_safe = 1e-4;
  else
    cp_safe = cp;
  end
  p.yaw = p.yaw + ((p.q * sr + p.r * cr) / cp_safe) * dt;

  % Thrust acts along body up. Rotated into the world it is the only force
  % besides gravity and drag.
  sp = sin(p.pitch); cp = cos(p.pitch);
  sr = sin(p.roll);  cr = cos(p.roll);
  sy = sin(p.yaw);   cy = cos(p.yaw);

  tx = total * (sp * cy + sr * cp * sy);
  ty = total * (sp * sy - sr * cp * cy);
  tz = total * (cr * cp);

  fx = tx - c.lin_drag * p.vx;
  fy = ty - c.lin_drag * p.vy;
  fz = tz - c.lin_drag * p.vz - c.mass * c.g;

  p.vx = p.vx + (fx / c.mass) * dt;
  p.vy = p.vy + (fy / c.mass) * dt;
  p.vz = p.vz + (fz / c.mass) * dt;
  p.x = p.x + p.vx * dt;
  p.y = p.y + p.vy * dt;
  p.z = p.z + p.vz * dt;

  if p.z <= 0
    p.z = 0;
    if p.vz < 0
      p.vz = 0;
    end
  end
end
