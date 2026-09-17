% TUNE_GAINS  Derive the attitude PID gains from the airframe, and print them
% in the form firmware/include/config.h expects.
%
% This exists so the claim that the gains are derived rather than guessed is
% executable. Change the airframe in quad_init.m, run this, and the numbers
% that come out are the ones to paste into config.h.
%
% Two steps: identify the plant numerically, then place the closed-loop poles.

clear; close all;
dt = 1/250;

%% Step 1: plant identification
%
% Measured rather than derived on paper, so an error in quad_step.m shows up
% here instead of being reproduced by an equation written to match it.
%
% Two things have to be kept out of the measurement, and both bite:
%
%   Motor lag. The ESC and propeller take about 30 ms to reach a commanded
%   thrust, so the acceleration in the first few steps is the lag's, not the
%   airframe's. Handled by pre-loading the motors with the thrust the command
%   implies, so the step starts from a settled actuator.
%
%   Aerodynamic damping. The torque is K*cmd - damping*p, so any measurement
%   taken once the rate has built up reads low. Taking it at p = 0 removes the
%   term entirely. Measuring the slope a second in instead gives about 2.5
%   rather than 13.3, because by then the airframe has rolled 145 degrees and
%   left the linear regime completely.

cmd = 20;                    % microseconds of roll command
p = quad_init();
base = p.cfg.esc_idle + 0.378 * (p.cfg.esc_max - p.cfg.esc_idle);
o = mix_x(base, 0, cmd, 0, p.cfg.esc_min, p.cfg.esc_max);

% Pre-load the motors so the first step measures the airframe, not the lag.
u = (o.pwm - p.cfg.esc_min) / (p.cfg.esc_max - p.cfg.esc_min);
p.thrust = p.cfg.max_thrust * u;

% One step from rest. p starts at zero, so the damping term is zero and what
% is left is the command gain alone.
p = quad_step(p, o.pwm, 0, dt);
K_plant = ((p.p / dt) * 180/pi) / cmd;    % deg/s^2 per microsecond

% The airframe's own aerodynamic damping, as a rate.
damping = p.cfg.rot_damping / p.cfg.inertia_xx;   % 1/s

% Cross-check against the closed form, which is what the comments in
% config.h and docs/verification.md quote. If these disagree, one of the two
% is wrong and the gains below inherit the error.
arm_eff = p.cfg.arm * 0.70710678;
thrust_per_us = p.cfg.max_thrust / (p.cfg.esc_max - p.cfg.esc_min);
K_closed_form = (arm_eff * 4 * thrust_per_us / p.cfg.inertia_xx) * 180/pi;

fprintf('Plant identification\n');
fprintf('  roll command gain   %.2f deg/s^2 per microsecond (measured)\n', K_plant);
fprintf('  closed form         %.2f deg/s^2 per microsecond\n', K_closed_form);
fprintf('  aerodynamic damping %.2f 1/s\n', damping);
if abs(K_plant - K_closed_form) / K_closed_form > 0.02
  error('measured plant gain disagrees with the closed form by more than 2 percent');
end
fprintf('  agreement           within 2 percent, ok\n\n');

%% Step 2: pole placement
%
% Treat the inner loop as second order:
%   omega_n^2      = K_plant * Kp
%   2 zeta omega_n = K_plant * Kd + damping
%
% omega_n = 6 rad/s is a normal attitude bandwidth for a quadcopter this size
% and keeps the 0.03 s motor lag pole, at 33 rad/s, an order of magnitude
% above the crossover where it cannot destabilise the loop. zeta = 0.75 trades
% a little overshoot for settling time.

omega_n = 6.0;
zeta = 0.75;

Kp = omega_n^2 / K_plant;
Kd = (2 * zeta * omega_n - damping) / K_plant;

% Integral time of about 1.5 s: fast enough to trim out a degraded motor,
% slow enough not to interact with the attitude loop it sits outside.
Ti = 1.5;
Ki = Kp / Ti;

fprintf('Closed loop design, omega_n = %.1f rad/s, zeta = %.2f\n', omega_n, zeta);
fprintf('  PID_ROLL_KP = %.2ff\n', Kp);
fprintf('  PID_ROLL_KI = %.2ff\n', Ki);
fprintf('  PID_ROLL_KD = %.2ff\n\n', Kd);

%% Step 3: what those poles actually mean
s = roots([1, 2*zeta*omega_n, omega_n^2]);
overshoot = 100 * exp(-pi*zeta / sqrt(1 - zeta^2));
ts = 4 / (zeta * omega_n);
fprintf('Predicted second order response\n');
fprintf('  closed loop poles   %.2f +/- %.2fi\n', real(s(1)), abs(imag(s(1))));
fprintf('  overshoot           %.0f %%\n', overshoot);
fprintf('  2%% settling time    %.2f s\n\n', ts);

fprintf('These are for the airframe in quad_init.m. A different mass, arm or\n');
fprintf('motor changes K_plant, so rerun this rather than reusing the numbers.\n');
