function log = run_mil(scenario, duration)
% RUN_MIL  Model in the loop: the plant, estimator and controller all in
% MATLAB, run as one closed loop.
%
% The counterpart to firmware/sil, which runs the real C++ flight code against
% the same plant. Having both is the point. This one is where the maths is
% readable and a gain or a pole can be changed in a line; that one is what
% actually flies. verify_mil_sil.m checks they still agree, which is the
% MIL and SIL equivalence step a model-based workflow turns on.
%
%   scenario  'hover' | 'gyro_bias' | 'roll_step'
%   duration  seconds
%
% Returns a log struct with the same fields the C++ harness writes to CSV.

  if nargin < 2
    duration = 20;
  end
  dt = 1/250;
  deg = 180/pi;

  % Gains straight from the derivation. tune_gains.m produces these, and
  % config.h carries the same numbers.
  KP = 2.7; KI = 1.8; KD = 0.55;
  YAW_KP = 4.0; YAW_KI = 0.5; YAW_KD = 0.0;

  gyro_bias_dps = 0;
  roll_step_at = -1;
  roll_step_deg = 0;
  switch scenario
    case 'hover'
      % nothing extra
    case 'gyro_bias'
      gyro_bias_dps = 3.0;
    case 'roll_step'
      roll_step_at = 8.0;
      roll_step_deg = 15.0;
    otherwise
      error('unknown scenario: %s', scenario);
  end

  p = quad_init();
  e = ekf_init();
  e = ekf_seed(e, 0, 0, 1);
  g = lcg_init(20260917);

  pid_roll  = pid_init(KP, KI, KD, 100, 400);
  pid_pitch = pid_init(KP, KI, KD, 100, 400);
  pid_yaw   = pid_init(YAW_KP, YAW_KI, YAW_KD, 100, 400);

  bias_p = gyro_bias_dps / deg;
  bias_q = -0.5 * gyro_bias_dps / deg;
  gyro_noise = 0.015;    % rad/s, one sigma
  accel_noise = 0.03;    % g

  armed = false;
  arm_timer = 0;
  authority_limited = false;

  n = round(duration / dt);
  log = struct('t', zeros(n,1), 'roll_true', zeros(n,1), 'pitch_true', zeros(n,1), ...
               'roll_est', zeros(n,1), 'pitch_est', zeros(n,1), 'bias_p_est', zeros(n,1), ...
               'z', zeros(n,1), 'pwm', zeros(n,4));

  for k = 1:n
    t = (k-1) * dt;

    % Pilot script, identical to the C++ harness.
    if t < 2.5
      throttle = 0; yaw_stick = 1;
    elseif t < 3.0
      throttle = 0.42; yaw_stick = 0;
    else
      throttle = 0.378; yaw_stick = 0;   % hover for this airframe
    end
    roll_stick = 0;
    if roll_step_at >= 0 && t >= roll_step_at
      roll_stick = roll_step_deg / 30;
    end

    % Arming: the gesture held with throttle down for two seconds. The full
    % failsafe table lives in flight_state.h and is covered by the C++ unit
    % tests; what is needed here is only enough of it to reproduce the same
    % trajectory.
    if ~armed
      if throttle <= 0.05 && yaw_stick > 0.8
        arm_timer = arm_timer + dt;
        if arm_timer >= 2.0
          armed = true;
        end
      else
        arm_timer = 0;
      end
    end

    % Sensors.
    [g, n1] = lcg_gauss(g); [g, n2] = lcg_gauss(g); [g, n3] = lcg_gauss(g);
    gx = p.p + bias_p + gyro_noise * n1;
    gy = p.q + bias_q + gyro_noise * n2;
    gz = p.r + gyro_noise * n3;

    sr = sin(p.roll); cr = cos(p.roll);
    sp = sin(p.pitch); cp = cos(p.pitch);
    specific = sum(p.thrust) / p.cfg.mass / p.cfg.g;
    [g, n4] = lcg_gauss(g); [g, n5] = lcg_gauss(g); [g, n6] = lcg_gauss(g);
    ax = -sp * specific + accel_noise * n4;
    ay = sr * cp * specific + accel_noise * n5;
    az = cr * cp * specific + accel_noise * n6;

    % Estimator.
    e = ekf_predict(e, gx, gy, gz, dt);
    e = ekf_update_accel(e, ax, ay, az);

    roll_est = e.x(1) * deg;
    pitch_est = e.x(2) * deg;

    if ~armed
      pid_roll = pid_init(KP, KI, KD, 100, 400);
      pid_pitch = pid_init(KP, KI, KD, 100, 400);
      pid_yaw = pid_init(YAW_KP, YAW_KI, YAW_KD, 100, 400);
      pwm = ones(1,4) * p.cfg.esc_min;
    else
      % Setpoints from the sticks, in the same units the firmware uses:
      % 30 degrees of tilt and 150 deg/s of yaw rate at full deflection. The
      % yaw term matters more than it looks: the arming gesture holds full
      % yaw for the first two and a half seconds, so leaving it out of the
      % model makes the airframe behave differently from the aircraft during
      % exactly the moment the motors come up. That was a real transcription
      % error here, and verify_mil_sil.m is what found it.
      [pid_roll,  out_roll]  = pid_step(pid_roll,  roll_stick * 30,  roll_est,  dt, authority_limited);
      [pid_pitch, out_pitch] = pid_step(pid_pitch, 0,                pitch_est, dt, authority_limited);
      [pid_yaw,   out_yaw]   = pid_step(pid_yaw,   yaw_stick * 150,  gz * deg,  dt, authority_limited);

      base = p.cfg.esc_idle + throttle * (p.cfg.esc_max - p.cfg.esc_idle);
      o = mix_x(base, out_pitch, out_roll, out_yaw, p.cfg.esc_min, p.cfg.esc_max);
      authority_limited = o.authority_limited;
      pwm = o.pwm;
    end

    p = quad_step(p, pwm, 0, dt);

    log.t(k) = t;
    log.roll_true(k) = p.roll * deg;
    log.pitch_true(k) = p.pitch * deg;
    log.roll_est(k) = roll_est;
    log.pitch_est(k) = pitch_est;
    log.bias_p_est(k) = e.x(3) * deg;
    log.z(k) = p.z;
    log.pwm(k,:) = pwm;
  end
end
