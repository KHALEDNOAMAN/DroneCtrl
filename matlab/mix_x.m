function out = mix_x(base_pwm, pitch_cmd, roll_cmd, yaw_cmd, esc_min, esc_max)
% MIX_X  Saturation-aware X-configuration motor mixer.
%
% The same law as firmware/include/mixer_math.h. Clamping each motor on its
% own is what quietly loses attitude authority at high throttle: once a motor
% hits the ceiling the extra command is discarded and the four outputs no
% longer differ by the torque that was asked for. This protects the
% differential and moves the common throttle instead.

  % A positive command produces a positive response in the axis it names.
  % Positive roll is right side down, which needs more thrust on the LEFT.
  d = [ pitch_cmd + roll_cmd - yaw_cmd;   % FL
        pitch_cmd - roll_cmd + yaw_cmd;   % FR
       -pitch_cmd + roll_cmd + yaw_cmd;   % BL
       -pitch_cmd - roll_cmd - yaw_cmd ]; % BR

  span = max(d) - min(d);
  range = esc_max - esc_min;

  out.authority_scale = 1.0;
  out.authority_limited = false;
  out.throttle_adjusted = false;

  if span > range
    % The torques alone are wider than the ESC range, so shrink every axis by
    % one factor. Magnitude is lost, direction is not.
    out.authority_scale = range / span;
    out.authority_limited = true;
    d = d * out.authority_scale;
  end

  lo = esc_min - min(d);
  hi = esc_max - max(d);
  base = base_pwm;
  if base < lo
    base = lo;
    out.throttle_adjusted = true;
  elseif base > hi
    base = hi;
    out.throttle_adjusted = true;
  end

  out.pwm = (base + d)';
end
