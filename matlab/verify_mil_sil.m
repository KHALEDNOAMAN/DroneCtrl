% VERIFY_MIL_SIL  Check the MATLAB model against the C++ flight code.
%
% This is the step that makes having two implementations worth the trouble.
% The C++ side in firmware/sil runs the real flight source against the plant;
% this side runs an independent implementation of the same plant, estimator,
% controller and mixer. Both are driven by the same pseudo-random generator,
% so they see identical noise sample for sample and any divergence is a
% difference in the models rather than in the dice.
%
% What it can catch: a transcription error in either direction, a sign that
% was fixed on one side only, a gain that was changed in config.h but not in
% the model, a Jacobian term that is wrong in the hand-rolled C++ but right in
% the matrix version, or vice versa.
%
% What it cannot: an error both implementations share. Two copies of the same
% misunderstanding agree perfectly. This checks consistency, not correctness,
% and the unit tests against a synthetic truth trajectory are what check the
% second thing.
%
% Run firmware/sil first so the CSV logs exist:
%   cd firmware/sil && make run
%   cd ../../matlab && octave-cli verify_mil_sil.m

clear;
log_dir = '../firmware/sil/logs';

% Only the scenarios both sides implement. The fault-injection cases live on
% the C++ side alone, because the full failsafe table is discrete logic that
% the unit tests already cover branch by branch.
scenarios = {'hover', 'gyro_bias', 'roll_step'};
durations = [20, 40, 20];

% Thresholds. The two implementations are not expected to agree to the bit:
% the firmware works in 32-bit floats and MATLAB in doubles, and in a closed
% loop that difference compounds. What should hold is that they agree far more
% closely than either agrees with a wrong model.
rms_limit = 0.5;    % degrees
max_limit = 2.0;    % degrees

fprintf('MIL and SIL equivalence check\n');
fprintf('  MATLAB model against the C++ flight code, same plant, same noise\n\n');

failures = 0;
for i = 1:numel(scenarios)
  name = scenarios{i};
  path = fullfile(log_dir, [name '.csv']);
  if exist(path, 'file') ~= 2
    fprintf('  %-12s SKIP  no log at %s, run firmware/sil first\n', name, path);
    failures = failures + 1;
    continue;
  end

  sil = csvread(path, 1, 0);   % header row skipped
  t_sil = sil(:,1);
  roll_true_sil = sil(:,2);
  roll_est_sil  = sil(:,4);

  mil = run_mil(name, durations(i));

  % The C++ harness logs at 50 Hz, the model runs at 250 Hz. Sample the model
  % at the log's own timestamps rather than assuming the two line up.
  roll_true_mil = interp1(mil.t, mil.roll_true, t_sil, 'linear', 'extrap');
  roll_est_mil  = interp1(mil.t, mil.roll_est,  t_sil, 'linear', 'extrap');

  d_true = roll_true_mil - roll_true_sil;
  d_est  = roll_est_mil  - roll_est_sil;

  rms_true = sqrt(mean(d_true.^2));
  rms_est  = sqrt(mean(d_est.^2));
  max_true = max(abs(d_true));
  max_est  = max(abs(d_est));

  ok = rms_true < rms_limit && rms_est < rms_limit && ...
       max_true < max_limit && max_est < max_limit;
  if ~ok
    failures = failures + 1;
  end

  if ok
    verdict = 'pass';
  else
    verdict = 'FAIL';
  end
  fprintf('  %-12s %s\n', name, verdict);
  fprintf('      true roll  rms %.4f deg, max %.4f deg\n', rms_true, max_true);
  fprintf('      estimate   rms %.4f deg, max %.4f deg\n', rms_est, max_est);
end

fprintf('\nthresholds: rms < %.2f deg, max < %.2f deg\n', rms_limit, max_limit);
if failures > 0
  fprintf('%d of %d scenarios FAILED\n', failures, numel(scenarios));
  exit(1);
else
  fprintf('all %d scenarios agree\n', numel(scenarios));
end
