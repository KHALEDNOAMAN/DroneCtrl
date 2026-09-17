function g = lcg_init(seed)
% LCG_INIT  The same linear congruential generator the C++ harness uses.
%
% Reimplemented here on purpose. Sharing the generator is what makes
% verify_mil_sil.m a real equivalence check: both models then see the same
% noise sample for sample, so any difference in the output is a difference in
% the model rather than a difference in the dice.
%
% Constants are Numerical Recipes'. Arithmetic is done in doubles because the
% product stays under 2^53 and is therefore exact, where MATLAB's uint32 would
% saturate on overflow instead of wrapping.
  g.s = seed;
end
