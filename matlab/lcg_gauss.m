function [g, v] = lcg_gauss(g)
% LCG_GAUSS  Three uniforms summed, matching the C++ harness exactly.
  [g, a] = lcg_uniform(g);
  [g, b] = lcg_uniform(g);
  [g, c] = lcg_uniform(g);
  v = a + b + c;
end
