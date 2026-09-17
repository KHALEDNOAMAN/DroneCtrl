function [g, v] = lcg_uniform(g)
% LCG_UNIFORM  Uniform on [-1, 1], bit for bit with the C++ side.
  g.s = mod(g.s * 1664525 + 1013904223, 4294967296);
  v = bitand(floor(g.s / 256), 16777215) / 8388608 - 1;
end
