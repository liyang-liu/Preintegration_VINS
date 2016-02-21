function [u, v] = fxyz2uv(x, y, z, f, x0, y0)
%% [u, v, 1]' = K * X: u = f * x / z + x0, v = f * y / z + y0
u = f * x / z + x0;
v = f * y / z + y0;