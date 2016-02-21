function [duvd] = fxyz2uvd_dr(x, y, z, f)
%% Compute the gradients of u(x,y,z) and v(x,y,z) in one go.
% [u, v, 1]' = K * X: u = f * x / z + x0, v = f * y / z + y0
% du = f * dx / z - f * x / z^2 * dz, 
% dv = f / z * dy - f * y / z^2 * dz

du = [f/z, 0,-f*x/z^2];
dv = [0, f/z, -f*y/z^2];
dd = [0, 0, 1];
duvd = [du; dv; dd];
