function [duvd] = fn_dUv_dCamFxyz_dr(p3d, fx, fy, N)
%% Compute the gradients of u(x,y,z) and v(x,y,z) in one go.
% [u, v, 1]' = K * X: u = fx * x / z + x0, v = fy * y / z + y0
% du = fx * dx / z - fx * x / z^2 * dz, 
% dv = fy / z * dy - fy * y / z^2 * dz

x = (p3d(1,:))';
y = (p3d(2,:))';
z = (p3d(3,:))';
du = [fx./z, zeros(N,1), -fx*x./(z.*z)];% Nx3
dv = [zeros(N,1), fy./z, -fy*y./(z.*z)];
dd = repmat([0, 0, 1], N, 1);
duvd = [du'; dv'; dd'];%9xN
duvd = (reshape(duvd, 3, []))';%3Nx3
