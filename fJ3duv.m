function J = fJ3duv(K, x)
%% Objective function elements: ei = (ui' - ui)^2, ei'= (vi' - vi)^2 (i=1...N)
% Find R, T corresponding to 3D points pi and pi'.
% 
% K: camera model
% p3d0: 3D points at the first camera pose
% x: current estimate of the states (alpha, beta, gamma, T)

f = K(1,1); cx0 = K(1,3); cy0 = K(2,3);
alpha = x(1); beta = x(2); gamma = x(3); T = x(4:6, 1);
p3d0 = reshape(x(7:end, 1), 3, []);
N = size(p3d0, 2);
J = sparse(6*N, 3*N+6);

R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
p3d1 = R1 * p3d0 + repmat(T, 1, N);

for i=1:N
	% Find the gradient of u(x,y,z) and v(x,y,z) at the first pose
	[duvd] = fxyz2uvd_dr(p3d0(1,i), p3d0(2,i), p3d0(3,i), f);
	J((1+3*(i-1)):(3*i), (7+(i-1)*3):(6+3*i)) = duvd;
    % Find the gradient of u(x,y,z) and v(x,y,z) at the second pose
	[duvd] = fxyz2uvd_dr(p3d1(1,i), p3d1(2,i), p3d1(3,i), f);
	% Find the gradient of xyz(alpha, beta, gamma, T) at the second pose
	[dxyz] = fxyz2xyz_dr(alpha, beta, gamma, p3d0(:,i), N);
    dxyz(:, (7+3*(i-1)):(6+3*i)) = R1;
	% Fill in J(i,:) & J(i+N, :)
	J((3*N+1+3*(i-1)):(3*N+3*i),:) = duvd * dxyz;
end

