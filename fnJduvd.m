function J = fnJduvd(K, x, nPoses, N)
%% Objective function elements: ei = (ui' - ui)^2, ei'= (vi' - vi)^2 (i=1...N)
% Find R, T corresponding to 3D points pi and pi'.
% 
% K: camera model
% p3d0: 3D points at the first camera pose
% x: current estimate of the states (alpha, beta, gamma, T)

f = K(1,1); cx0 = K(1,3); cy0 = K(2,3);
%alpha = x(1); beta = x(2); gamma = x(3); T = x(4:6, 1);
%N = size(p3d0, 2);
J = sparse(3*N*nPoses, 3*N+6*(nPoses-1));

% Section for pose 1
p3d0 = reshape(x(((nPoses-1)*6+1):end, 1), 3, []);
x0 = (p3d0(1,:))';
y0 = (p3d0(2,:))';
z0 = (p3d0(3,:))';
du = [f./z0, zeros(N,1),-f*x0./(z0.*z0)]; % Nx3
dv = [zeros(N,1), f./z0, -f*y0./(z0.*z0)];
dd = repmat([0, 0, 1], N, 1);
duvd = [du'; dv'; dd'];% 9xN
duvd = (reshape(duvd, 3, []))'; % 3Nx3
for i = 1:N
    J((3*(i-1)+1):(3*i), (6*(nPoses-1)+3*(i-1)+1):(6*(nPoses-1)+3*i)) = duvd((3*(i-1)+1):(3*i),:);
end
% Section for the rest poses
for pid=2:nPoses
    alpha = x(1+(pid-2)*6); beta = x(2+(pid-2)*6); gamma = x(3+(pid-2)*6); 
    T1 = x((4+(pid-2)*6):((pid-1)*6), 1);    
    R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
    p3d1 = R1 * p3d0 + repmat(T1, 1, N);    
    % Find the gradient of u(x,y,z) and v(x,y,z) at the second pose
	[duvd] = fnuvd5xyz_dr(p3d1, f, N);%[duvd] = fxyz2uvd_dr(p3d1(1,i), p3d1(2,i), p3d1(3,i), f);
	% Find the gradient of xyz(alpha, beta, gamma, T) at the second pose
	[dxyz] = fxyz5abgxyz_dr(alpha, beta, gamma, p3d0, N);
    duvddabgxyz = zeros(3*N, 3*N+6*(nPoses-1));
    for i = 1:N
        dabgxyz = sparse(3, 3*N+6*(nPoses-1));
        dabgxyz(:,(6*(pid-2)+1):(6*(pid-1))) = dxyz(:,(6*(i-1)+1):(6*i));
        dabgxyz(:, (6*(nPoses-1)+3*(i-1)+1):(6*(nPoses-1)+3*i)) = R1;
        duvddabgxyz((3*(i-1)+1):(3*i), :) = duvd(((i-1)*3+1):(3*i),:) * dabgxyz;        
    end
    J((3*N*(pid-1)+1):(3*N*pid),:) = duvddabgxyz;
end

