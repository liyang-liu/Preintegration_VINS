function [dxyz] = fxyz5abgxyz_dr(alpha, beta, gamma, X0, N)
%% X = R * X0 + T: x = r11 * x0 + r12 * y0 + r13 * z0 + t1, 
%%                 y = r21 * x0 + r22 * y0 + r23 * z0 + t2,
%%                 z = r31 * x0 + r32 * y0 + r33 * z0 + t3,

% X0 = (x0,y0,z0)';
dT = eye(3);
dT = repmat(dT(:), 1, N); % 9xN
%dxyz = sparse(3, 3*N+6*(nPoses-1), N);
[drda, drdb, drdg] = fabg2r_dr(alpha, beta, gamma);
%R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
dxyzda = drda*X0; % 3xN
dxyzdb = drdb*X0; % 3xN
dxyzdg = drdg*X0; % 3xN
dxyz = reshape([dxyzda;dxyzdb;dxyzdg; dT], 3, []);% 3x6N
%dxyz(:, 1:6) = [drda*X0, drdb*X0, drdg*X0, dT];

