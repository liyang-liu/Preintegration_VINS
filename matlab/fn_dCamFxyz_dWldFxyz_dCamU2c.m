function [dxyz,dxyz_u2c, dp0] = fn_dCamFxyz_dWldFxyz_dCamU2c(a, b, g, a_u2c, b_u2c, g_u2c, Ru,...
            Ru2c, Tu, Tu2c, X0, N)
%% X = R * X0 + T: x = r11 * x0 + r12 * y0 + r13 * z0 + t1, 
%%                 y = r21 * x0 + r22 * y0 + r23 * z0 + t2,
%%                 z = r31 * x0 + r32 * y0 + r33 * z0 + t3,
%% X = Ru2c * Ru * (X0 - Ru'* Tu2c - Tu)
%%   = Ru2c * Ru * (X0 - Tu)- Ru2c * Tu2c 
% X0 = (x0,y0,z0)';
% dT = eye(3);
% dT = repmat(dT(:), 1, N); % 9xN

Tu2cN = repmat(Tu2c, 1, N);
TuN = repmat(Tu, 1, N);

%dxyz = sparse(3, 3*N+6*(nPoses-1), N);
[drda, drdb, drdg] = fn_ABG2R_dr(a, b, g);

%R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
% dxyzdabg: to Ru
dxyzda = Ru2c*drda*(X0 - TuN); % 3xN
dxyzdb = Ru2c*drdb*(X0 - TuN); % 3xN
dxyzdg = Ru2c*drdg*(X0 - TuN); % 3xN
dT = -Ru2c * Ru;
dT = repmat(dT(:), 1, N);
dxyz = reshape([dxyzda;dxyzdb;dxyzdg; dT], 3, []);% 3x6N

% dxyzdabg: to Ru2c ---TobeVerified
[drda, drdb, drdg] = fn_ABG2R_dr(a_u2c, b_u2c, g_u2c);
dxyzda = drda * Ru * (X0 - Ru'* Tu2cN - TuN); % 3xN
dxyzdb = drdb * Ru * (X0 - Ru'* Tu2cN - TuN); % 3xN
dxyzdg = drdg * Ru * (X0 - Ru'* Tu2cN - TuN); % 3xN;
dT = -Ru2c;
dT = repmat(dT(:), 1, N);
dxyz_u2c = reshape([dxyzda;dxyzdb;dxyzdg; dT], 3, []);% 3x6xN
dp0 = Ru2c * Ru;
