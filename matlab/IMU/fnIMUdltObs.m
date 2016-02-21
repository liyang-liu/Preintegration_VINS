function [dp1, dv1, dphi1] = fnIMUdltObs(dp0, dv0, dphi0, dt, fb0, omega0, bf, bo)
%% Predict the inertial states according to the standard formula
% pi: (xi, yi, zi)'
% vi: (vxi, vyi, vzi)'
% phii: (alphai, betai, gammai)

Cb0 = (fnRFromABG(dphi0(1), dphi0(2), dphi0(3)))';% EulerAngle(dphi0) FunRfrmEulerAngle(phi0);
f0 = Cb0 * (fb0 - bf);
dv1 = dv0 + f0 * dt;
dp1 = dp0 + 0.5*(dv0+dv1) * dt;
Eb0 = Jac_koInv(dphi0);% FunRratefrmEulerAngle(phi0);
dphi1 = dphi0 + Eb0 * (omega0 - bo) * dt;


