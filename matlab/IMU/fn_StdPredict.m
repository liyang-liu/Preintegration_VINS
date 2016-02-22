function [p1, v1, phi1] = fn_StdPredict(p0, v0, phi0, dt, fb0, omegab0, bf, bo, gn)
%% Predict the inertial states according to the standard formula
% pi: (xi, yi, zi)'
% vi: (vxi, vyi, vzi)'
% phii: (alphai, betai, gammai)


Cb0n = EulerAngle(phi0);% FunRfrmEulerAngle(phi0);
v1 = v0 + (Cb0n * (fb0 - bf) + gn) * dt;
p1 = p0 + v0*dt;%0.5*(v0+v1) * dt;
Eb0n = Jaccobian_RotationInverse(phi0);% FunRratefrmEulerAngle(phi0);
phi1 = phi0 + Eb0n * (omegab0 - bo) * dt;


