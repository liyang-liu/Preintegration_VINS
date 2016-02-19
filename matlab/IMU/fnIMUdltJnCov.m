function [J1, R1] = fnIMUdltJnCov(Q, J0, R0, omega0, phi0, dt, fb0, bf, bo)
%% Calculate standard inertial prediction covariance
%
% Input:
% P0: 15x15, prior covariance matrix
% omega0: 1x3, rotation rate
% phi0: 1x3, Euler angles
% dt: time interval
%
% Output:
% P: 15x15, updated covariance matrix

%
alpha = fndRdphi(phi0, (fb0-bf));
beta =  fndEdphi(phi0, (omega0 - bo));
Cb0n = (fnR5ABG(phi0(1), phi0(2), phi0(3)))';% EulerAngle(phi0);
% F
F = zeros(15);
F(1:3, 1:3) = eye(3);
F(4:6, 4:6) = eye(3);
F(7:9, 7:9) = dt * beta + eye(3);
F(10:12, 10:12) = eye(3);
F(13:15, 13:15) = eye(3);
F(1:3, 4:6) = dt * eye(3);
F(1:3, 7:9) = 0.5*dt*dt*alpha;
F(1:3, 10:12) = -0.5*dt*dt*Cb0n;

F(4:6, 7:9) = dt * alpha;
F(4:6, 10:12) = - dt * Cb0n;
Eb0n = Jac_koInv(phi0);%Jaccobian_RotationInverse(phi0)
F(7:9, 13:15) = - dt * Eb0n;
% G
G = zeros(15, 6);
G(4:6, 1:3) = dt * Cb0n;
G(7:9, 4:6) = dt *Eb0n;

J1 = F * J0;
R1 = F * R0 * F' + G * Q * G';



