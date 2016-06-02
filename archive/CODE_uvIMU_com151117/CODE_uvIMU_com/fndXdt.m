function [ dxdt ] = fndXdt( t, x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

load('SimuPoses.mat');

theta=x(1:3);
% position=x(4:6);
v=x(7:9); % v is the linear velocity expressed in the global frame

[data5IMU] = fnSimuIMU_test(t);

angularv=data5IMU(1:3); % angularv is the angular velocity expressed in the body frame
acc=data5IMU(4:6); % acc is the accelaration of the body relative to the global frame, expressed in the global frame


dtdtheta=Jac_koInv(theta)*(angularv-bw0);
dtdposition=v;
dtdv=(fnR5ABG(theta(1), theta(2), theta(3)))'*(acc-bf0)+g0;%EulerAngle(theta)%(fnR5ABG_zyx(theta(1), theta(2), theta(3)))'
%pause


dxdt=[dtdtheta;dtdposition;dtdv];




end

