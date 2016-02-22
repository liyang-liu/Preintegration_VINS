function [drda, drdb, drdg] = fn_ABG2R_dr(alpha, beta, gamma)
%% Passive rotation, Tait-Bryan Angles
%% R = Rx(alpha) * Ry(beta) * Rz(gamma): 
% r11 = cos(beta) * cos(gamma), [c2c3]
% r12 = cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta), [c1s3 +
% c3s1s2]
% r13 = sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta), [s1s3-c1c3s2]
% r21 = -cos(beta)*sin(gamma), [-c2s3]
% r22 = cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma), [c1c3-s1s2s3]
% r23 = cos(gamma)*sin(alpha)+cos(alpha)sin(beta)*sin(gamma), [c3s1+c1s2s3]
% r31 = sin(beta), [s2]
% r32 = -cos(beta)*sin(beta), [-c2s1]
% r33 = cos(alpha)*cos(beta), [c1c2]

%dr11 = [0, -sin(beta)*cos(gamma), -cos(beta)*sin(gamma)];
%dr12 = [-sin(alpha)*sin(gamma)+cos(gamma)*cos(alpha)*sin(beta), cos(gamma)*
%    sin(alpha)*cos(beta), cos(alpha)*cos(gamma)-sin(gamma)*sin(alpha)*sin(beta)];
%dr13 = [cos(alpha)*sin(gamma)+sin(alpha)*cos(gamma)*sin(beta),    -cos(alpha)*cos(gamma)*cos(beta), 
%    sin(alpha)*cos(gamma)+cos(alpha)*sin(gamma)*sin(beta)];
%dr21 = [0, sin(beta)*sin(gamma), -cos(beta)*cos(gamma)];
%dr22 = [-sin(alpha)*cos(gamma)-cos(alpha)*sin(beta)];

drda = fn_Rx_dr(alpha) * fn_Ry(beta) * fn_Rz(gamma);
drdb = fn_Rx(alpha) * fn_Ry_dr(beta) * fn_Rz(gamma);
drdg = fn_Rx(alpha) * fn_Ry(beta) * fn_Rz_dr(gamma);

