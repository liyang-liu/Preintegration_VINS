clc;
close all;
clear;

addpath(genpath('..'));
% g = [0; 0; -9.8];
nsamplerate = 1e3;%2e2;%  1e5;
% t = 0:(1.0/nsamplerate):1;
nPoses  = 4;
% nsamplerate = nsamplerate + 1;
% Angles
dtang = pi / (3 * (nPoses - 1));
% theta = [zeros(1, nsamplerate); -dtang*t; zeros(1, nsamplerate)];%+-pi/7*t
% dthetadt = [0; -dtang; 0];% Linear
% dthetadt = repmat(dthetadt, 1, nsamplerate);
% Position
Zcirc = 9; fdcirc2camera = 3;
spn = 1;
dstep = 2 * (Zcirc - fdcirc2camera) * sin(dtang / 2);
dx = dstep * cos(dtang / 2)
dy = dstep * sin(dtang / 2)
dz = spn * 0.1% 0;% temporarily in a plane % 
% dx = dstep * cos(dtang / 2);
% dz = dstep * sin(dtang / 2);
% dy = spn * 0.1;

% p = [dx * t; spn * dy * t; dz * (t .* t)];
% v = [dx; spn * dy; dz]
% dvdt = repmat([0;0;0], 1, nsamplerate);
% dvdt(3,:) = 2 * dz;
bConstV = 0;g = [0; 0; -9.8];
[x0, dataIMU] = fnSimuIMU(nsamplerate, dtang, dx, dy, dz, g, bConstV);% theta, dthetadt, dvdt, g);
%dataIMU = [t',dataIMU'];

% initialtheta = zeros(3,1);
% initialposition = zeros(3,1);
% initialvelocity = [dx; spn * dy; 0];
% x0=[initialtheta;initialposition;initialvelocity];

save('Initial_state_for_Youbing1.mat', 'x0')
save('IMUdata_for_Youbing1.mat', 'dataIMU');

%[T,X]=ode45(@dynamics_plant,[0: 0.01: 1],x0);


