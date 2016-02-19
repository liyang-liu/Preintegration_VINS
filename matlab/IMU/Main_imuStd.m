%% Main Entrance of IMU state prediction.

close all;
clear;
clc;

dataDir = './';%'../IMUsimulator_continuous/';
load([dataDir 'Initial_state_for_Youbing1.mat']); % x0 = [Euler angle, position, velocity]
load([dataDir 'IMUdata_for_Youbing1.mat']); % dataIMU: time step, rotation rate, acceleration

%% Initialization
% Bias
bf = [0, 0, 0]';
bo = [0, 0, 0]';
% Weight
gn = [0, 0, -9.8]';
% Timesteps
nts = size(dataIMU, 1);
% State vectors.
p = zeros(3, nts);
v = zeros(3, nts);
phi = zeros(3, nts);
phi(:, 1) = x0(1:3);
p(:, 1) = x0(4:6);
v(:, 1) = x0(7:9);
% Initial state uncertainty
P = eye(15);
% IMU noise covariance matrix
Q = eye(6);

%% Iterative prediction of inertial values and covariance
k = 1; 
while(k < nts)%20
	dt = (dataIMU(k+1, 1) - dataIMU(k, 1));
	omega0 = (dataIMU(k, 2:4))';
	fb0 = (dataIMU(k, 5:7))';
	[p(:, k+1), v(:, k+1), phi(:, k+1)] = fnStdPredict(p(:, k), v(:, k), phi(:, k), dt, fb0, omega0, bf, bo, gn);
	%P = fnStdCovPredict(Q, P, omega0, phi(:, k), dt, fb0, bf, bo)
	k = k + 1;
	disp([sprintf('T%d: [%f, %f, %f], [%f, %f, %f], [%f, %f, %f]', k, p(1, k), p(2, k), p(3, k), v(1, k), v(2, k), v(3, k), phi(1, k), phi(2, k), phi(3, k))]);	
end

%% Graphical display
figure(1); plot(dataIMU(:, 1)', p'); xlabel('Time (s)'); ylabel('Position (m)'); legend('x','y','z'); title('Translation Results: ');
figure(2); plot(dataIMU(:, 1)', v'); xlabel('Time (s)'); ylabel('Translational Velocity (m/s)'); legend('vx','vy','vz'); title('Translational Velocity Results: ');
figure(3); plot(dataIMU(:, 1)', phi'); xlabel('Time (s)'); ylabel('Rotational Velocity (rad/s)'); legend('Ox','Oy','Oz'); title('Rotation Results: ');
