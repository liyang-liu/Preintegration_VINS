function [dp, dv, dphi, J, R] = fnDeltObsAccu(bf, bw, dataIMU)

%% Main Entrance of IMU state prediction.

dp = zeros(3,1);
dv = zeros(3,1);
dphi = zeros(3,1);

%% Initialization
% Bias
% bf = [0, 0, 0]';
% bo = [0, 0, 0]';
% Timesteps
nts = size(dataIMU, 1);
% State vectors.
% Initial state uncertainty
J = eye(15);
% IMU noise covariance matrix
R = zeros(15, 15);
% IMU noise covariance matrix
Q = eye(6);
dt = (dataIMU(2, 1) - dataIMU(1, 1));
%% Iterative prediction of inertial values and covariance
k = 1; 
while(k <= nts)%20	
	omega0 = (dataIMU(k, 2:4))';
	fb0 = (dataIMU(k, 5:7))';
	[dp, dv, dphi] = fnIMUdltObs(dp, dv, dphi, dt, fb0, omega0, bf, bw);
    [J, R] = fnIMUdltJnCov(Q, J, R, omega0, dphi, dt, fb0, bf, bw);
	%P = fnStdCovPredict(Q, P, omega0, phi(:, k), dt, fb0, bf, bo)
	k = k + 1;
% 	disp([sprintf('T%d: [%f, %f, %f], [%f, %f, %f], [%f, %f, %f]', k, dp(1), dp(2), dp(3), dv(1), dv(2), dv(3), dphi(1), dphi(2), dphi(3))]);	
end


% %% Graphical display
% figure(1); plot(dataIMU(:, 1)', p'); xlabel('Time (s)'); ylabel('Position (m)'); legend('x','y','z'); title('Translation Results: ');
% figure(2); plot(dataIMU(:, 1)', v'); xlabel('Time (s)'); ylabel('Translational Velocity (m/s)'); legend('vx','vy','vz'); title('Translational Velocity Results: ');
% figure(3); plot(dataIMU(:, 1)', phi'); xlabel('Time (s)'); ylabel('Rotational Velocity (rad/s)'); legend('Ox','Oy','Oz'); title('Rotation Results: ');
