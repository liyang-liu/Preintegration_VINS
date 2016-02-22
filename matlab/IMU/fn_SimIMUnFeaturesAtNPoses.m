function [imuData_cell, uvd_cell, Ru_cell, Tu_cell, Ru2c, Tu2c] = ...
            fn_SimIMUnFeaturesAtNPoses(nPoses, nPts, Ru2c, Tu2c, g)
%% Simulate a group of features, n camera poses and IMU readings during each step
%% Here I assume the features are situated on twocircles, each of which 
%   is put at a certain distance to the camera pose 1. 
%
% uvd: feature images in [u; v; d] form; each pose corresponds to one cell
% R, T: camera pose ground truth values; each pose corresponds to one cell

% Assume that the IMU and Camera are fixed on a rigid body, having no relative motion.
% Ru2c = fRx(-pi/2);
% Tu2c = [1.50; 0; 0]; %
% g = [0; 0; -9.8];
if(nPoses < 1)
    imuData_cell = [];
	uvd_cell = {};
	Ru_cell = {};
	Tu_cell = {};
	return;
end

% Initialization of feature positions.
nf1c = nPts/2; %10;% 5;%200; % number of features on one circle
nc = 2; % number of circles
radius = 3; % the dimension of the circles
np = nf1c * nc;
fp0 = zeros(3, np);
% uvd1 = fp1;
% fp2 = fp1;
% uvd2 = fp1;

% Parameters for the initial camera pose.
%cx = 0; cy = 0; cz = 0; 
cx0 = 240; cy0 = 320; f = 575;
% Feature distance to the initial camera pose.
fdcirc2camera = 3.0;
zcirc = fdcirc2camera; 

% Ru_cell{1} = eye(3);
% Tu_cell{1} = zeros(3, 1);

% Calculate feature positions at the first camera pose
fangle = -[(2*pi/nf1c):(2*pi/nf1c):(2*pi)];
deltangle = pi/nf1c;
% uvd_cell{1} = zeros(3, np);

% Initialize features in the first IMU pose and then transfer.
for m=1:nc % on nc circles	
	pids = (1:nf1c) + ((m - 1) * nf1c);
	fp0(2, pids) = zcirc; % IMUy
	%uvd_cell{1}(3, pids) = zcirc;	
	fp0(1, pids) = Tu2c(1) + radius * cos(fangle);% IMUx
	fp0(3, pids) = Tu2c(3) + radius * sin(fangle);% IMUz
% 	uvd_cell{1}(1, pids) = f * fp1(1, pids) ./ fp1(3, pids) + cx0;
% 	uvd_cell{1}(2, pids) = f * fp1(2, pids) ./ fp1(3, pids) + cy0;
	zcirc = zcirc + fdcirc2camera;
	fangle = fangle + deltangle; % rotate an angle
end

% Parameters for the following camera poses
dtang = pi / (3 * (nPoses - 1));
% dstep = 2 * (zcirc - fdcirc2camera) * sin(dtang / 2);
% dx = dstep * cos(dtang / 2);
% dy = dstep * sin(dtang / 2);
dz = 0.0;%-0.1;%
spn = 1;
beta = 0;% pi/6;
alpha = 0;% spn * pi/7; % alpha = pi/30; beta = pi/40; gamma = pi/50;
for cid=1:nPoses
	uvd_cell{cid} = zeros(3, np);
	gamma = (cid - 1) * dtang;
	Ru_cell{cid} = fRfrmABG(alpha, beta, gamma);
    if(cid == 1)
        tT = zeros(3,1);
    else
        fd = abs(complex(zcirc - fdcirc2camera, Tu2c(1)));
        dstep = 2 * fd * sin(gamma / 2);
        theta = acos(Tu2c(1)/fd)-(pi/2-gamma/2);
        dx = dstep * cos(theta);
        dy = dstep * sin(theta);          
        tT = [dx, dy, spn * dz]';
        % IMU data generation
        nsamplerate = 1e3;%2e2;%
        bConstV = 0;
        % Relative displacement of Pose 2 in Pose 1.
        T12 = Ru_cell{cid-1} * (tT - Tu_cell{cid-1});        
        [x0, dataIMU] = fnSimuIMU(nsamplerate, -dtang, T12(1), T12(2), T12(3), g, bConstV);
%         [dp, dv, dphi, J, R] = fnDeltObsAccu(dataIMU);
        
%         save('Initial_state_for_Youbing1.mat', 'x0')
%         save('IMUdata_for_Youbing1.mat', 'dataIMU');
        imuData_cell{cid} = dataIMU;
    end
    Tu_cell{cid} = tT;% Already in the initial coordinates 
    Rc = Ru2c * Ru_cell{cid};% Left multiply R to comply with the vectors to be multiplied on the right.
    Tc = Tu_cell{cid} + (Ru_cell{cid})' * Tu2c;
	% Features at the new position;
	fp2 = Rc * (fp0 - repmat(Tc, 1, np));
	uvd_cell{cid}(3, :) = fp2(3, :);
	uvd_cell{cid}(1, :) = f * fp2(1, :) ./ fp2(3, :) + cx0;
	uvd_cell{cid}(2, :) = f * fp2(2, :) ./ fp2(3, :) + cy0;
    
	spn = -spn;
    
    
end
		
