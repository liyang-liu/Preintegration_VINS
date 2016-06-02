function [uvd_cell, R_cell, T_cell] = funSimFeaturesAtNPoses(nPoses, nPts)
%% Simulate two camera poses and a group of features.
%% Here I assume the features are situated on twocircles, each of which 
%   is put at a certain distance to the camera pose 1. 
%
% uvd: feature images in [u; v; d] form; each pose corresponds to one cell
% R, T: camera pose ground truth values; each pose corresponds to one cell

if(nPoses < 1)
	uvd_cell = {};
	R_cell = {};
	T_cell = {};
	return;
end

% Initialization
nf1c = nPts/2; %10;% 5;%200; % number of features on on circle
nc = 2; % number of circles
radius = 3; % the dimension of the circles
np = nf1c * nc;
fp1 = zeros(3, np);
uvd1 = fp1;
fp2 = fp1;
uvd2 = fp1;

% Parameters for the initial camera pose.
%cx = 0; cy = 0; cz = 0; 
cx0 = 240; cy0 = 320; f = 575;
% Feature distance to the initial camera pose.
fdcirc2camera = 3.0;
zcirc = fdcirc2camera; 

% Calculate feature positions at the first camera pose
fangle = (2*pi/nf1c):(2*pi/nf1c):(2*pi);
deltangle = pi/nf1c;
uvd_cell{1} = zeros(3, np);
R_cell{1} = eye(3);
T_cell{1} = zeros(3, 1);
for m=1:nc % on nc circles	
	pids = (1:nf1c) + ((m - 1) * nf1c);
	fp1(3, pids) = zcirc;
	uvd_cell{1}(3, pids) = zcirc;	
	fp1(1, pids) = fdcirc2camera * cos(fangle);%zcirc
	fp1(2, pids) = fdcirc2camera * sin(fangle);%zcirc	
	uvd_cell{1}(1, pids) = f * fp1(1, pids) ./ fp1(3, pids) + cx0;
	uvd_cell{1}(2, pids) = f * fp1(2, pids) ./ fp1(3, pids) + cy0;
	zcirc = zcirc + fdcirc2camera;
	fangle = fangle + deltangle; % rotate an angle
end

% Parameters for the following camera poses
dtang = pi / (3 * (nPoses - 1));
dstep = 2 * (zcirc - fdcirc2camera) * sin(dtang / 2);
dx = dstep * cos(dtang / 2);
dz = dstep * sin(dtang / 2);
dy = 0.1;%0;%
spn = 1;
gamma = 0;% pi/6;
alpha = 0;
for cid=2:nPoses
	uvd_cell{cid} = zeros(3, np);
	beta = -(cid - 1) * dtang;
% 	alpha = spn * pi/7; % alpha = pi/30; beta = pi/40; gamma = pi/50;
	R_cell{cid} = fRfrmABG(alpha, beta, gamma);
        dstep = 2 * (zcirc - fdcirc2camera) * sin(-beta / 2);
        dx = dstep * cos(-beta / 2);
        dz = dstep * sin(-beta / 2);    
	tT = [dx, spn * dy, dz]';
	T_cell{cid} = -(R_cell{cid}) * tT;
	% Features at the second position;
	fp2 = R_cell{cid} * fp1 + repmat(T_cell{cid}, 1, np);
	uvd_cell{cid}(3, :) = fp2(3, :);
	uvd_cell{cid}(1, :) = f * fp2(1, :) ./ fp2(3, :) + cx0;
	uvd_cell{cid}(2, :) = f * fp2(2, :) ./ fp2(3, :) + cy0;
	spn = -spn;
end
		
