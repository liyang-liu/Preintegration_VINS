function [uvd1, uvd2, R, T] = funSimFeaturesAtTwoPoses()
%% Simulate two camera poses and a group of features.
%% Here I assume the features are situated on circles, each of which is put at a certain distance to the camera pose 1. 
%
% uvd1, uvd2: feature images in [u; v; d] form
% fp1, fp2: feature positions in [x; y; z] form
% R, T: camera pose ground truth values

% Initialization
nf1c = 10;% 5;%200; % number of features on on circle
nc = 2; % number of circles
radius = 3; % the dimension of the circles
np = nf1c * nc;
fp1 = zeros(3, np);
uvd1 = fp1;
fp2 = fp1;
uvd2 = fp1;

% Parameters for the initial camera pose
cx = 0; cy = 0; cz = 0; 
cx0 = 240; cy0 = 320; f = 575;
fdcirc2camera = 3.0;
zcirc = cz + fdcirc2camera; 

% Features at the first camera pose
fangle = (2*pi/nf1c):(2*pi/nf1c):(2*pi);
deltangle = pi/nf1c;
for m=1:nc % on nc circles
	pids = (1:nf1c) + ((m - 1) * nf1c);
	fp1(3, pids) = zcirc;
	uvd1(3, pids) = zcirc;	
	fp1(1, pids) = fdcirc2camera * cos(fangle);%zcirc
	fp1(2, pids) = fdcirc2camera * sin(fangle);%zcirc	
	uvd1(1, pids) = f * fp1(1, pids) ./ fp1(3, pids) + cx0;
	uvd1(2, pids) = f * fp1(2, pids) ./ fp1(3, pids) + cy0;
	zcirc = zcirc + fdcirc2camera;
	fangle = fangle + deltangle; % rotate an angle
end

% Parameters for the second camera pose
alpha = pi/5; beta = pi/6; gamma = pi/7;% alpha = pi/30; beta = pi/40; gamma = pi/50;
R = fRfrmABG(alpha, beta, gamma);
T = [3,2,1]';
% Features at the second position;
fp2 = R * fp1 + repmat(T, 1, np);
uvd2(3, :) = fp2(3, :);
uvd2(1, :) = f * fp2(1, :) ./ fp2(3, :) + cx0;
uvd2(2, :) = f * fp2(2, :) ./ fp2(3, :) + cy0;
		
