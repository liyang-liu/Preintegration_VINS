function [e] = fnCnUPredErr_lsqnonlin(x)

global InertialDelta_options

addpath(genpath('IMU'));
load('consts.mat');
%load('bAddZg.mat');% Use the latest value
%load('bAddZau2c.mat');
%load('bAddZtu2c.mat');
%load('bAddZbf.mat');
%load('bAddZbw.mat');

e = zeros(size(Zobs));
idr = nPoses*nPts*2;%2

% 1. UVD error:
[e(1:idr,1)] = fnUVDErr_C1U(K, x, Zobs(1:idr,1), nPoses, nPts, nIMUrate);
% 2. IMU dlt error:
[e((idr+1):end,1)] = fnIMUdltErr(x, Zobs((idr+1):end,1), nPoses, nPts, bf0, ...
                                    bw0, dt, Jd, nIMUrate);



