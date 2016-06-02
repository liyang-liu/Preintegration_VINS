function [e] = fnCnUPredErr_lsqnonlin_general(x)

addpath(genpath('IMU'));
load('consts.mat');
load('bAddZg.mat');% Use the latest value
load('bAddZau2c.mat');
load('bAddZtu2c.mat');
load('bAddZbf.mat');
load('bAddZbw.mat');
load('RptFeatureObs.mat');%obsfeatures
load('bUVonly.mat');
load('bVarBias.mat');
if(bUVonly == 0)
    load('ImuTimestamps.mat');
    load('dtIMU.mat');
else
    ImuTimestamps = [];
    dtIMU = [];
end

e = zeros(size(Zobs));
%idr = nPoses*nPts*2;%2

% 1. UVD error:
[e, nUV] = fnUVDErr_C1U_genral(RptFeatureObs, K, x, Zobs, nPoseNew, nPts, ...
    ImuTimestamps, bUVonly, bPreInt);
% 2. IMU dlt error:
[e((nUV+1):end,1)] = fnIMUdltErr_general(x, Zobs((nUV+1):end,1), nPoseNew, nPts, bf0, ...
    bw0, dtIMU, Jd, nIMUrate, ImuTimestamps, bUVonly, bPreInt, bAddZg, bAddZau2c, ...
    bAddZtu2c, bAddZbf, bAddZbw, bVarBias);



