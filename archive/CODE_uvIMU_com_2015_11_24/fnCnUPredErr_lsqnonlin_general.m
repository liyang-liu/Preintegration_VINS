function [e] = fnCnUPredErr_lsqnonlin_general(x)

addpath(genpath('IMU'));
load('../../temp/consts.mat');
load('../../temp/bAddZg.mat');% Use the latest value
load('../../temp/bAddZau2c.mat');
load('../../temp/bAddZtu2c.mat');
load('../../temp/bAddZbf.mat');
load('../../temp/bAddZbw.mat');
load('../../temp/RptFeatureObs.mat');%obsfeatures
load('../../temp/bUVonly.mat');
load('../../temp/bVarBias.mat');
if(bUVonly == 0)
    load('../../temp/ImuTimestamps.mat');
    load('../../temp/dtIMU.mat');
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



