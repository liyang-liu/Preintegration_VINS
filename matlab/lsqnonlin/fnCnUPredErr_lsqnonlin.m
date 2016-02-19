function [e] = fnCnUPredErr_lsqnonlin(x)

addpath(genpath('..'));
load('consts.mat');
e = zeros(size(Zobs));
% 1. UVD error:
[e(1:(nPoses*nPts*3),1)] = fnUVDErr_C1U(K, x, Zobs(1:(nPoses*nPts*3),1), nPoses, nPts);
% 2. IMU dlt error:
% [e((nPoses*N*3+1):end,1)] = 0;
[e((nPoses*nPts*3+1):end,1)] = fnIMUdltErr(x, Zobs((nPoses*nPts*3+1):end,1), ...
    nPoses, nPts, bf0, bw0, dt, Jd);

