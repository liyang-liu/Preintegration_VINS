function [e] = fnCnUPredErr(K,x,Zobs,nPoses,N,bf0,bw0,dt,J)%g,

e = zeros(size(Zobs));
% 1. UVD error:
[e(1:(nPoses*N*3),1)] = fnUVDErr_C1U(K, x, Zobs(1:(nPoses*N*3),1), nPoses, N);
% 2. IMU dlt error:
% [e((nPoses*N*3+1):end,1)] = 0;
[e((nPoses*N*3+1):end,1)] = fnIMUdltErr(x, Zobs((nPoses*N*3+1):end,1), ...
    nPoses, N, bf0, bw0, dt, J);%g, 

