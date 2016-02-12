function [] = fnCalnShowUncert(ef, K, x, nPoses, nPts, dt, Jd, CovMatrixInv, ...
    nIMUrate, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw)


J = fnJduvd_CnU_dbg(K, x, nPoses, nPts, nIMUrate, bPreInt);
if(bPreInt == 1)
    J = fnJddpvphi_IMU_dbg(J, dt, Jd, nPoses, nPts, x, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);%g0, 
else
    J = fnJdaw0_IMU(J, nPoses, nPts, x, nIMUrate, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);
end

Info = J'*CovMatrixInv*J;

% Tao = linsolve(Info, eye(size(Info,1)));%Info \
% Td = diag(Tao);
Td = fnGetSigm5Info(Info);
Td = 3*sqrt(Td);
%nTd = -Td;

fnShowEstmBnd(ef, Td, nPoses, nPts, nIMUrate, bPreInt);


% 