function [] = fnCalcShowUncert_general(nUV, RptFeatureObs, ImuTimestamps, ...
    dtIMU, ef, K, x, nPoses, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata )
    
global InertialDelta_options

% J = fnJduvd_CnU_dbg(K, x, nPoses, nPts, nIMUrate, bPreInt);
% if(bPreInt == 1)
%     J = fnJddpvphi_IMU_dbg(J, dt, Jd, nPoses, nPts, x, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);%g0, 
% else
%     J = fnJdaw0_IMU(J, nPoses, nPts, x, nIMUrate, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);
% end

    [idRow, idCol, nJacs] = fnFndJacobianID(nIMUdata, nPoses, RptFeatureObs, ImuTimestamps);
    if(bUVonly == 1)% UVonly, add Au2c,Tu2c and Z2
        idx_au2c = (nPoses-1)*6+nPts*3;
        uidRow = [1,2,3,1,2,3,1,2,3, ... %Au2c
                  4,5,6,4,5,6,4,5,6, ... %Tu2c
                  7 ... % Z2
                  ];                  
        uidCol = [idx_au2c+1,idx_au2c+1,idx_au2c+1, ... 
                  idx_au2c+2,idx_au2c+2,idx_au2c+2, ... 
                  idx_au2c+3,idx_au2c+3,idx_au2c+3, ... %Au2c
                  idx_au2c+4,idx_au2c+4,idx_au2c+4, ... 
                  idx_au2c+5,idx_au2c+5,idx_au2c+5, ... 
                  idx_au2c+6,idx_au2c+6,idx_au2c+6, ... %Tu2c
                  6 ... %4 for Malaga
                 ]; 
        unJacs = 3*3*2+1; 
    else
        [uidRow, uidCol, unJacs] = fnFndJacIDimu(ImuTimestamps, nIMUdata, bUVonly, nPoses, nPts, bAddZg, ...
            bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias, bPreInt);        
    end
    
    [J] = fnJacobian_dUv_dX(nJacs, idRow, idCol, K, x, nPoses, nPts, nIMUdata, ImuTimestamps, ...
    RptFeatureObs, bUVonly, bPreInt, nUV, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, ...
    bAddZbw, bVarBias);

    if((bUVonly == 1) || (bPreInt == 1))
        J((nUV+1):end,:) = fnJacobian_dIntlDelta_dX(uidRow, uidCol, unJacs, nUV, bUVonly, dtIMU, Jd, nPoses, nPts, x, ...
                bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
    else
        J((nUV+1):end,:) = fnJdaw0_IMU_gq(uidRow, uidCol, unJacs, nUV, nPts, x, nIMUrate, nIMUdata, ...
            bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias, nPoses);    
    end
%%%%%%%%%%%
Info = J'*CovMatrixInv*J;

% Tao = linsolve(Info, eye(size(Info,1)));%Info \
% Td = diag(Tao);
Td = fnGetSigmFromInfo(Info);
Td = 3*sqrt(Td);
%nTd = -Td;

fnShowEstmBnd_general(ef, Td, nPoses, nPts, nIMUdata, bPreInt, bUVonly, bVarBias);%fnShowEstmBnd(ef, Td, nPoses, nPts, nIMUrate, bPreInt);


% 