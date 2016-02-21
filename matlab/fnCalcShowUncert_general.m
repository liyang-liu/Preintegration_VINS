function [] = fnCalcShowUncert_general( RptFeatureObs, ImuTimestamps, ...
                dtIMU, ef, K, X_obj, nPoses, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata )
    
global InertialDelta_options


    J_obj = InertialDelta_InitJacobian( );
            J_obj = fnJacobian_dUv_dX(J_obj, K, X_obj, Zobs, nPoses, nPts, nIMUdata, ImuTimestamps, RptFeatureObs );		
    J_obj = fnJacobian_dIntlDelta_dX( J_obj, dtIMU, Jd, nPoses, nPts, X_obj, Zobs );
    
    J = JObject2Matrix( J_obj, Zobs, X_obj );
    
%%%%%%%%%%%
Info = J'*CovMatrixInv*J;

% Tao = linsolve(Info, eye(size(Info,1)));%Info \
% Td = diag(Tao);
Td = fnGetSigmFromInfo(Info);
Td = 3*sqrt(Td);
%nTd = -Td;

fnShowEstmBnd_general(ef, Td, nPoses, nPts, nIMUdata, bPreInt, bUVonly, bVarBias);%fnShowEstmBnd(ef, Td, nPoses, nPts, nIMUrate, bPreInt);


% 