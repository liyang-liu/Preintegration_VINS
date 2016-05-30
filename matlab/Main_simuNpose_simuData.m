%% The main function.
clear;
close all;
clc;

global PreIntegration_options
PreIntegration_options.bInc = 0;
PreIntegration_options.bSimuNpose = 1;
run PreIntegration_config_script

global Data_config
run Data_config_script

assert( PreIntegration_options.bSimData == 1, 'Malag not tested');
%assert( PreIntegration_options.bPreInt == 1, 'Non-preintegration not tested');
assert( PreIntegration_options.bUVonly == 0, 'UV only not tested');
assert( PreIntegration_options.bVarBias == 0, 'Variable Bias not tested');

%% Configure the conditions of the problem
nPoses = PreIntegration_options.nPoses
kfids = 1:PreIntegration_options.kfspan:1200;
nMinObsTimes = 2;%3;%5;%4;%6;%


%% Configure paramters according to the chosen dataset
if(PreIntegration_options.bSimData == 1)
    % Add noise to the observation vector z
    %bZnoise = 1;%fZnoisescale = 1;%6;%1e2;%12; % bXnoise = 1;
    fXnoisescale                = 0;%1e-6;%1e-2;%1e0;%5e-2;%3e-2        
    nIMUrate                    = 5e2;% IMU data rate
    SLAM_Params.sigma_w_real    = 0;%0.03; 
    SLAM_Params.sigma_f_real    = 0;%0.2;%    
    SLAM_Params.sigma_w_cov     = 1;%2*sigma_w_real;%1e-3; 
    SLAM_Params.sigma_f_cov     = 1;%2*sigma_f_real;%1e-3;%0.0775; % Siga for the noises to be added to the IMU raw data (wi,ai)
    SLAM_Params.sigma_uov_real  = 0;%1e-2;%1;% 
    SLAM_Params.sigma_uov_cov   = 1;%2*sigma_uov_real;%
    SLAM_Params.sigma_d_real    = 0;%
    SLAM_Params.sigma_d_cov     = 0.1;
    SLAM_Params.sigma_bf_real   = 0;% 1e-3;
    SLAM_Params.sigma_bw_real   = 0;% 1e-3;
    SLAM_Params.sigma_g_cov     = 1e-4;
    SLAM_Params.sigma_au2c_cov  = 1e-4;
    SLAM_Params.sigma_tu2c_cov  = 1e-4;
    SLAM_Params.sigma_bf_cov    = 1e-4;
    SLAM_Params.sigma_bw_cov    = 1e-4;
    SLAM_Params.sigma_tv        = 1e-4/(nIMUrate);
    
    nPts = 100;% Number of points to be simulated. %1e3;%10;%50;%4;%1;%2;%20;% 
    
    % Configure pseudo observations    
    
    SLAM_Params.g0              = [ 0; 0; -9.8 ]; % g value in the first key frame
    SLAM_Params.g_true          = [ 0; 0; -9.8 ];
    SLAM_Params.bf0             = [ 1.0, 2.0, 3.0 ]'; % bias for acceleration
    SLAM_Params.bw0             = [ 1.0, 2.0, 3.0 ]'; %[0, 0, 0]'; % bias for rotaion velocity    
    SLAM_Params.bf_true         = [ 1.0; 2.0; 3.0 ]; % bias for acceleration
    SLAM_Params.bw_true         = [ 1.0; 2.0; 3.0 ]; %[0, 0, 0]'; % bias for rotaion velocity 
    
    ImuTimestamps               = 1 : (nIMUrate) : ( ( nPoses - 1 ) * nIMUrate + 1 );
    nUV                         = nPts * 2 * nPoses;
    nIMUdata                    = ( nPoses - 1 ) * nIMUrate;
    dtIMU                       = ones( nPoses, 1 );
    dt                          = 1.0 / nIMUrate;
    
    % Iteration times and bounds for Gauss-Newton
    SLAM_Params.nMaxIter                = 1e3;%50;%30;%100;%15;%5;%10;%50;%3;% 20;% 
    SLAM_Params.fLowerbound_e           = 1e-10;%1e-6;%1e-5;%1e-1;
    SLAM_Params.fLowerbound_dx          = 1e-10;%1e-6;%
    SLAM_Params.fLowerbound_chi2Cmpr    = 1e-4;
end


addpath(genpath('IMU'));
addpath(genpath('MoSeg_2D'));%addpath(genpath('ms3D'));
addpath(genpath('Ransac'));

uvd_cell    = [];
dp          = zeros(3, nPoses);
dv          = zeros(3, nPoses);
dphi        = zeros(3, nPoses);
Jd          = [];
Rd          = [];


%% The main switch
if(PreIntegration_options.bSimData)

    fbiascef = 3;
    
    idr = nPoses*nPts*2;% total number of camera observations: (ui,vi) 
           
    %% Other parameters for the simulated scenario
	cx0 = 240; cy0 = 320; f = 575; % camera intrinsic parameters
	K = [f 0 cx0; 0 f cy0; 0 0 1]; 
    SLAM_Params.Ru2c = fn_Rx( -pi/2 ); % Relative rotation
    SLAM_Params.Tu2c = [0; 0; 0]; % Relative translation
    %     SLAM_Params.bf0 = [0, 0, 0]'; % bias for acceleration
    %     SLAM_Params.bw0 = [0, 0, 0]'; % bias for rotaion velocity
    %     SLAM_Params.g0 = [0; 0; -9.8]; % g value in the first key frame
            
    [imuData_cell, uvd_cell, noisefree_imuData_cell, noisefree_uvd_cell, Ru_cell, Tu_cell, FeatureObs, ...
                    dp, dv, dphi, Jd, Rd, vu, SLAM_Params] = ...
                            fn_ObtainSimuData( SLAM_Params, nPoses, nPts, nIMUrate );
                        
    RptFeatureObs = FeatureObs;
    save([ Data_config.TEMP_DIR 'RptFeatureObs.mat'], 'RptFeatureObs');

    X_obj = SLAM_X_Define( nPts, nPoses, nIMUrate );
    Xg_obj = X_obj;        
    [X_obj, Xg_obj, Feature3D ] = fn_Generate_Xinit_and_Xgt( X_obj, Xg_obj, RptFeatureObs, imuData_cell, uvd_cell, noisefree_imuData_cell, noisefree_uvd_cell, Ru_cell, Tu_cell, nIMUdata, nIMUrate, ImuTimestamps, dtIMU, dp, dv, dphi, K, cx0, cy0, f, dt, vu, SLAM_Params );
    
    %--------------------------------------------------
    % Hack: force X_obj to have same pose as GT
    %--------------------------------------------------
    X_obj = Xg_obj;
    %--------------------------------------------------
    

    X_init = X_obj;    
    save([ Data_config.TEMP_DIR 'X_init.mat'],'X_init');
    
    if(PreIntegration_options.bShowFnP == 1)
        fn_ShowFeaturesnPoses( Xg_obj, nPoses, nPts, nIMUdata, 'Ground Truth Values' );
        fn_ShowFeaturesnPoses( X_obj, nPoses, nPts, nIMUdata, 'Initial Values' );
    end    
    
    Zobs = fn_Collect_Zobs( RptFeatureObs, imuData_cell, nPoses, nPts, nIMUrate, dp, dv, dphi, SLAM_Params )
    
    %% Save data for nonlin method.
    dt = ((imuData_cell{2}.samples(2, 1) - imuData_cell{2}.samples(1, 1))) * size(imuData_cell{2}.samples,1);
    nPoseNew = nPoses;
    save([ Data_config.TEMP_DIR 'consts.mat'],'nIMUrate','K','Zobs','nPoses','nPts', 'SLAM_Params', 'dt','Jd');
    save( [ Data_config.TEMP_DIR 'Zobs.mat' ], 'Zobs'); 
    save( [ Data_config.TEMP_DIR 'RptFeatureObs.mat' ], 'RptFeatureObs'); 

    %% Covariance matrix
    CovMatrixInv = ...%SLAM_CalcCovMatrixInv( SLAM_Params, Zobs, Rd );
            fn_Generate_CovMatrixInv2( nPoses, nPts, length(Zobs.fObs)*2, nIMUrate, Rd, SLAM_Params );
            %fn_Generate_CovMatrixInv( SLAM_Params, Zobs, Rd );
    save([ Data_config.TEMP_DIR 'CovMatrixInv.mat'],'CovMatrixInv', '-v7.3');   
    save([ Data_config.TEMP_DIR 'ImuTimestamps.mat'], 'ImuTimestamps');
    save([ Data_config.TEMP_DIR 'dtIMU.mat'], 'dtIMU');    
    
tic
    if(PreIntegration_options.bGNopt == 1)
        %% GN Iterations 
        [X_obj, nReason] = fn_GaussNewton_GraphSLAM(K, X_obj, nPoses, nPts, Jd, CovMatrixInv, ...
                        nIMUrate, nIMUdata,  ImuTimestamps, dtIMU, RptFeatureObs, SLAM_Params );
        
    else    
        [x,Reason,Info] = fn_LeastSqrLM_GraphSLAM(nUV, K, x, nPoses, nPts, Jd, ...
                        CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
                        bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);
    end
toc    

    X_final = X_obj;
    if(PreIntegration_options.bPreInt == 0)
        fprintf('\n###Poses[x(1-%d)], Features[x(%d-%d)], Velocity[x(%d-%d]###\n',...
            (nPoses-1)*nIMUrate*6, (nPoses-1)*nIMUrate*6+1, ...
            (nPoses-1)*nIMUrate*6+nPts*3, (nPoses-1)*nIMUrate*6+nPts*3+1, ...
            (nPoses-1)*nIMUrate*6+nPts*3+3*((nPoses-1)*nIMUrate+1));
    else
        fprintf('\n###Poses[x(1-%d)], Features[x(%d-%d)], Velocity[x(%d-%d)]###\n',...
            (nPoses-1)*6, (nPoses-1)*6+1, ...
            (nPoses-1)*6+nPts*3, (nPoses-1)*6+nPts*3+1, ...
            (nPoses-1)*6+nPts*3+3*nPoses);
    end     
    ef = SLAM_X_Object2Vector( SLAM_X_ObjectDiff( X_final, Xg_obj ));
    [maxe, idx] = max(abs(ef));
    fprintf('Final Error: maxXef=%f, idx=%d\n', maxe, idx);
    %     fprintf('Final Value:\n\t Xf=[');
    %     fprintf('%f ', xf);
    %     fprintf(']\n');    
    
    %%%%%%%%%%%%    
    
    save( [Data_config.TEMP_DIR 'x_Jac.mat'], 'X_obj');
    %% Show pose-feature graph
    if(PreIntegration_options.bShowFnP == 1)
        fn_ShowFeaturesnPoses_general(X_final, nIMUrate, 'Final Values');
    end
    fn_ShowFeaturesnPoses_superimpose(Xg_obj, X_init, X_final);
    
    %% Show uncertainty
    if(PreIntegration_options.bShowUncertainty == 1)
        fn_CalcShowUncert_general( RptFeatureObs, ImuTimestamps, ...
                        dtIMU, ef, K, X_obj, nPoses, nPts, Jd, ...
                        CovMatrixInv, nIMUrate, nIMUdata );    
    end

    return;

end
