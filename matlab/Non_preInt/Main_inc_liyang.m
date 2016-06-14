%% The main function.
if 0
    close all;    
    clear all;
    clear class;
    clc;
end

global PreIntegration_options
PreIntegration_options.bInc = 1;
PreIntegration_options.bSimuNpose = 0;
run PreIntegration_config_script

global Data_config
run Data_config_script
global Data_config

nPoseOld = PreIntegration_options.nPoseOld
nAddPoses = PreIntegration_options.nAddPoses
nPoseNew = PreIntegration_options.nPoseNew
kfids = 1:PreIntegration_options.kfspan:1200;
nPts = PreIntegration_options.nPts
nAllposes = PreIntegration_options.nAllposes

uvd_cell = [];
dp = zeros(3,nPoseNew);
dv = dp; 
dphi = dp;
x_old = [];
PBAFeature = [];

if(PreIntegration_options.bSimData == 0)
    nPts1 = 60000;
else
    nPts1 = nPts;
end

nIMUdata_old = 0;
RptFidSet_old = [];
gtIMUposes = [];
selpids = [];

% if(bPreInt == 1)
%    bShowFnP = 1;
% else
%    bShowFnP = 0; 
% end

if(PreIntegration_options.bDinuka == 1)
    dtIMU = zeros(nPoseNew, 1);
    nIMUrate = 2e2;    
    
    SLAM_Params.bf0 = zeros(3,1);%[-0.55;0.6;0.61];
    SLAM_Params.bw0 = zeros(3,1);
    SLAM_Params.sigma_w_real = 0.03;%0.03;%0.01;%0.15;% 5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    SLAM_Params.sigma_f_real = 0.1;%0.2;%0.01;%0.1;%1;%  
    SLAM_Params.sigma_w_cov = 2 * SLAM_Params.sigma_w_real;%1;%1;%0.03;%0.1;% 0.01;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    SLAM_Params.sigma_f_cov = 2 * SLAM_Params.sigma_f_real;%1;%1;%0.2;%0.8;% 0.1;%1e-2;%1e-5;%1;%1e0;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%1e0;%2e-3;%
    SLAM_Params.sigma_uov_real = 0.1;%0.5;%0.2;% 1e-2;%1e-3;%2e-3;%1e-1;%1;%1e-6;%1e-4;%
    SLAM_Params.sigma_uov_cov = 3 * SLAM_Params.sigma_uov_real;%1;%1;%1e-1;%
   
    SLAM_Params.sigma_g_cov = 1e-4;
    SLAM_Params.sigma_au2c_cov = 1e-4;
    SLAM_Params.sigma_tu2c_cov = 1e-4;    
	SLAM_Params.sigma_bf_cov=1*2e-2;
    SLAM_Params.sigma_bw_cov = 1*20*pi/(180*3600); 
    SLAM_Params.sigma_tv = 1e-4/(nIMUrate);%1;%*nIMUrate*nIMUrate);
    % Configure pseudo observations
    
    SLAM_Params.g0 = [0; 0; 9.81]; % g value in the first key frame
    
    SLAM_Params.g_true = [0; 0; 9.81];
    SLAM_Params.bf_true = [0; 0; 0]; % bias for acceleration
    SLAM_Params.bw_true = [0; 0; 0]; %[0, 0, 0]'; % bias for rotaion velocity     
    % Directories
end


addpath(genpath('IMU'));
addpath(genpath('MoSeg_2D'));%addpath(genpath('ms3D'));
addpath(genpath('Ransac'));

% Iteration times and bounds for Gauss-Newton
nMaxIter = 30;%1e3;%50;%100;%15;%5;%10;%50;%3;% 20;% 
fLowerbound_e = 1e-10;%1e-6;%1e-5;%1e-1;
fLowerbound_dx = 1e-10;%1e-6;%

%dtIMU = [];
Jd =[];
Rd = [];


%% The main switch
    if(PreIntegration_options.bMalaga == 1)
        K = [923.5295, 0, 507.2222; 0, 922.2418, 383.5822; 0, 0, 1];% Left
        % [911.3657, 0, 519.3951; 0, 909.3910, 409.0285; 0, 0, 1]; &Right
    elseif(PreIntegration_options.bDinuka == 1)
       load([Data_config.DATA_DIR 'cam.mat']); 
       K = cam.K;
    end   
    
   
    if(PreIntegration_options.bMalaga == 1)
        SLAM_Params.Au2c_true = [-87.23; -2.99; -88.43]*pi/180;%[0;0;0];%[-86.19;-3.53;-90.31]*pi/180;%
        SLAM_Params.Ru2c_true = fnR5ABG(Au2c(1), Au2c(2), Au2c(3));
        SLAM_Params.Tu2c_true = [2.2-0.25;-0.427-0.029;0.025+(23-13.9)*1e-3];%[0;0;0];%
        %nIMUrate = 100; 
        dt = 1e-2;
    
    elseif(PreIntegration_options.bDinuka == 1)
        load([Data_config.DATA_DIR 'gtIMUposes.mat']);% ts, Aimu, Timu
        nt = size(gtIMUposes, 1); 
        selpids = 9:(10*PreIntegration_options.kfspan):nt;
        SLAM_Params.Ru2c_true = ([0,1,0; 0,0,1; 1,0,0]); 
        SLAM_Params.Au2c_true = zeros(3,1);
        [SLAM_Params.Au2c_true(1), SLAM_Params.Au2c_ture(2), SLAM_Params.Au2c_true(3) ] = ...
                                                fnABGFromR(SLAM_Params.Ru2c_true);
        SLAM_Params.Tu2c_true = zeros(3,1);
        %nIMUrate = 200; 
        dt = 1.0/nIMUrate;
    end    
%     g0 = [0; 0; -9.8]; 
    SLAM_Params.Au2c = SLAM_Params.Au2c_true;
    SLAM_Params.Ru2c = SLAM_Params.Ru2c_true;
    SLAM_Params.Tu2c = SLAM_Params.Tu2c_true;
    
    [ FeatureObs, Feature3D, imufulldata, ImuTimestamps, dtIMU, dp, dv, dphi, Jd, Rd ] = ...
                                            LoadData( nPts, nAllposes, kfids, SLAM_Params );
    
%% Incrementally construct x, z and cov, then solve them trhough iterations 
    while(nPoseOld < nAllposes)
        fprintf('\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
        fprintf('n%%            PoseOld - %d, nPoseNew - %d                  %%\n', nPoseOld, nPoseNew);
        fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
        
        nPoses = nPoseNew - nPoseOld;
        if(nPoseOld == 1)
            pid = 1;
            [FeatureObs] = fnCollectfObsFromImgs( ...
                                kfids, pid, Data_config.imgdir, SLAM_Params.sigma_uov_real, FeatureObs );                              
        end 
        
        for(pid=(nPoseOld+1):nPoseNew)
               [FeatureObs] = fnCollectfObsFromImgs( ...
                                kfids, pid, Data_config.imgdir, SLAM_Params.sigma_uov_real, FeatureObs ); 
        end
        
        if(PreIntegration_options.bMalaga == 1)
            load([ Data_config.DATA_DIR 'PBAFeature.mat']);
            %RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoseNew, nMinObsTimes));
            %RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
            %RptFeatureObs = FeatureObs(RptFidSet, :);
            RptFidSet = find( [FeatureObs(:).nObs] > 1);
            RptFidSet = RptFidSet(:);
            RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
            RptFeatureObs = FeatureObs(RptFidSet);
        elseif(PreIntegration_options.bDinuka == 1)
            RptFidSet = find( [FeatureObs(:).nObs] >= min(nPoseNew, PreIntegration_options.nMinObsTimes));
            RptFidSet = RptFidSet(:);
            RptFeatureObs = FeatureObs(RptFidSet);
        end
        
        nPts = size(RptFidSet, 1);
                   
        nIMUdata = ImuTimestamps(nPoseNew) - ImuTimestamps(1);
        
        %% X---the state vector
        X_obj = InertialDelta_CreateX( nPts, nPoseNew, nIMUdata );
        Xg_obj = X_obj;

        %% Compose the ground truth value 1
        [~, fscaleGT] = fnGetXgroundtruth_general(Xg_obj, ...
            Data_config.DATA_DIR, nPoseNew, ImuTimestamps, gtIMUposes, selpids, ...
            nPts, PBAFeature, RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, dp, dv, ...
            Data_config.gtVelfulldir, SLAM_Params );     
    
        %% Compose Initial value of X from odometry 
        if(PreIntegration_options.bInitPnF5VoU == 1)
            [X_obj, RptFidSet, RptFeatureObs, nPts] = fnCompXFromOdometry( ...
                        nPoseOld, nPoseNew, nPoses, nPts, x_old, ...
                        ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
                        RptFidSet_old, dtIMU, dp, dv, dphi, K, RptFeatureObs, ...
                        fscaleGT, kfids, nIMUrate, X_obj, SLAM_Params, imufulldata);
            
        end
        
        %% Compose the ground truth value 2
        [Xg_obj, fscaleGT] = fnGetXgroundtruth_general(Xg_obj, ...
            Data_config.DATA_DIR, nPoseNew, ImuTimestamps, gtIMUposes, selpids, ...
            nPts, PBAFeature, RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, dp, dv, ...
            Data_config.gtVelfulldir, SLAM_Params);
        
            
        % Display Xgt
        fprintf('Ground Truth Value:\n\t Xg_obj=[\nAng: ');
        %fprintf('%f ', Xg_obj(1:20));
        fprintf('%f ', [Xg_obj.pose(1).ang.val; Xg_obj.pose(2).ang.val; Xg_obj.pose(3).ang.val]);
        fprintf('\nTrans: ');
        fprintf('%f ', [Xg_obj.pose(1).trans.xyz; Xg_obj.pose(2).trans.xyz; Xg_obj.pose(3).trans.xyz]);
        fprintf('\nFeature_1: ');
        fprintf('%f ', Xg_obj.feature(1).xyz');
        fprintf('...]\n');  
        
        if(PreIntegration_options.bMalaga == 1)
            z2= Xg_obj(4);% Timu(:,4) correspond to Tcam(:,6)==> x-z
        elseif(PreIntegration_options.bDinuka == 1)
            %z2 = Xg_obj(6);
            z2 = Xg_obj.pose(2).trans.xyz(3);
        end         
        
        if(PreIntegration_options.bInitPnF5VoU == 0)
            x = Xg_obj;
            if(bAddInitialNoise == 1)
                x = x + 1e-2*(rand(size(x)) - 0.5);
            end
        end
        % Display X0
        fprintf('\nInitial Value:\n\t X0=[\nAng: ');
        %fprintf('%f ', x(1:20));
        fprintf('%f ', [X_obj.pose(1).ang.val; X_obj.pose(2).ang.val; X_obj.pose(3).ang.val]);
        fprintf('\nTrans: ');
        fprintf('%f ', [X_obj.pose(1).trans.xyz; X_obj.pose(2).trans.xyz; X_obj.pose(3).trans.xyz]);
        fprintf('\nFeature_1: ');
        fprintf('%f ', X_obj.feature(1).xyz');
        fprintf('...]\n');    
    %ie = x-xg;
    ie = XObject2Vector( XObjectDiff(X_obj, Xg_obj) );
    [me, id] = max(abs(ie))   
    if(nPoseNew == 25)
        aa = 1;
    end
    
    % Show Pose-feature graph
    if((PreIntegration_options.bShowFnP == 1) && ((nPoseOld == 1) || (nPoseNew == nAllposes)))
        fnShowFeaturesnPoses(Xg_obj, nPoseNew, nPts, nIMUdata, 'Ground Truth Values');
        fnShowFeaturesnPoses(X_obj, nPoseNew, nPts, nIMUdata, 'Initial Values');
    end

    %% Z---the observation vector
    Zobs = InertialDelta_CreateZ( nPoseNew, nPts );
    
    Zobs = InertialDelta_InitZ( Zobs, RptFeatureObs, nPoseNew, nPts, dp, dv, dphi, SLAM_Params );

    %% Save data for nonlin method.
    save( [ Data_config.TEMP_DIR 'initX.mat' ],'X_obj');
    %((dataIMU{2}(2, 1) - dataIMU{2}(1, 1)))*size(dataIMU{2},1);
    save( [ Data_config.TEMP_DIR 'consts.mat' ],'nIMUrate','K','Zobs','nPoseNew','nPts','SLAM_Params','dt','Jd');
    save( [ Data_config.TEMP_DIR 'Zobs.mat' ], 'Zobs'); 
    save( [ Data_config.TEMP_DIR 'RptFeatureObs.mat' ], 'RptFeatureObs'); 
    
    %% Covariance matrix
    CovMatrixInv = fnCalcCovMatrixInv( SLAM_Params, Zobs, Rd, nPoseNew, nIMUdata );
    save( [ Data_config.TEMP_DIR 'CovMatrixInv.mat' ], 'CovMatrixInv', '-v7.3');    


    tic
    if(PreIntegration_options.bGNopt == 1)
    %% GN Iterations 
        [X_obj, nReason] = fnGaussNewton_GraphSLAM(K, X_obj, nPoseNew, nPts, Jd, CovMatrixInv, ...
                        nMaxIter, fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ...
                        ImuTimestamps, dtIMU, RptFeatureObs );
        nReason
    else    
        [X_obj,nReason,Info] = fnLeastSqrLM_GraphSLAM(nUV, K, X_obj, nPoseNew, nPts, Jd, ...
            CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs);        
    end        
    
    toc
    
    fprintf('\n###Poses[x(1-%d)], Features[x(%d-%d)], Velocity[x(%d-%d]###\n',...
        nIMUdata*6, nIMUdata*6+1, ...
        nIMUdata*6+nPts*3, nIMUdata*6+nPts*3+1, ...
        nIMUdata*6+nPts*3+3*(nIMUdata+1));
    
    % Check against the ground truth
    %xf = x;
    %load('Xgt.mat');%xf

    ef = XObject2Vector( XObjectDiff( X_obj, Xg_obj ));
    [maxe, idx] = max(abs(ef));
    fprintf('Final Error: maxXef=%f, idx=%d\n', maxe, idx);

    if(nReason < 0)
        return;
    end
    
    if((nPoseOld == 1) || (nPoseNew == nAllposes))
        Tcam = zeros(3, nPoseNew);
        Timu = Tcam;        
            
        for(pid = 2:nPoseNew)
            cid = ImuTimestamps(pid)-ImuTimestamps(1);
            Rimu = fnR5ABG(X_obj(6*(cid-1)+1), X_obj(6*(cid-1)+2), X_obj(6*(cid-1)+3));
            Timu(:, pid) = X_obj((6*(cid-1)+4):(6*(cid-1)+6),1);
            Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
        end        
        
        load( Data_config.gtFile );        
        figure(); hold on;
        if(PreIntegration_options.bMalaga == 1)
            % plot(GT_P0(:,4),GT_P0(:,6),'-+r');
            % plot(Tcam(1,:),Tcam(3,:),'-*b');
        elseif(PreIntegration_options.bDinuka == 1)
            Tcam = Timu;
            GT_P0 = gtIMUposes(selpids(1:nPoseNew),2:7);
            GT_P0(:,4:6) = GT_P0(:,4:6) - repmat(GT_P0(1,4:6),nPoseNew,1);
            %   plot(gtIMUposes(selpids(1:nPoses),5)-gtIMUposes(selpids(1),5),...
            %           gtIMUposes(selpids(1:nPoses),7)-gtIMUposes(selpids(1),7),'-+r');
            %   plot(Timu(1,:),Timu(3,:),'-*b');
        end
        
        %	plot(GT_P0(:,4),GT_P0(:,6),'-+r');
        %   plot(Tcam(1,:),Tcam(3,:),'-*b');    
        plot3(GT_P0(:,4),GT_P0(:,5),GT_P0(:,6),'-+r');
        plot3(Tcam(1,:),Tcam(2,:),Tcam(3,:),'-*b');
        %     plot3(Timu(1,:),Timu(2,:), Timu(3,:),'-pg');
        view(-45,30);
        %     axis equal;
        title('Comparison of Estimated Poses Against the Ground Truth');

        figure();
        err = Tcam(:,1:nPoseNew) - GT_P0(1:nPoseNew, 4:6)';
        ce = complex(err(1,:), err(2,:));
        ce = complex(abs(ce), err(3,:));  
        plot(1:nPoseNew, abs(ce), 'p');
        title('Pose Translational Error');
        
    %%%%%%%%%%%%    
        save( [ Data_config.TEMP_DIR 'x_Jac.mat' ], 'X_obj');
        
        %% Show pose-feature graph
        if((PreIntegration_options.bShowFnP == 1) && (nPoseNew == nAllposes))
            fnShowFeaturesnPoses_general(X_obj, nPoseNew, nPts, nIMUdata, 'Final Values');
            
            %% Show uncertainty
            fnCalcShowUncert_general(nUV, RptFeatureObs, ImuTimestamps, ...
                dtIMU, ef, K, X_obj, nPoseNew, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata );
        end
        
    end
    
    x_old = X_obj;
    nPoseOld = nPoseNew;
    nPoseNew = nPoseNew + nAddPoses;
    
    if(nPoseNew > nAllposes)
        nPoseNew = nAllposes;
    end
    
    RptFidSet_old = RptFidSet;
    nIMUdata_old = nIMUdata;
    
end

return;    
    
