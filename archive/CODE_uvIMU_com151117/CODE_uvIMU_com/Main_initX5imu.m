%% The main function.
clear;
close all;
clc;

%% Choose to use simulated data or or real data.
bSimData = 0;% p15-30
% Select one of the real datasets
bMalaga = 0;
bDinuka = 1;%p4-15

if(bSimData + bDinuka + bMalaga > 1)
    fprintf('Only one of [bSimData, bDinuka, bMalaga] could be 1, all of the rest 0s.');
    return;
end

%% Configure the conditions of the problem
nFullposes = 120;
nPoseOld = 0;
nPoseNew = 10;
nPoses = 10;%9;%30;%15;%120;%170;%4;%50;%5;%6;%10;%20;%3;%2;%60;%80;%13;%12;%170;% 200;%350;%30;%14;%60;%5;%40;%1000;%350;%15;%1200;%30;%170;%120;%7;%50;%120;%30;%60;%170;%
bPreInt = 1;%0;% Use pre-integration method?
    
bInitPnF5VoU = 1;% Use visual odometry or IMU data to initialize x?
bIMUodo = 1;% Use IMU data to initilize x?
bGNopt = 1;% Use Gauss-Newton method?
bShowFnP = 0;% Show poses and features?
bShowUncertainty = 1;% Show uncertainty of the result?
bAddInitialNoise = 0;% x0 + noise or not
kfspan = 10;%20;%1;%5;%15;%50;%30;%2;% Choose keyframes
kfids = 1:kfspan:1200;
nMinObsTimes = 3;%2;%5;%4;%6;%
fMaxDistance = 50;%1e2;%5e2;%80;%30;%20;%10;%84;%60;
nPts = 60000;%15000;%6640;%3243;%800;%14172;%1614;%58404;%
bPerfectIMUdlt = 0;
bVarBias = 0;

bUVonly = 0;% If 1, only UVs are used; otherwise, IMU and UVs are fused.

bf0 = zeros(3,1);%[-0.55;0.6;0.61];
bw0 = zeros(3,1);
uvd_cell = [];
dp = zeros(3,nPoses);
dv = dp; 
dphi = dp;
dtIMU = [];

% if(bPreInt == 1)
%    bShowFnP = 1;
% else
%    bShowFnP = 0; 
% end

%% Configure paramters according to the chosen dataset
if(bSimData == 1)
    % Add noise to the observation vector z
    %bZnoise = 1;%fZnoisescale = 1;%6;%1e2;%12; % bXnoise = 1;
    fXnoisescale = 0;%1e-6;%1e-2;%1e0;%5e-2;%3e-2        
    nIMUrate = 5e2;
    sigmaw_real = 0.03; 
    sigmaf_real = 0.2;%0.    
    sigmaw_cov = sigmaw_real;%1;%1e-3; 
    sigmaf_cov = sigmaf_real;%1;%1e-3;%0.0775; % Siga for the noises to be added to the IMU raw data (wi,ai)
    sigmauov_real = 1e-2;%1;% 
    sigmauov_cov = sigmauov_real;%1;
    sigmad_real = 0;%
    sigmad_cov = 0.1;
    sigmabf_real = 0;% 1e-3;
    sigmabw_real = 0;% 1e-3;
    sigmabf_cov = 1;
    sigmabw_cov = 1;
    sigmatv = 1e-4/(nIMUrate);
    
    nPts = 100;%1e3;%10;%50;%4;%1;%2;%20;% 
    
    % Configure pseudo observations
    bAddZg = 1; % Add pseudo observation of g or not
    bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
    bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
    bAddZbf = 1; % Add pseudo observation of bias in acceleration or not
    bAddZbw = 1; % Add pseudo observation of bias in rotation or not
    
    g0 = [0; 0; -9.8]; % g value in the first key frame
    g_true = [0; 0; -9.8];
    bf_true = [1.0; 2.0; 3.0]; % bias for acceleration
    bw_true = [1.0; 2.0; 3.0]; %[0, 0, 0]'; % bias for rotaion velocity 
    imgdir = [];
    ImuTimestamps = 1:(nIMUrate):((nPoses-1)*nIMUrate+1);
    nUV = nPts*2*nPoses;
    nIMUdata = (nPoses-1)*nIMUrate;
    dtIMU = ones(nPoses, 1);
    RptFeatureObs = zeros(nPts, 50);%[fid, nObs, [pid, ui,vi]]
    fId_FeatureObs = 1; nObsId_FeatureObs = 2;
    dt = 1.0/nIMUrate;
elseif(bMalaga == 1)
    nIMUrate = 1e2;
    sigmaw_real = 1e0*(5e-2)*pi/180;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    sigmaf_real = 2e-3;    
    sigmaw_cov = 1e0*(5e-2)*pi/180;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    sigmaf_cov = 2e-3;%1e-2;%1e-5;%1;%1e0;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%1e0;%2e-3;%
    sigmauov_real = 1;%1e-6;%1e-4;%1e-3;%
    sigmauov_cov = 1e-3;%1;
    sigmabf=1*2e-2;
    sigmabw = 1*20*pi/(180*3600); 
    sigmatv = 1e-4/(nIMUrate);
    % Configure pseudo observations
    bAddZg = 1; % Add pseudo observation of g or not
    bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
    bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
    bAddZbf = 0; % Add pseudo observation of bias in acceleration or not
    bAddZbw = 0; % Add pseudo observation of bias in rotation or not   
    
    g0 = [0; 0; -9.8]; % g value in the first key frame
    g_true = [0; 0; -9.8];
    bf_true = [1.0; 2.0; 3.0]; % bias for acceleration
    bw_true = [1.0; 2.0; 3.0]; %[0, 0, 0]'; % bias for rotaion velocity     
    % Directories
    datadir = ['.' filesep 'Whole170R' filesep 'Result' filesep];
    imgdir = ['.' filesep 'Whole170R' filesep];
    imufulldir = ['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
    gtFile = ['.' filesep 'Whole170R' filesep 'GT_P0_PA.mat'];    
elseif(bDinuka == 1)
    dtIMU = zeros(nPoses, 1);
    nIMUrate = 2e2;    
    sigmaw_real = 0.03;%0.01;%0.0;%0.15;% 5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    sigmaf_real = 0.2;%0.1;%0;%1;%  
    sigmaw_cov = sigmaw_real;%1;%0.03;%0.1;% 0.01;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    sigmaf_cov = sigmaf_real;%1;%0.2;%0.8;% 0.1;%1e-2;%1e-5;%1;%1e0;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%1e0;%2e-3;%
    sigmauov_real = 1e-2;%0;%2e-3;%1e-1;%1;%1e-6;%1e-3;%1e-4;%
    sigmauov_cov = sigmauov_real;%1e-2;%1;%
   
    sigmabf=1*2e-2;
    sigmabw = 1*20*pi/(180*3600); 
    sigmatv = 1e-4/(nIMUrate);%1;%*nIMUrate*nIMUrate);
    % Configure pseudo observations
    bAddZg = 1; % Add pseudo observation of g or not
    bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
    bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
    bAddZbf = 0; % Add pseudo observation of bias in acceleration or not
    bAddZbw = 0; % Add pseudo observation of bias in rotation or not    
    
    g0 = [0; 0; 9.81]; % g value in the first key frame
    g_true = [0; 0; 9.81];
    bf_true = [0; 0; 0]; % bias for acceleration
    bw_true = [0; 0; 0]; %[0, 0, 0]'; % bias for rotaion velocity     
    % Directories
    datadir = ['..' filesep '..' filesep 'Dinuka' filesep 'dataset_19_10_15' filesep];%Dinuka/dataset_19_10_15
    imgdir = datadir;
    imufulldir = [datadir 'imudata_nonoise.mat'];% small imudata_nonoise['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
    gtVelfulldir = [datadir 'velocity_ground_truth.mat'];
    gtFile = [datadir 'gtIMUposes.mat'];    
end


addpath(genpath('IMU'));
addpath(genpath('MoSeg_2D'));%addpath(genpath('ms3D'));
addpath(genpath('Ransac'));

save('bVarBias.mat','bVarBias');
   
%     sigmaw = 0.01/(nIMUrate);%0.03;%1e1*(5e-2)*pi/180;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
%     sigmaf = 0.1/(nIMUrate);
    

% Iteration times and bounds
nMaxIter = 1e3;%50;%30;%100;%15;%5;%10;%50;%3;% 20;% 
fLowerbound_e = 1e-10;%1e-6;%1e-5;%1e-1;
fLowerbound_dx = 1e-10;%1e-6;%

%dtIMU = [];
Jd =[];
Rd = [];

%% The main switch
if(bSimData)
    bUsePriorZ = 0; % Use pre-set observation value or not 
    bTestIMU = 0;
    % IMU data rate
%     nIMUrate = 1e2;%5e2;%50;%1e3;%2e2;%8e2;%6e2;
    % Number of points to be simulated.
    
    % Number of keyframes to be simulated.
%     nPoses = 5;%10;%50;%100;%350;%6;%60;%3;%4;%9;%7;%13;%20;%500;%150;%30;%14;%5;%30;%11;
    % Add noise to the state vector x
   
    
    % save the configured data
    save('bAddZg.mat','bAddZg');    
    save('bAddZau2c.mat','bAddZau2c');
    save('bAddZtu2c.mat','bAddZtu2c');
    save('bAddZbf.mat','bAddZbf');
    save('bAddZbw.mat','bAddZbw');
    % Show pose-feature graph or not
%     if(bPreInt == 1)
%         bShowFnP = 1;
%     else
%         bShowFnP = 0;
%     end
    bChIMUd = 0; % average the raw IMU data or not
    bPerfectIMUdlt = 0; % For the pre-integration method, use perfect IMU delta values or not    

    fbiascef = 3;

    
    idr = nPoses*nPts*2;% camera observations: (ui,vi) 
    
    if(bPreInt == 0)
        nlenpp = nIMUrate*3*3; % IMU raw data observations between two key frames: (wi,ai,dTi)
        Zobs = zeros(idr+(nPoses-1)*nlenpp, 1);
    else
        Zobs = zeros(idr+(nPoses-1)*9, 1); % Number of observations for pre-integration: (ui,vi) + (dpi,dvi,dphii) 
    end
    
   
    %% Other parameters for the simulated scenario
	cx0 = 240; cy0 = 320; f = 575; % camera intrinsic parameters
	K = [f 0 cx0; 0 f cy0; 0 0 1]; 
    Ru2c = fRx(-pi/2); % Relative rotation
    Tu2c = [0; 0; 0]; % Relative translation
%     bf0 = [0, 0, 0]'; % bias for acceleration
%     bw0 = [0, 0, 0]'; % bias for rotaion velocity
%     g0 = [0; 0; -9.8]; % g value in the first key frame
    
 %% Use previous observation data or not
if(bUsePriorZ == 0)   
    bf0 = [1.0, 2.0, 3.0]'; % bias for acceleration
    bw0 = [1.0, 2.0, 3.0]'; %[0, 0, 0]'; % bias for rotaion velocity    
    arPts = [0,0,0; 1,0.5,2;0.5,2,2.6;1.8,2.4,3.5];

%     [imuData_cell, uvd_cell, Ru_cell, Tu_cell, Ru2c, Tu2c, vu] = ...
%         fnSimIMUnCameraFeatures1Pts(arPts, nPts, Ru2c, Tu2c, g0, bf0, bw0, nIMUrate, bPreInt);%
    [imuData_cell, uvd_cell, Ru_cell, Tu_cell, Ru2c, Tu2c, vu] = ...
        fnSimIMUnFeaturesAtNPoses_helix(nPoses, nPts, Ru2c, Tu2c, g0, bf0, bw0, nIMUrate, bPreInt);% line   

    
    if(bTestIMU == 1)
        fnTestIMU_cmp(nPoses, imuData_cell, Ru_cell, Tu_cell, ...
                            vu, bPreInt, bf0, bw0, g0);
    end
    
    %% Add noise to observations
    FeatureObs = zeros(nPts, 300);%[fid, nObs, [pid, ui,vi]]
%     fId_FeatureObs = 1; nObsId_FeatureObs = 2;
    FeatureObs(:, fId_FeatureObs) = 1:nPts;    
%     if(bZnoise == 1)
        % Add noise to the uvd data
        [~,nc] = size(uvd_cell{1});                
        for pid=1:nPoses
%             [gns] = fnGenGaussNoise(1, nc, [1;1;0.01]);
%             uvd_cell{pid} = uvd_cell{pid} + gns;  
            [gns] = fnGenGaussNoise(2, nc, sigmauov_real);
            uvd_cell{pid}(1:2,:) = uvd_cell{pid}(1:2,:) + gns;
            FeatureObs(:,nObsId_FeatureObs) = FeatureObs(:,nObsId_FeatureObs)...
                + 1;
            nObs = FeatureObs(:, nObsId_FeatureObs);
            FeatureObs(:, 3*nObs) = pid;
            FeatureObs(:, (3*nObs+1):(3*nObs+2)) = (uvd_cell{pid}(1:2,:))';
            [gns] = fnGenGaussNoise(1, nc, sigmad_real);
            uvd_cell{pid}(3,:) = uvd_cell{pid}(3,:) + gns;            
        end
        RptFeatureObs = FeatureObs;
        % Add noise to imu data
        [gns] = fnGenGaussNoise(3, 1, sigmabf_real);
        bf0 = bf0+gns;%fbiascef*sigmaf;
        [gns] = fnGenGaussNoise(3, 1, sigmabw_real);
        bw0 = bw0+gns;%fbiascef*sigmaw;
        [nr,nc] = size(imuData_cell{2}.samples(:, 2:4));
        for pid=2:nPoses
%             imuData_cell{pid}.samples(:, 2:7) = imuData_cell{pid}.samples(:, 2:7) + ...
%                 repmat([sigmaw*ones(1,3),sigmaf*ones(1,3)], nIMUrate,1) .* randn(size(imuData_cell{pid}.samples(:, 2:7)))/fZnoisescale;
            [gns] = fnGenGaussNoise(nr, nc, sigmaw_real);
            imuData_cell{pid}.samples(:, 2:4) = ...
                imuData_cell{pid}.samples(:, 2:4) + gns;
            [gns] = fnGenGaussNoise(nr, nc, sigmaf_real);
            imuData_cell{pid}.samples(:, 5:7) = ...
                imuData_cell{pid}.samples(:, 5:7) + gns;
        end      
%     end  
    save('SimuData.mat', 'imuData_cell', 'uvd_cell', 'Ru_cell', 'Tu_cell', ...
        'Ru2c', 'Tu2c', 'vu', 'bf0', 'bw0');
else

    load('SimuData.mat');
           
%     Zobs1(1:tid) = Zobs(1:tid); % Only substitute the uv+IMU part
%     Zobs = Zobs1;

end    

%     if(bPreInt == 1)
%         tid = idr + (nPoses - 1)*9;
%     else
%         tid = idr + (nPoses - 1)*nlenpp;
%     end 
    
    for pid=2:nPoses%1e-2,1e-3+3*sigmaf+3*sigmaw
        [dp(:,pid), dv(:,pid), dphi(:,pid), Jd{pid}, Rd{pid}] = ...
            fnDeltObsAccu(bf0, bw0, ...
            imuData_cell{pid}.samples, sigmaw_cov, sigmaf_cov); 
    end 

    %% Compose initial value x:
    % x = [(A_ui; T_ui); fp10(:);(V_ui);g;Au2c,Tu2c;bf;bw];
    %   size: [(nPoses-1)*6+nPts*3+3*nPoses + 15)x1] (pre-integration)
    %         [(nPoses-1)*nIMUrate*6+nPts*3+3*(nIMUrate*(nPoses-1)+1) + 15)x1] (non-pre-integration)
    % IMU poses, assuming keyframes are 1s away.    
    if(bPreInt == 0)
        x = zeros((nPoses-1)*nIMUrate*6+nPts*3+3*((nPoses-1)*nIMUrate+1)+15+6,1);% one additional 6 for convenience
    else
        x = zeros((nPoses-1)*6+nPts*3+3*nPoses+3+6+6, 1); 
    end 
    
    xg = x;
    
        %% 1. Pose_ui: Obtain the initial estimation of IMU poses (camera has the
        % following relation with IMU;
        %         Rc = Ru2c * Ru_cell{cid};% Left multiply R to comply with the vectors to be multiplied on the right.
        %         Tc = Tu_cell{cid} + (Ru_cell{cid})' * Tu2c;    
    if(bInitPnF5VoU == 1)
        for pid=2:(nPoses-1)
            % Pick out T corresponding to the current pose 
            Timu = Tu_cell{pid}(:,1) - Tu_cell{pid-1}(:,1);
            fscaleGT(pid) = norm(Timu);
        end
        % The final pose
        Timu = Tu_cell{nPoses} - Tu_cell{nPoses-1}(:,1);
        fscaleGT(nPoses) = norm(Timu);     
     
        if(bIMUodo == 1)
            %% Obtain initial poses from IMU data
            [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5IMUdata(dtIMU, g0, dp, dv, dphi,nPoses, nPts, ...
                    K, bSimData, bMalaga, RptFeatureObs, Tu2c, Ru2c);               
        else
            %% obtain relative poses from visual odometry
            [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5MatchedFeatures(nPoses, nPts, ...
                        K, fscaleGT, bSimData, RptFeatureObs, kfids);  
        end
%         [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5MatchedFeatures(nPoses, nPts, ...
%                 K, fscaleGT, bSimData, RptFeatureObs, kfids);
        ABGimu = zeros(3, nPoses);
        Timu = zeros(3, nPoses);
        for(pid=1:(nPoses))% correspond to pose 2...n
    %         Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
            Rimu = Ru2c'*Rcam(:,:,pid)*Ru2c;
            [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
            Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
        end
        if(bPreInt == 1)
            idstart  = 1; idend = 6*(nPoses - 1);
            tv = [ABGimu(:,2:end); Timu(:,2:end)];
            x(idstart:idend) = tv(:);
%             idx = (nPoses - 1)*6;
        else            
            [x] = fnLinearInterpPoses(nPoses, ABGimu, Timu, ImuTimestamps,x);
%             idx = (nPoses - 1)*nIMUrate*6;
        end
    end
%     else
%% Ground truth poses
        if(bPreInt == 1)
            for pid=2:(nPoses-1)
                % Pick out R & T corresponding to the current pose       
                [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid}{1});%Rc,Tc
                xg(((pid-2)*6+1):((pid-1)*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,1)];       
            end
            % The final pose
            [alpha, beta, gamma] = fABGfrmR(Ru_cell{nPoses});%Rc,Tc
            xg(((nPoses-2)*6+1):((nPoses-1)*6),1) = [alpha; beta; gamma; Tu_cell{nPoses}];        
            idx = (nPoses - 1)*6;
        else
            for pid=1:(nPoses-1)
                for(k=1:nIMUrate)
                % Pick out R & T corresponding to the current pose       
                [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid}{k});% Pick up the first key frame as well, so we need to get rid of it at L146
                xg(((pid-1)*nIMUrate*6+6*(k-1)+1):((pid-1)*nIMUrate*6+k*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,k)];       
                end
            end
            % The final pose
            [alpha, beta, gamma] = fABGfrmR(Ru_cell{nPoses});%Rc,Tc
            xg(((nPoses - 1)*nIMUrate*6+1):((nPoses - 1)*nIMUrate*6+6),1) = [alpha; beta; gamma; Tu_cell{nPoses}];
            xg = xg(7:end); % remove the initial camera pose.
            idx = (nPoses - 1)*nIMUrate*6;
        end
%     end
    
    %% 2. f_ui: Extract feature positions at the initial IMU pose.
    if(bInitPnF5VoU == 1)
        fp_c1 =  (Feature3D(:, 5:7))';% select the first group 
        fp_u1 = Ru2c' * fp_c1 + repmat(Tu2c, 1, size(fp_c1, 2));
        x((idx+1):(idx+nPts*3)) = fp_u1(:);    
    end
%     else        
        uvd1 = uvd_cell{1}; 
        fp_c1 = uvd1; 
        fp_c1(1, :) = (uvd1(1, :) - cx0) .* fp_c1(3, :) / f;
        fp_c1(2, :) = (uvd1(2, :) - cy0) .* fp_c1(3, :) / f;
        fp_u1 = Ru2c' * fp_c1 + repmat(Tu2c, 1, size(fp_c1, 2));
        xg((idx+1):(idx+nPts*3)) = fp_u1(:);
%     end    
%     % Ru2c * (Pui - Tu2c) = Pci  ==> Pui = Ru2c' * Pci + Tu2c
%     fp_u1 = Ru2c' * fp_c1 + repmat(Tu2c, 1, size(fp_c1, 2));
%     x((idx+1):(idx+nPts*3)) = fp_u1(:);
    %% 3. Vi: Initial velocity for each pose
    idx = idx+nPts*3;
    if(bInitPnF5VoU == 1)
       [x, ~] = fnInitVelocity(nPoses, bPreInt, x, idx, g0, bf0, dp, dv, ...
           dtIMU, imuData_cell, nIMUdata, nIMUrate, dt); 
    end
%     else
        % Special case for Pose 1:        
        if(bPreInt == 1)
            xg((idx+1):(idx+3)) = imuData_cell{2}.initstates(7:9);
            for pid = 2:nPoses
                idx = idx + 3;
                xg((idx+1):(idx+3)) = imuData_cell{pid}.initstates(7:9);
            end
            idx = idx + 3;
        else
            vu = vu';
            xg((idx+1):(idx+((nPoses-1)*nIMUrate+1)*3)) = vu(:);
            idx = idx+((nPoses-1)*nIMUrate+1)*3;
        end
%     end
    %% 4. g:%(nPoses-1)*6+nPts*3+3*nPoses (nPoses-1)*6+nPts*3+3*nPoses 
    x((idx+1):(idx+3)) = g0;
    xg((idx+1):(idx+3)) = g_true;
    idx = idx +3;
    %% 5. Ru2c, Tu2c:(nPoses-1)*6+nPts*3+3*nPoses+4:(nPoses-1)*6+nPts*3+3*nPoses + 9
    [alpha, beta, gamma] = fABGfrmR(Ru2c);
    x((idx+1):(idx+6)) = [alpha;beta;gamma;Tu2c];
    xg((idx+1):(idx+6)) = [alpha;beta;gamma;Tu2c];
    idx = idx + 6;
    %% 6. bf,bw %(nPoses-1)*6+nPts*3+3*nPoses+10
    x((idx+1):(idx+6)) = [bf0;bw0]; 
    xg((idx+1):(idx+6)) = [bf_true;bw_true];
    idx = idx + 6;
    xg = xg(1:idx);
    x = x(1:idx);
    %% Display Xgt
    fprintf('Ground Truth Value:\n\t Xg=[');
    fprintf('%f ', xg);
    fprintf(']\n');    
%     save('Xgt.mat','xg');
    % Show Pose-feature graph
    
    if(bInitPnF5VoU == 0)
        x = xg;
    end
% Add noise to the state vector x
if(bAddInitialNoise == 1) 
    nc = 1; idst = 0;    
    % 1. Add to IMU poses
    if(bPreInt == 1)
        nr = 6*(nPoses - 1);
    else
        nr = 6*nIMUrate*(nPoses - 1);
    end
    [gns] = fnGenGaussNoise(nr, nc, fXnoisescale);
    x((idst+1):(idst+nr)) = x((idst+1):(idst+nr)) + gns; % x = x + gns;[nr,nc] = size(x((idr+1):end));    
    idst = idst + nr + 3*nPts; % Already added noises to feature positions    
    % 2. Add to IMU velocity
    if(bPreInt == 1)
        nr = 3*nPoses;
    else
        nr = 3*(nIMUrate*(nPoses-1)+1);
    end
    [gns] = fnGenGaussNoise(nr, nc, fXnoisescale);
    x((idst+1):(idst+nr)) = x((idst+1):(idst+nr)) + gns;    
    idst = idst + nr;
    % 3. Add to g, Au2c, Tu2c, bf, bw
    nr = 3*5;
    [gns] = fnGenGaussNoise(nr, nc, fXnoisescale);
    x((idst+1):(idst+nr)) = x((idst+1):(idst+nr)) + gns;  
end
% Show initial value
    fprintf('Initial Value:\n\t X0=[');
    fprintf('%f ', x);
    fprintf(']\n'); 

    if(bShowFnP == 1)
        fnShowFeaturesnPoses(xg, nPoses, nPts, nIMUrate, bPreInt, 'Ground Truth Values');
        fnShowFeaturesnPoses(x, nPoses, nPts, nIMUrate, bPreInt, 'Initial Values');
    end    
    
%% Z: Put camera observations into Z
%     for pid=1:nPoses % Old sequence: pose oriented
%         t = uvd_cell{pid}(1:2, :);  
%         Zobs(((pid-1)*2*nPts+1):(pid*2*nPts),1) = t(:);%uvd_cell{pid}
% %     	Zobs(((pid-1)*3*nPts+1):(pid*3*nPts),1) = uvd_cell{pid}(:);
%     end 
    RptFeatureObs(:, fId_FeatureObs) = (1:nPts)';
    for(pid=1:nPoses)
        obsfeatures_i1 = uvd_cell{pid}(1:2, :);
        fidset = (1:nPts)';
        RptFeatureObs(fidset, nObsId_FeatureObs) = ...
            RptFeatureObs(fidset, nObsId_FeatureObs) + 1;
        for(fid=1:size(fidset,1))
            nObs = RptFeatureObs(fidset(fid), nObsId_FeatureObs);
            RptFeatureObs(fidset(fid), 3*nObs) = pid;
            RptFeatureObs(fidset(fid), (3*nObs+1):(3*nObs+2)) = (uvd_cell{pid}(1:2, fid))';
        end
    end   
    save('RptFeatureObs.mat', 'RptFeatureObs');
    % Order UVs according to fid
    zidend = 0;     
    for(fid=1:nPts)% local id
        nObs = RptFeatureObs(fid, nObsId_FeatureObs);
        for(oid=1:nObs)
            tv = (RptFeatureObs(fid, (oid*3+1):(oid*3+2)))';%+ ...
                %fnGenGaussNoise(2, 1, sigmauov);
            zidstart = zidend + 1; zidend = zidend + 2;%*size(tv, 2);
            Zobs(zidstart:zidend, 1) = tv;%(:);
        end
    end
    nUV = zidend;
    idr = zidend;
    
%     %% Prepare IMU observations  
%     % Average the raw data
%     if(bChIMUd == 1)
%         for pid=2:nPoses
%             tv = imuData_cell{pid}.samples(:, 2:7);
%             imuData_cell{pid}.samples(1:(end-1), 2:7) = 0.5 * (tv(2:end, :) + tv(1:(end-1), :));
%         end
%     end  

    % Put IMU data into observation vector z: 
    if(bPreInt == 0) % Non-pre-integration: put raw data  
        for pid=2:nPoses
            tv = [imuData_cell{pid}.samples(:, 2:7),zeros(nIMUrate,3)]';
            Zobs((idr+(pid-2)*nlenpp+1):(idr+(pid-1)*nlenpp)) = tv(:);
        end 
        utid = idr + (nPoses - 1)*nlenpp;
    else  % Generate pre-integration observations based on IMU raw data.   
        % Add interated IMU observations
        if(bPerfectIMUdlt == 0)
            Zobs((idr+1):(idr+9*(nPoses-1)),1) = reshape([dp(:,2:end);dv(:,2:end);dphi(:,2:end)],[],1);
        else
            dt = 1;
            Zobs((idr+1):(idr+9*(nPoses-1)),1) = fnCalPerfectIMUdlt(x, nPoses, nPts, Jd, dt, bf0, bw0); 
        end
        utid = idr + (nPoses - 1)*9;
    end

    %% Continue filling in Zobs with psedu observations related to IMU
%     if(bPreInt == 1)
%         if(bPerfectIMUdlt == 1)
%            dt = 1;
%            Zobs((idr+1):(idr+9*(nPoses-1)),1) = fnCalPerfectIMUdlt(x, nPoses, nPts, Jd, dt, bf0, bw0); 
%         end
%         tid = idr + (nPoses - 1)*9;
%     else
%         tid = idr + (nPoses - 1)*nlenpp;
%     end

    if(bAddZg == 1)
        % Add pseudo observation of g        
        Zobs((utid+1):(utid+3)) = g0; 
        utid = utid + 3;
    end
    if(bAddZau2c == 1)
        % Add pseudo observation of Tu2c
        [alpha, beta, gamma] = fABGfrmR(Ru2c);
        Zobs((utid+1):(utid+3)) = [alpha;beta;gamma];
        utid = utid + 3;
    end
    if(bAddZtu2c == 1)
        % Add pseudo observation of Tu2c
        Zobs((utid+1):(utid+3)) = Tu2c;
        utid = utid + 3;
    end
%     if(bAddZgantu2c == 1)%bAddZantu2c
%         % Add pseudo observation of g, Au2c,Tu2c
%         [alpha, beta, gamma] = fABGfrmR(Ru2c);
%         Zobs((tid+1):(tid+15)) = [g0;alpha;beta;gamma;Tu2c;bf0;bw0]; 
%         tid = tid + 3;
%     end
    if(bAddZbf == 1)
        % Add pseudo observation of bf
        Zobs((utid+1):(utid+3)) = bf0; 
        utid = utid + 3;            
    end
    if(bAddZbw == 1)
        % Add pseudo observation of bf
        Zobs((utid+1):(utid+3)) = bw0; 
        utid = utid + 3;            
    end
    Zobs = [Zobs(1:utid)];%(idr+9*(nPoses-1))

% %% Add noise to the uv part of Z    
% if(bZnoise == 1)    
%     % Only add noise to UV here, because IMU data has been added before.
% %     Zobs(1:idr) = Zobs(1:idr) + randn(size(Zobs(1:idr,1)));% fZnoisescale;%
%     nr = idr; nc = 1;
%     [gns] = fnGenGaussNoise(nr, nc, 1);
%     Zobs(1:idr) = Zobs(1:idr) + gns;
% end

%% Save data for nonlin method.
save('initX.mat','x');
dt = ((imuData_cell{2}.samples(2, 1) - imuData_cell{2}.samples(1, 1)))*size(imuData_cell{2}.samples,1);
save('consts.mat','nIMUrate','bPreInt','K','Zobs','nPoses','nPts','bf0','bw0','dt','Jd');

%% Covariance matrix
% Original     
%     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3));
if(bPreInt == 1)
    utid = idr+(nPoses-1)*3*3+15;% initialized with possible maximum size.
else
    utid = idr + (nPoses - 1)*nlenpp+15;
end
CovMatrixInv = speye(utid);
% Initialize the part corresponding to IMU data
if(bPreInt == 1)
    for pid = 2:nPoses
        covInv = 1e0*inv(Rd{pid}(1:9,1:9));    
        CovMatrixInv((idr+9*(pid-2)+1):(idr+9*(pid-1)), (idr+9*(pid-2)+1):(idr+9*(pid-1))) = covInv;
    end
    utid = idr+(nPoses-1)*3*3;
else
    q = inv(diag([sigmaw_cov*sigmaw_cov*ones(3,1); sigmaf_cov*sigmaf_cov*ones(3,1); sigmatv*sigmatv*ones(3,1)]));
    for pid = 1:((nPoses-1)*nIMUrate)
    %CovMatrixInv((idr+1):end,(idr+1):end) =
    % kron(eye((nPoses-1)*nIMUrate),q); % not suitalbe for large scale
    % computation
        CovMatrixInv((idr+9*(pid-1)+1):(idr+9*(pid)), (idr+9*(pid-1)+1):(idr+9*(pid))) = q;
    end
    utid = idr+(nPoses-1)*nlenpp;
end
% Initilized additional parts
    if(bAddZg == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(bAddZau2c == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
        if(bAddZtu2c == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(bAddZbf == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end 
    if(bAddZbw == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    
    CovMatrixInv = CovMatrixInv(1:utid,1:utid);
save('CovMatrixInv.mat','CovMatrixInv', '-v7.3');
    
% %% GN Iterations    
%     [x] = fnVI_BA(K, x, nPoses, nPts, dt, Jd, CovMatrixInv, nMaxIter, ...
%         fLowerbound_e, fLowerbound_dx, nIMUrate, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);

        save('ImuTimestamps.mat', 'ImuTimestamps');
        save('dtIMU.mat', 'dtIMU');    
tic
    if(bGNopt == 1)
    %% GN Iterations 
        [x] = fnVI_BA_general(nUV, K, x, nPoses, nPts, Jd, CovMatrixInv, ...
            nMaxIter, fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ...
            ImuTimestamps, dtIMU, RptFeatureObs, bUVonly, bPreInt, ...
            bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
    else    
        [x,Reason,Info] = fnleastsquaresLM(nUV, K, x, nPoses, nPts, Jd, ...
            CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
            bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);        
%         [x,Reason,Info] = fnleastsquaresLM(nUV, K, x, nPoses, nPts, Jd, CovMatrixInv, nMaxIter, ...
%             fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
%             bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);
    end
toc    

    if(bPreInt == 0)
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
    % Check against the ground truth
    xf = x;
%     load('Xgt.mat');
    ef = xf - xg;
    [maxe, idx] = max(abs(ef));
    fprintf('Final Error: maxXef=%f, idx=%d\n', maxe, idx);
%     fprintf('Final Value:\n\t Xf=[');
%     fprintf('%f ', xf);
%     fprintf(']\n');    
%%%%%%%%%%%%    
save('x_Jac.mat', 'xf');
%% Show pose-feature graph
if(bShowFnP == 1)
    fnShowFeaturesnPoses(xf, nPoses, nPts, nIMUrate, bPreInt, 'Final Values');
end
    %% Show uncertainty
    if(bShowUncertainty == 1)
    %     fnCalnShowUncert(ef, K, x, nPoses, nPts, dt, Jd, CovMatrixInv, ...
    %         nIMUrate, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);
        fnCalnShowUncert_general(bUVonly, nUV, RptFeatureObs, ImuTimestamps, ...
            dtIMU, ef, K, x, nPoses, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata, bPreInt, ...
            bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);    
    end

    return;

else
    if(bMalaga == 1)
        K = [923.5295, 0, 507.2222; 0, 922.2418, 383.5822; 0, 0, 1];% Left
        % [911.3657, 0, 519.3951; 0, 909.3910, 409.0285; 0, 0, 1]; &Right
    elseif(bDinuka == 1)
       load([datadir 'cam.mat']); 
       K = cam.K;
    end   
    
    
    if(bMalaga == 1)
        Au2c = [-87.23; -2.99; -88.43]*pi/180;%[0;0;0];%[-86.19;-3.53;-90.31]*pi/180;%
        Ru2c = fnR5ABG(Au2c(1), Au2c(2), Au2c(3));
        Tu2c = [2.2-0.25;-0.427-0.029;0.025+(23-13.9)*1e-3];%[0;0;0];%
        %nIMUrate = 100; 
        dt = 1e-2;
    %     bPerfectIMUdlt = 1;   
%         bAddZg = 1; % Add pseudo observation of g or not
%         bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
%         bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
%         bAddZbf = 0; % Add pseudo observation of bias in acceleration or not
%         bAddZbw = 0; % Add pseudo observation of bias in rotation or not    
    
    elseif(bDinuka == 1)
        load([datadir 'gtIMUposes.mat']);% ts, Aimu, Timu
        nt = size(gtIMUposes, 1); 
        selpids = 9:(10*kfspan):nt;
        Ru2c = ([0,1,0; 0,0,1; 1,0,0]); 
        Au2c = zeros(3,1);
        [Au2c(1), Au2c(2), Au2c(3)]= fnABG5R(Ru2c);
        Tu2c = zeros(3,1);
        %nIMUrate = 200; 
        dt = 1.0/nIMUrate;
%     bPerfectIMUdlt = 1;
%         bAddZg = 1; % Add pseudo observation of g or not
%         bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
%         bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
%         bAddZbf = 1; % Add pseudo observation of bias in acceleration or not
%         bAddZbw = 1; % Add pseudo observation of bias in rotation or not           
    end    
%     g0 = [0; 0; -9.8]; 

    % save the configured data
    save('bAddZg.mat','bAddZg');    
    save('bAddZau2c.mat','bAddZau2c');
    save('bAddZtu2c.mat','bAddZtu2c');
    save('bAddZbf.mat','bAddZbf');
    save('bAddZbw.mat','bAddZbw');  
    save('bUVonly.mat', 'bUVonly');
    
    %%%%%%%%%%%%%%
    % camera observations  

    FeatureObs = zeros(nPts, 500);%[fid, nObs, [pid, ui,vi]]
    fId_FeatureObs = 1; nObsId_FeatureObs = 2;
    FeatureObs(:, fId_FeatureObs) = 1:nPts;
    nPstart = 1; nPend = nPoses;
    [FeatureObs] = fnFillFeatureObs(bMalaga, bDinuka, nObsId_FeatureObs, ...
                sigmauov_real, FeatureObs, nPstart, nPend, kfids, imgdir);

    if(bMalaga == 1)
        load([datadir 'PBAFeature.mat']);
        RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoses, nMinObsTimes));
        RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
    % RptFidSet(586) = [];    
        RptFeatureObs = FeatureObs(RptFidSet, :);
    elseif(bDinuka == 1)
        %load([datadir 'feature_pos.mat']);
        RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoses, nMinObsTimes));
        %RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
        RptFeatureObs = FeatureObs(RptFidSet, :);        
    end
    
%     save('RptFeatureObs.mat', 'RptFeatureObs');
    % Arrange feature observations according to feature ids
% newimgdir = '/media/New Volume/uDocs/IMU/dataset/Liang/ParallaxBA2Shoudong/DataPrepareBA/Whole170R_New/';%['.' filesep 'Whole170R_New' filesep];    
% if(~exist(newimgdir, 'dir'))
%     mkdir(newimgdir);    
% end
% fnBldRptFeatureListFiles(RptFidSet, nPoses, imgdir, newimgdir);
   
%     load([datadir 'PBAFeature.mat']);
%     tv = PBAFeature(RptFidSet, :)'; %% Global ids %only pickup repeated features 
%     negids = find(tv(3,:) < 0);
%     idpt0(negids) = [];
%     bestmodel(negids) = [];    
    
    nPts = size(RptFidSet, 1);
%     tfids = fidset{1}; % Re-identify the points
%     obsfeatures{1} = [(1:nPts)', ... % obsfeatures{1}(tfids(bestmodel),1)
%         obsfeatures{1}(tfids(bestmodel),2:3)];
%     tfids = fidset{2};
%     obsfeatures{2} = [(1:nPts)', ... % obsfeatures{1}(tfids(bestmodel),1)
%         obsfeatures{2}(tfids(bestmodel),2:3)];    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    nIMUdata = 0;    
    ImuTimestamps = zeros(nFullposes, 1);    
    if(bUVonly == 0)
        % IMU observations
        if(bMalaga == 1)
            load([imgdir 'KeyframeTimestamps.mat']);
        elseif(bDinuka == 1)
            load([imgdir 'image_time_stamp.mat']);
            KeyframeTimestamps = vis_time(kfids);
        end
%         load('/home/youbwang/Documents/Malaga/IMUrawData.mat');    
        load(imufulldir);
        if(bMalaga == 1)
            imufulldata = IMUparking6L;
        elseif(bDinuka == 1)
            nt = size(imudata,1);
            imufulldata = imudata;% ts, wb,fb
            imufulldata(:,2:4) = imufulldata(:,2:4) + fnGenGaussNoise(nt, 3, sigmaw_real);
            imufulldata(:,5:7) = imufulldata(:,5:7) + fnGenGaussNoise(nt, 3, sigmaf_real);
        end
        utid = 1; %ctid = 1;
        for(ctid = 1:nFullposes)
            while(imufulldata(utid,1) < KeyframeTimestamps(ctid))
                utid = utid + 1;
            end
            if((imufulldata(utid,1) - KeyframeTimestamps(ctid)) >...
                   (KeyframeTimestamps(ctid) - imufulldata(utid-1,1)))
               ImuTimestamps(ctid) = utid - 1;
            else
               ImuTimestamps(ctid) = utid; 
            end
        end
        %save('ImuTimestamps.mat', 'ImuTimestamps');
        nIMUdata = ImuTimestamps(nPoses) - ImuTimestamps(1);
        dataIMU = {};     
        for pid=2:nFullposes
            uid0 = ImuTimestamps(pid-1);
            uid1 = ImuTimestamps(pid)-1;
%             nIMUdata = nIMUdata + ImuTimestamps(pid) - ImuTimestamps(pid-1);
            dtIMU(pid) = imufulldata(ImuTimestamps(pid)) - imufulldata(ImuTimestamps(pid-1));
            if(bMalaga == 1)
                dataIMU{pid} = [IMUparking6L(uid0:uid1, 1), IMUparking6L(uid0:uid1, 7), IMUparking6L(uid0:uid1, 6),...
                    IMUparking6L(uid0:uid1, 5), IMUparking6L(uid0:uid1, 2:4)];
            elseif(bDinuka == 1)
                dataIMU{pid} = imufulldata(uid0:uid1, :);                
            end
        end 
        if(bPreInt == 1)%bUVonly == 0)%
        % Generate pre-integration observations based on IMU raw data.          
            dp = zeros(3,nFullposes);
            dv = dp; 
            dphi = dp;    
            for pid=2:nFullposes%1e-2,1e-3+3*sigmaf+3*sigmaw
                [dp(:,pid), dv(:,pid), dphi(:,pid), Jd{pid}, Rd{pid}] = ...
                    fnDeltObsAccu(bf0, bw0, dataIMU{pid}, sigmaw_cov, sigmaf_cov); 
            end
        end
                
        save('ImuTimestamps.mat', 'ImuTimestamps');
        save('dtIMU.mat', 'dtIMU');
    end
    
    
    %% X---the state vector
    if(bUVonly == 1)
        x = zeros(6*(nPoses-1)+3*nPts+3*2,1);
    else
        if(bPreInt == 0)
            x = zeros((nIMUdata)*6+nPts*3+3*(nIMUdata+1)+15+6,1);%???(nPoses-1)*nIMUrate (nPoses-1)*nIMUrate+1 one additional 6 for convenience
        else
            x = zeros((nPoses-1)*6+nPts*3+3*nPoses+3+6+6, 1); 
        end  
    end   
    
    xg = x;
    
    % Based on Liang's results, pack the state vector x.
%     if(isunix)
%         datadir = '/home/youbwang/Documents/Malaga/Liang/ParallaxBA2Shoudong/DataPrepareBA/Whole170R/Result/';
%     else
%         datadir = 'E:\uDocs\Research\IMU\Liang\ParallaxBA2Shoudong\DataPrepareBA\Whole170R\Result\';
%     end
    % Intial values of poses and IMU states
    if(bMalaga == 1)
        load([datadir 'PBAPose.mat']);    
        tv = (PBAPose(1:nPoses, :))';
        
        ABGcam = [tv(3, :);tv(2, :);tv(1, :)];%tv(1:3, :);%
        Tcam = tv(4:6, :);           
        clearvars PBAPoses tv
        % Given Rcam/Tcam, Rimu/Timu can be calculated as follows:
        % Rimu = Ru2c' * Rc1u; Timu = Tc1u - Rimu '* Tu2c;
        % Rc1u = Rcam * Ru2c; Tc1u = Tu2c + Ru2c'*Tcam
        for(pid=1:(nFullposes))% correspond to pose 1...n
            Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
            Rimu = Ru2c'*Rcam*Ru2c;
            [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
            Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
        end   
        tv = [ABGimu(:, 2:end); Timu(:, 2:end)];
        if(bPreInt == 1)
           xg(1:((nPoses-1)*6)) = tv(:);
        else
           [xg] = fnLinearInterpPoses(nPoses, ABGimu, Timu, ImuTimestamps,xg);
        end
    elseif(bDinuka == 1)
        tv = (gtIMUposes(selpids(1:(nFullposes)), 2:7))';
        ABGimu = tv(1:3, :);
        Timu = tv(4:6, :);
        Tcam = zeros(3, nFullposes);
        for(pid=1:nFullposes)
            % Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
            Rimu = fnR5ABG(ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid));
            Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
        end
        if(bPreInt == 1)
            [xg] = fnCal9RelativePoses(xg, nPoses, tv);
        else
            pall = (gtIMUposes(selpids(1):(nIMUdata+selpids(1)), 2:7))';
            [xg] = fnCal9RelativePoses(xg, nIMUdata+1, pall);
        end
    end    
    fscaleGT = zeros(nFullposes, 1);
    for(pid=2:nFullposes)        
       Timu = Tcam(:, pid) - Tcam(:, pid-1); 
       fscaleGT(pid) = norm(Timu);
    end
    
    if(bInitPnF5VoU == 1)
        if(bIMUodo == 1)
            %% Obtain initial poses from IMU data
            [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5IMUdata(dtIMU, g0, dp, dv, dphi,nPoses, nPts, ...
                    K, bSimData, bMalaga, RptFeatureObs, Tu2c, Ru2c);               
        else
            %% obtain relative poses from visual odometry
            [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5MatchedFeatures(nPoses, nPts, ...
                        K, fscaleGT, bSimData, RptFeatureObs, kfids);  
        end
        ABGimu = zeros(3, nPoses);
        Timu = zeros(3, nPoses); % ABGimu = zeros(3, nPoses);            
        for(pid=1:(nPoses))% correspond to pose 1...n
    %         Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
            Rimu = Ru2c'*Rcam(:,:,pid)*Ru2c;
            [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
            Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%         tv = [ABGimu; Timu];
%         x(idstart:idend) = tv(:);
%     else
%     %     load('pbaPose10.mat');
%     %     tv = (Pose(2:nPoses, :))';
%         camposes = tv(:);
%         if(bDinuka == 1)
%             ABGimu = tv(1:3, 2:nPoses);%
%             Timu = tv(4:6, 2:nPoses) - repmat(tv(4:6,1),1,nPoses-1);
%             clearvars tv
%         elseif(bMalaga == 1)
% %             ABGcam = [tv(3, :);tv(2, :);tv(1, :)];%tv(1:3, :);%
% %             Tcam = tv(4:6, :);           
% %             clearvars PBAPoses tv
% %             % Given Rcam/Tcam, Rimu/Timu can be calculated as follows:
% %             % Rimu = Ru2c' * Rc1u; Timu = Tc1u - Rimu '* Tu2c;
% %             % Rc1u = Rcam * Ru2c; Tc1u = Tu2c + Ru2c'*Tcam
% %             for(pid=1:(nPoses-1))% correspond to pose 2...n
% %                 Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
% %                 Rimu = Ru2c'*Rcam*Ru2c;
% %                 [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
% %                 Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
% %             end
%         end
    end
    idstart = 1; 
    if((bUVonly == 1) || (bPreInt == 1))
        idend = 6*(nPoses-1);
%     else
%         idend = 0;%6*(nIMUdata-1);%nIMUrate*(nPoses-1);
%     end     
%     if((bUVonly == 1) || (bPreInt == 1))
        if(bInitPnF5VoU == 1)
            tv = [ABGimu(:, 2:end); Timu(:, 2:end)];% tv = [ABGimu; Timu];        
            x(idstart:idend) = tv(:);
        end
    else% For non-pre-int method, we need interpolation.
        idend = 6*nIMUdata;
%         for(pid=1:(nPoses-1))% only poses at keyframes are available
%             uid = ImuTimestamps(pid) - ImuTimestamps(1);
%             x((idstart+uid*6):(idstart+uid*6+5)) = [ABGimu(:,pid); Timu(:,pid)];         
%         end
        if(bInitPnF5VoU == 1)
            % Interpolation for the other poses, assuming uniform motion
            [x] = fnLinearInterpPoses(nPoses, ABGimu, Timu, ImuTimestamps,x);
        end
%         % Pose 1 is special
%         idend = 0;
%         ai = 0; bi = 0; gi = 0;
%         Ri = eye(3); 
%         Ti = zeros(3,1);
%         for(pid=1:(nPoses-1))% correspond to Pose 2...n
%             ai1 = ABGimu(1,pid);
%             bi1 = ABGimu(2,pid);
%             gi1 = ABGimu(3,pid);
%             Ri1 = fnR5ABG(ai1, bi1, gi1);
%             Ti1 = Timu(:,pid);
%             dR = Ri1*Ri';
%             [da, db, dg] = fnABG5R(dR);        
%             dT = Ti1 - Ti; 
%             nImu = ImuTimestamps(pid+1) - ImuTimestamps(pid);            
%             ts = 1:nImu;
%             ait = ai + da*ts/nImu;
%             bit = bi + db*ts/nImu;
%             git = gi + dg*ts/nImu;
%             Tit = repmat(Ti,1,nImu) + repmat(dT,1,nImu).*repmat(ts,3,1)/nImu;
%             idend = idend + nImu*6; 
%             tv = [ait;bit;git;Tit];
%             x(idstart:idend) = tv(:);
%             idstart = idend + 1;
%             Ri = Ri1; Ti = Ti1;
%             ai = ai1; bi = bi1; gi = gi1;            
%         end        
    end

    
    % Intial values of features at the initial IMU pose
    % Given Pf1c, Pf1u can be calculated as:
    % Pf1u = Ru2c'*Pf1c + Tu2c
    idstart = idend + 1; 
    idend = idend + 3*nPts;
%     load([datadir 'PBAFeature.mat']);
    if(bInitPnF5VoU == 1)
        tv =  (Feature3D(RptFidSet, 5:7))';% select the first group      
        Pf1u = Ru2c'*tv + repmat(Tu2c, 1, nPts);
        x(idstart:idend) = Pf1u(:);
    end
%   Fill in ground truth of features
    if(bMalaga == 1)
        tv = PBAFeature(RptFidSet, :)'; %% Global ids %only pickup repeated features
        Pf1u = Ru2c'*tv + repmat(Tu2c, 1, nPts);
    elseif(bDinuka == 1)
        load([datadir 'feature_pos.mat']);
        tv = feature_pos(RptFidSet, :)';
        abg10 = (gtIMUposes(selpids(1), 2:4))'; % Rotation of the IMU pose corresponding to the first key frame
        R10 = fnR5ABG(abg10(1, 1), abg10(2, 1), abg10(3, 1));
        Pf1u = R10'*(tv - repmat((gtIMUposes(selpids(1),5:7))', 1,size(RptFidSet,1)));
        clearvars gtIMUposes            
    end        
    xg(idstart:idend) = Pf1u(:);
%     load([datadir 'PBAFeature.mat']);
%     tv = PBAFeature(idpt0, :)'; %% Global ids %1:nPts  
%     negids = find(tv(3,:) < 0);
%     tv(3,negids) = -tv(3,negids);
%         if(bMalaga == 1)
%             
%         elseif(bDinuka == 1)
% 
%         end        
%     end

%     x(idstart:idend) = Pf1u(:);
    %% Initial values of velocity
%     clearvars PBAFeature tv
    if(bUVonly == 0)
        if(bInitPnF5VoU == 1)
            [x,idend] = fnCalV5Kposes(nIMUdata, ImuTimestamps, nIMUrate, bDinuka, bPreInt, x, nPoses, dtIMU, idend, dp, dv, g0, bf0, imufulldata);   
        elseif(bPreInt == 1)
           idend = idend + 3*nPoses;
        else
           idend = idend + 3*(nIMUdata+1); 
        end
%         if(bPreInt == 1)            
%             idstart = idend + 1;
%             idend = idend + 3; 
%             % The velocity of the first pose.
%             x(idstart:idend, 1) = (x(4:6,1)-0.5*dtIMU(2)*dtIMU(2)*g0-dp(:,2))/(dtIMU(2));               
%             pidloopmax = nPoses - 1;
%             for(pid=2:pidloopmax)
%               idstart = idend + 1;
%               idend = idend + 3;
%               Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
%               x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dtIMU(pid+1)...
%                   *dtIMU(pid+1)*g0-Ri'*dp(:,(pid+1)))/(dtIMU(pid+1));%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
%             end
%             % The velocity of the last pose.
%             idstart = idend + 1;
%             idend = idend + 3;
%             Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
%             x(idstart:idend) = x((idstart-3):(idend-3), 1)+dtIMU(nPoses)*g0+Ri'*dv(:,nPoses);             
% %             idp1s = (nPoses-1)*6+nPts*3+1;
% %             x((idp1s):(idp1s+2)) = x((idp1s+3):(idp1s+2+3));            
%         else
%             %idend = idend + 3*nIMUdata;%(nPoses-1)*nIMUrate
%             idstart = idend + 1;
%             idend = idend + 3; 
%             % The velocity of the first pose.IMUparking6L
%             if(bDinuka == 1)
%                 imufulldata = [imufulldata(:,1), imufulldata(:,5:7), imufulldata(:,2:4)];% ts, fb, wb
%             end
%             x(idstart:idend, 1) = (x(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imufulldata(ImuTimestamps(1), 2:4))'-bf0))/dt; 
%             pidloopmax = nIMUdata;
%             for(pid=2:pidloopmax)
%               idstart = idend + 1;
%               idend = idend + 3;
%               Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
%               x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dt...
%                   *dt*g0-Ri'*0.5*dt*dt*((imufulldata(ImuTimestamps(1)+pid-1, 2:4))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
%             end
%             % The velocity of the last pose.
%             idstart = idend + 1;
%             idend = idend + 3;
%             Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
%             x(idstart:idend) = x((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imufulldata(ImuTimestamps(1)+nIMUdata, 2:4))'-bf0);            
%         end        
%         x(idstart:idend) = 0; % A better way is to calculate an average speed based on two consecutive keyframes
        %% Intial values of g 
        idstart = idend + 1; idend = idend + 3;
        if(bInitPnF5VoU == 1)
            x(idstart:idend) = g0;%[0,0,-9.8]';
        end
        xg(idstart:idend) = g_true;
    end
    %% Au2c, Tu2c
    idstart = idend + 1; idend = idend + 6;
    if(bInitPnF5VoU == 1)
        x(idstart:idend) = [Au2c;Tu2c];
    end
    xg(idstart:idend) = [Au2c;Tu2c];
    if(bUVonly == 0)
        %% bf, bw
        if(bVarBias == 0)
            idstart = idend + 1; idend = idend + 6;
            if(bInitPnF5VoU == 1)
                x(idstart:idend) = [bf0;bw0];%zeros(6,1);  
            end
            xg(idstart:idend) = [bf_true;bw_true];
        else
            idstart = idend + 1; idend = idend + 6*(nPoses-1);
            if(bInitPnF5VoU == 1)
                x(idstart:idend) = repmat([bf0;bw0],nPoses-1, 1);%zeros(6,1); 
            end
            xg(idstart:idend) = repmat([bf_true;bw_true],nPoses-1, 1);
        end
    end         
    
    xg = xg(1:idend);
    if(bUVonly == 0)
        if(bDinuka == 1)
            load(gtVelfulldir);
            if(bPreInt == 1)
                idstart = (nPoses-1)*6+3*nPts+1;
                idend = (nPoses-1)*6+3*nPts+3*nPoses;
                tv = (true_vel(ImuTimestamps(1:nPoses), 2:end))';
            else
                idstart = nIMUdata*6+3*nPts+1;
                idend = nIMUdata*6+3*nPts+3*(nIMUdata+1);
                tv = (true_vel(ImuTimestamps(1):ImuTimestamps(nPoses), 2:end))';
            end        
            xg(idstart:idend) = tv(:);
        else
            if(bPreInt == 1)
                idend = (nPoses-1)*6+3*nPts;
            else
                idend = nIMUdata*6+3*nPts;
            end
            %[xg] = fnCalV5Kposes(bPreInt, xg, nPoses, dtIMU, idend, dp, dv, g0, bf0, imufulldata);
            [xg,idend] = fnCalV5Kposes(nIMUdata, ImuTimestamps, ...
                nIMUrate, bDinuka, bPreInt, xg, nPoses, dtIMU, idend, ...
                dp, dv, g0, bf0, imufulldata);
        end 
    end
    
    % Display Xgt
    fprintf('Ground Truth Value:\n\t Xg=[');
    fprintf('%f ', xg(1:20));
    fprintf('...]\n'); 

    if(bMalaga == 1)
        z2= xg(4);% Timu(:,4) correspond to Tcam(:,6)==> x-z
    elseif(bDinuka == 1)
        z2 = xg(6);
    end 
    
    if(bInitPnF5VoU == 0)
        x = xg;
        if(bAddInitialNoise == 1)
            x = x + 1e-2*(rand(size(x)) - 0.5);
        end
    else
        x = x(1:(size(xg,1)));
    end
    % Display X0
    fprintf('\nInitial Value:\n\t X0=[');
    fprintf('%f ', x(1:20));
    fprintf('...]\n');    
ie = x-xg;
[me, id] = max(abs(ie))    
    %save('Xgt.mat','x');
    % Show Pose-feature graph
    if(bShowFnP == 1)
        fnShowFeaturesnPoses(xg, nPoses, nPts, nIMUdata, bPreInt, 'Ground Truth Values');
                
        idstart = 1; 
        if(bPreInt == 1)
            idend = 6*(nPoses-1);
        else
            idend = 6*nIMUrate*(nPoses-1);
        end
%         x(idstart:idend) = camposes;
        fnShowFeaturesnPoses(x, nPoses, nPts, nIMUdata, bPreInt, 'Initial Values');
    end

    
    %% Z---the observation vector
    % Firstly allocate a maximal space.
    if(bPreInt == 1)
        Zobs = zeros(2*nPts*nPoses+9*(nPoses-1), 1);
    else
        Zobs = zeros(2*nPts*nPoses+9*(nPoses-1), 1);
    end
    
    %% camera observations (u,v)
    zidend = 0;     
    for(fid=1:nPts)% local id
        nObs = RptFeatureObs(fid, nObsId_FeatureObs);
        for(oid=1:nObs)
            tv = (RptFeatureObs(fid, (oid*3+1):(oid*3+2)))';
            zidstart = zidend + 1; zidend = zidend + 2;%*size(tv, 2);
            Zobs(zidstart:zidend, 1) = tv;%(:);
        end
    end
    nUV = zidend;
%     for(pid=1:nPoses)
%         tv = (obsfeatures{pid}(:,2:3))';
%         zidstart = zidend + 1; zidend = zidend + 2*size(tv, 2);
%         Zobs(zidstart:zidend, 1) = tv(:);
%     end
%     save('obsfeatures.mat', 'obsfeatures');
%     save('RptFeatureObs.mat', 'RptFeatureObs');
    
    
    
    %%%%%%%%%%%%%%%%%
    utid = zidend;
    idr = zidend;    
    if(bUVonly == 0)
        %% Put IMU data into observation vector z: 
        if(bPreInt == 0) % Non-pre-integration: put raw data  
            for pid=2:nPoses
                ndata = size(dataIMU{pid}, 1);
                tv = [dataIMU{pid}(:,2:7), zeros(ndata,3)]';%wi,ai,0
                Zobs((idr+1):(idr+ndata*9)) = tv(:);
                idr = idr+ndata*9;
            end 
            utid = idr;% + (nPoses - 1)*nlenpp;
        else    
            % Add interated IMU observations
            if(bPerfectIMUdlt == 0)
                Zobs((idr+1):(idr+9*(nPoses-1)),1) = reshape([dp(:,2:nPoses);dv(:,2:nPoses);dphi(:,2:nPoses)],[],1);
            else
                %dt = 1;
                Zobs((idr+1):(idr+9*(nPoses-1)),1) = fnCalPerfectIMUdlt_general(x, nPoses, nPts, Jd, dtIMU, bf0, bw0); 
            end
            utid = idr + (nPoses - 1)*9;
        end 

     %% Continue filling in Zobs with psedu observations related to IMU
        if(bAddZg == 1)
            % Add pseudo observation of g        
            Zobs((utid+1):(utid+3)) = g0; 
            utid = utid + 3;
        end
    end
    if(bAddZau2c == 1)
        % Add pseudo observation of Tu2c
        [alpha, beta, gamma] = fnABG5R(Ru2c);
        Zobs((utid+1):(utid+3)) = [alpha;beta;gamma];
        utid = utid + 3;
    end
    if(bAddZtu2c == 1)
        % Add pseudo observation of Tu2c
        Zobs((utid+1):(utid+3)) = Tu2c;
        utid = utid + 3;
    end

    if(bUVonly == 1)% Add A2, T2 as additional observation
%         %%%%%%%
%         pid = 2;
%         Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
%         Rimu = Ru2c'*Rcam*Ru2c;
%         [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
%         Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;                
        Zobs((utid+1)) = z2;%Zobs((utid+1):(utid+6)) = x(1:6);
        utid = utid + 1; %6       
    else
        if(bVarBias == 0)
            if(bAddZbf == 1)
                % Add pseudo observation of bf
                Zobs((utid+1):(utid+3)) = bf0; 
                utid = utid + 3;            
            end
            if(bAddZbw == 1)
                % Add pseudo observation of bf
                Zobs((utid+1):(utid+3)) = bw0; 
                utid = utid + 3;            
            end
        else
            for(pid=2:(nPoses-1))
               Zobs((utid+1):(utid+3)) = 0; %bfi-bfi1 = 0
               utid = utid + 3; 
               Zobs((utid+1):(utid+3)) = 0; %bwi-bwi1 = 0
               utid = utid + 3;               
            end
        end
    end
    Zobs = [Zobs(1:utid)];%(idr+9*(nPoses-1))    
    
    %% Covariance Matrix
%% Save data for nonlin method.
save('initX.mat','x');
%((dataIMU{2}(2, 1) - dataIMU{2}(1, 1)))*size(dataIMU{2},1);
save('consts.mat','nIMUrate','bPreInt','K','Zobs','nPoses','nPts','bf0','bw0','dt','Jd');

%% Covariance matrix
% Original     
%     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3))
    if(bUVonly == 1)
        utid = zidend;
        CovMatrixInv = speye(utid + 6 + 1);
    else
        if(bPreInt == 1)
            utid = nUV+(nPoses-1)*3*3+15;% initialized with possible maximum size.
        else
            utid = nUV + nIMUdata*9 +15;%(nPoses - 1)*nlenpp
        end
        CovMatrixInv = speye(utid);        
        % Initialize the part corresponding to IMU data
        if(bPreInt == 1)
            for pid = 2:nPoses
                covInv = 1e0*inv(Rd{pid}(1:9,1:9)); %2e0 1e0-1 -2 -4  
                CovMatrixInv((nUV+9*(pid-2)+1):(nUV+9*(pid-1)), (nUV+9*(pid-2)+1):(nUV+9*(pid-1))) = covInv;
            end
            utid = nUV+(nPoses-1)*3*3;
        else
            q = inv(diag([sigmaw_cov^2*ones(3,1); sigmaf_cov^2*ones(3,1); sigmatv*sigmatv*ones(3,1)]));%1e-6
            for pid = 1:nIMUdata%((nPoses-1)*nIMUrate
            %CovMatrixInv((idr+1):end,(idr+1):end) =
            % kron(eye((nPoses-1)*nIMUrate),q); % not suitalbe for large scale
            % computation
                CovMatrixInv((nUV+9*(pid-1)+1):(nUV+9*(pid)), (nUV+9*(pid-1)+1):(nUV+9*(pid))) = q;
            end
            utid = nUV+nIMUdata*9;%(nPoses-1)*nlenpp;
        end
    % Initilized additional parts
        if(bAddZg == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
            utid = utid + 3;
        end
    end 
    CovMatrixInv(1:nUV,1:nUV) = 1/(sigmauov_cov*sigmauov_cov)*CovMatrixInv(1:nUV,1:nUV);
    if(bAddZau2c == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(bAddZtu2c == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(bUVonly == 1)% Add A2, T2 as additional observation
        CovMatrixInv((utid+1), (utid+1)) = 1e8;
        %CovMatrixInv((utid+1):(utid+6), (utid+1):(utid+6)) = 1e8*eye(6);
        utid = utid + 1;%6;        
    elseif(bVarBias == 0)
        if(bAddZbf == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);%1e8
            utid = utid + 3;
        end 
        if(bAddZbw == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
            utid = utid + 3;
        end
    else
        for(pid=2:(nPoses-1))
            tv = eye(6);
            tv(1:3, 1:3) = (1/sigmabf)^2 * tv(1:3, 1:3);%(bfi-bfi1)
            tv(4:6, 4:6) = (1/(dtIMU(pid)*sigmabw))^2 * tv(4:6, 4:6);
            CovMatrixInv((utid+1):(utid+6), (utid+1):(utid+6)) = tv;%1e8
            utid = utid + 6; 
        end
    end
    CovMatrixInv = CovMatrixInv(1:utid,1:utid);
    save('CovMatrixInv.mat','CovMatrixInv', '-v7.3');    
    
   
tic
    if(bGNopt == 1)
    %% GN Iterations 
        [x, nReason] = fnVI_BA_general(nUV, K, x, nPoses, nPts, Jd, CovMatrixInv, ...
            nMaxIter, fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ...
            ImuTimestamps, dtIMU, RptFeatureObs, bUVonly, bPreInt, ...
            bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
        nReason
    else    
        [x,nReason,Info] = fnleastsquaresLM(nUV, K, x, nPoses, nPts, Jd, ...
            CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
            bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);        
%         [x,Reason,Info] = fnleastsquaresLM(nUV, K, x, nPoses, nPts, Jd, CovMatrixInv, nMaxIter, ...
%             fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
%             bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);
    end        
toc
    if(bPreInt == 0)
        fprintf('\n###Poses[x(1-%d)], Features[x(%d-%d)], Velocity[x(%d-%d]###\n',...
            nIMUdata*6, nIMUdata*6+1, ...
            nIMUdata*6+nPts*3, nIMUdata*6+nPts*3+1, ...
            nIMUdata*6+nPts*3+3*(nIMUdata+1));
    else
        fprintf('\n###Poses[x(1-%d)], Features[x(%d-%d)], Velocity[x(%d-%d)]###\n',...
            (nPoses-1)*6, (nPoses-1)*6+1, ...
            (nPoses-1)*6+nPts*3, (nPoses-1)*6+nPts*3+1, ...
            (nPoses-1)*6+nPts*3+3*nPoses);
    end     
    % Check against the ground truth
    %xf = x;
    %load('Xgt.mat');%xf

    ef = x - xg;
    [maxe, idx] = max(abs(ef));
    fprintf('Final Error: maxXef=%f, idx=%d\n', maxe, idx);
    
if(nReason < 0)
    return;
end
    Tcam = zeros(3, nPoses);
    Timu = Tcam;
    if(bPreInt == 1)
        for(pid = 2:nPoses)
            %Rcam = Ru2c*Rimu*Ru2c;
            Rimu = fnR5ABG(x(6*(pid-2)+1), x(6*(pid-2)+2), x(6*(pid-2)+3));
            Timu(:, pid) = x((6*(pid-2)+4):(6*(pid-2)+6),1);
            Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
        end
    else
        for(pid = 2:nPoses)
            cid = ImuTimestamps(pid)-ImuTimestamps(1);
            Rimu = fnR5ABG(x(6*(cid-1)+1), x(6*(cid-1)+2), x(6*(cid-1)+3));
            Timu(:, pid) = x((6*(cid-1)+4):(6*(cid-1)+6),1);
            Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
        end        
    end
    load(gtFile);
    figure(); hold on;
    if(bMalaga == 1)
%         plot(GT_P0(:,4),GT_P0(:,6),'-+r');
%         plot(Tcam(1,:),Tcam(3,:),'-*b');
    elseif(bDinuka == 1)
        Tcam = Timu;
        GT_P0 = gtIMUposes(selpids(1:nPoses),2:7);
        GT_P0(:,4:6) = GT_P0(:,4:6) - repmat(GT_P0(1,4:6),nPoses,1);
%         plot(gtIMUposes(selpids(1:nPoses),5)-gtIMUposes(selpids(1),5),...
%             gtIMUposes(selpids(1:nPoses),7)-gtIMUposes(selpids(1),7),'-+r');
%         plot(Timu(1,:),Timu(3,:),'-*b');
    end
%     plot(GT_P0(:,4),GT_P0(:,6),'-+r');
%     plot(Tcam(1,:),Tcam(3,:),'-*b');    
    plot3(GT_P0(:,4),GT_P0(:,5),GT_P0(:,6),'-+r');
    plot3(Tcam(1,:),Tcam(2,:),Tcam(3,:),'-*b');
%     plot3(Timu(1,:),Timu(2,:), Timu(3,:),'-pg');
    view(-45,30);
%     axis equal;
    title('Comparison of Estimated Poses Against the Ground Truth');
    
    figure();
%     if(bMalaga == 1)
        err = Tcam(:,1:nPoses) - GT_P0(1:nPoses, 4:6)';
%     elseif(bDinuka == 1)
%         err = Tcam(:,1:nPoses) - gtIMUposes(selpids(1:nPoses), 5:7)';        
%     end
    ce = complex(err(1,:), err(2,:));
    ce = complex(abs(ce), err(3,:));  
    plot(1:nPoses, abs(ce), 'p');
    title('Pose Translational Error');
%     stop;
%%%%%%%%%%%%    
save('x_Jac.mat', 'x');
%% Show pose-feature graph
if(bShowFnP == 1)
    fnShowFeaturesnPoses_general(x, nPoses, nPts, nIMUdata, bPreInt, 'Final Values');
%     fnShowFeaturesnPoses(xf, nPoses, nPts, nIMUrate, bPreInt, 'Final Values');
end
%% Show uncertainty
if(bShowUncertainty == 1)
    fnCalnShowUncert_general(bUVonly, nUV, RptFeatureObs, ImuTimestamps, ...
        dtIMU, ef, K, x, nPoses, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata, bPreInt, ...
        bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
end
%     fnCalnShowUncert(ef, K, x, nPoses, nPts, dt, Jd, CovMatrixInv, ...
%         nIMUrate, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);
    return;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
	cSift = 1;%0;% % 0--use sift provided by matlab; 1--use another sift realization
	if(cSift == 1)%
		addpath(genpath('sift'));
		featurename = 'SIFT';
	else
		featurename = 'SURF';
	end
	cDataset = 3;%0;%1;%1;% 0--My own dataset; 1--Freigburg dataset; 2-- face; 3--KITTI
	bMyApril = 1;%0;%
	datasetnum = 10;
	datasetname = '2011_09_30_drive_0020_sync';%sprintf('data%d', datasetnum);
    dataDir_root = '/home/youbwang/Documents/KITTI/2011_09_30/2011_09_30_drive_0020_sync/image_00/';
	dltnp = 10;%4;

		if(cDataset == 0)
		    fx = 525.0; %# focal length x
		    fy = 525.0; %# focal length y
		    cx = 319.5; %# optical center x
		    cy = 239.5; %# optical center y
		elseif(cDataset == 2)%%data parameters
		    fx = 575.8157496;%# focal length x
		    fy = 575.8157496;%# focal length y
		    cx = 320.;%# optical center x
		    cy = 240.;%# optical center y        
		elseif(cDataset == 1)%%Freigburg data parameters
		    fx = 575.8157;%# focal length x
		    fy = 575.8157;%# focal length y
		    cx = 320.;%# optical center x
		    cy = 240.;%# optical center y
        elseif(cDataset == 3)
		    fx = 575.8157;%# focal length x
		    fy = 575.8157;%# focal length y
		    cx = 320.;%# optical center x
		    cy = 240.;%# optical center y          
        elseif(cDataset == 4) % KITTI dataset
		    fx = 7.215377e+02;%# focal length x
		    fy = 7.215377e+02;%# focal length y
		    cx = 6.095593e+02;%# optical center x
		    cy = 1.728540e+02;%# optical center y             
		end

	K = [fx 0 cx; 0 fy cy; 0 0 1];
% 
% 	% Initialization
% 	alpha = pi/3;
% 	beta = pi/6;
% 	gamma = pi/4;
% 	T = [1, 2, 3.5]';
% 
% 	% 1. p3d0, alpha, beta, gamma, T => p3d
% 	%p3d0 = 
% 	% 2. p3d0, p3d, K => p2d0, p2d
% 
	% 1. Read images I1, I2
	dmin = 0.01;%0.5;%0.8;%1.;%1.2;%
	dmax = 4;%100;%3.5;%
	Td = 0.035;% 0.02;
	Tnd = Td;
	dfactor = 255./4;
	% Matlab SURF
	fMthreshold = 1000.;%500.;%2000.;%1500.;%10.;%1.;%
	nSlevel = 4;%5;%6;%

	if(isunix) 
	   dataDir = [dataDir_root, 'data', filesep];% cluster %(HP)'/mnt/B0A18DDEEC101C79/uDocs/Research/MyPapers/acra2013/code/SegData/data10/'; %
       nfiles = size(dir(dataDir),1)-3;%2;%5;%
	   %saveDir = ['../datasetinfo/', datsetname, '/unix/'];%/home/yowang/Documents/segDataset/
	else % my pc
	   %dataDir = ['E:\uDocs\Research\MyPapers\acra2013\code\SegData\', datasetname, filesep]; 
	   %saveDir = fullfile('..', 'datasetinfo', datasetname, filesep);
	end
	saveDir = fullfile(dataDir_root, 'datasetinfo', featurename, filesep);

	%% Extract features
	% Batch processing
	if(~exist(saveDir, 'dir'))
		mkdir(saveDir);
	end

	for nPose = 0:dltnp:nfiles
		%sprintf('../segDataset/data%d/', ndg);
        sfrgb1= sprintf('%s%010d.png', dataDir, nPose);
% 		sfrgb1= sprintf('%srgb%04d.png', dataDir, nPose);%ni{ndg}()
% 		sfd1= sprintf('%sdepth%04d.png', dataDir, nPose);%ni{ndg}()
		if(~exist(fullfile(saveDir, sprintf('%010d.mat', nPose))))            
		    %sfrgb2= sprintf('%srgb%04d.png', dir, nPose+1);%ni{ndg}()
		    %sfd2= sprintf('%sdepth%04d.png', dir, nPose+1);%ni{ndg}()

% 		    [fn, ~] = Collectimgfs(sfrgb1, sfd1, nPose, dfactor, K, ...
% 		                                fMthreshold, nSlevel, cSift, cDataset, dmin, dmax);
            [fdes,fp, Ig1] = DetectMonoFeatures(sfrgb1, fMthreshold, nSlevel, cSift);
		    ImgInfo.fdes = fdes;
            ImgInfo.fp = fp;
		    ImgInfo.rgb = sfrgb1;
		    %ImgInfo.depth = sfd1;
		    s = [saveDir sprintf('%010d.mat', nPose)];%['../Output/mbdSLAM/' savename '.mat'];
		    save(s, 'ImgInfo', '-v7.3');%, 'nPose');
		end
	end


	%% Match features
	nPose = 0;
	load(fullfile(saveDir, sprintf('%010d.mat', nPose)));
	I1 = imreadbw(ImgInfo.rgb); % from I2->I1, R/T correspond to camera I1->I2
	ImgInfo1 = ImgInfo;
	nPose = nPose + dltnp;
	load(fullfile(saveDir, sprintf('%010d.mat', nPose)));
	I2 = imreadbw(ImgInfo.rgb);
	ImgInfo2 = ImgInfo;

	if(cSift == 0) 
		matchpairs = matchFeatures(ImgInfo1.fdes(:, 1:end), ImgInfo2.fdes(:, 1:end), 'Prenormalized', false, 'MatchThreshold', 100.0);%);%8
		mnp1 = matchpairs;
	else
		% By passing to integers we greatly enhance the matching speed (we use
		% the scale factor 512 as Lowe's, but it could be greater without
		% overflow)
	%                 descr1=uint8(512*fn(:, 8:end)) ;
	%                 descr2=uint8(512*fk(:, 8:end)) ;
	%                 matchpairs = siftmatch(descr1', descr2');%(fn(:, 8:end))', (fk(:, 8:end))');
		matchpairs = siftmatch((ImgInfo1.fdes(:, 1:end))', (ImgInfo2.fdes(:, 1:end))',10);%, 100);%8  5
		mnp1 = matchpairs';
	end

	% get rid of repeating items
% 	[index_pairs] = GetUniqueMatch(mnp1);
    index_pairs = mnp1;%(1:500, :)index_pairs(1:10, :);
% 	   
% 
	%% Segmentation and visual odometry
% 	% ACRA method 
% 	%bPrev = 0; nGroup = 0; bStatic = 0; bTdAdpt = 0;
% 	%[Yaw, Pitch, Roll, R, T, mnp1] = findInliers(bPrev, nGroup, nPose, I1, I2, bStatic, Td, ...
% 	%                                bTdAdpt, K, cSift, ImgInfo1.featureset, ImgInfo2.featureset, ... index_pairs);%img{nPose}, img{nPose+1}
% 
	% WCICA method
		u10f = ImgInfo1.fp(index_pairs(:, 1), 1);%3
		v10f = ImgInfo1.fp(index_pairs(:, 1), 2);%4
% 		z1 = ImgInfo1.featureset(index_pairs(:, 1), 7);
% 		p10 = ImgInfo1.featureset(index_pairs(:, 1), 5:7);        
		u20f = ImgInfo2.fp(index_pairs(:, 2), 1);
		v20f = ImgInfo2.fp(index_pairs(:, 2), 2);
% 		z2 = ImgInfo2.featureset(index_pairs(:, 2), 7);
% 		p20 = ImgInfo2.featureset(index_pairs(:, 2), 5:7); 
% end
% 
% Prepare for motion segmentation
    y(1:2,:,1) = [u10f, v10f]';
    y(3,:,1) = 1;
    y(1:2,:,2) = [u20f, v20f]'; 
    y(3,:,2) = 1;
    X1 = y(:,:,1); X2 = y(:,:,2);
    addpath(genpath('Ransac'));
    addpath(genpath('Petercorke'));
    bRansac = 1;
    if(bRansac == 1)
        [F,inliers] = ransacF(X1,X2);
        %inliers = inliers(1:20);
        tmpim1 = im2double(I1);
        tmpim2 = im2double(I2);
        if 1
            % Display images and ALL matches
            hold on
            % Prepare lines to plot
            X = [X1(1,:)',X2(1,:)'];
            Y = [X1(2,:)',X2(2,:)'+size(tmpim1,1)];
            clf; imshow([tmpim1;tmpim2]);
            hold on;plot(X1(1,:),X1(2,:),'bo');plot(X2(1,:),X2(2,:)+size(tmpim1,1),'bo');hold on;%line(X',Y',zeros(size(X')),'Color','b');
            % Display onli inliers
            % Prepare lines to plot
            Xi = [X1(1,inliers)',X2(1,inliers)'];
            Yi = [X1(2,inliers)',X2(2,inliers)'+size(tmpim1,1)];
            hold on;line(Xi',Yi',zeros(size(Xi')),'Color','r');

        end;
        hold off;  
    end
    
%%%%%%%%%%%%%%%%%%%%%%%    
	nminp = 8;%3; 
    nModelid = 1;%2;%1;%Fundamental matrix; %
    bAdaptive = 1;
	fthreshold = 0.5; 
    fthreshold1 = 2.5;%3;%3.5;%4;%2.5;%0.5;%10;
	nminclustersize = 8;%3; 
	colors = 'brgymkwcbrgymkwcbrgymkwc';
	bDisplay = 1;
	sinfo = 'MoSeg_2D';%'Motion3DSeg';
    [grouppointid] = MotionSegment(y, nModelid, nminp, fthreshold, fthreshold1, ...
                                            nminclustersize, bAdaptive, colors, bDisplay, sinfo);
     [groupid, R, T1, ninliers] = MotionSegment(I1, y, p10', p20', nminp, fthreshold, ...
                                    nminclustersize, colors, bDisplay, sinfo);     
%     [groupid, R, T1, ninliers] = Motion3DSegment(I1, y, p10', p20', nminp, fthreshold, ...
%                                     nminclustersize, colors, bDisplay, sinfo); 
% 	%T1
%     [alpha1, beta1, gamma1] = fABGfrmR(R);
%     u1f = u10f(groupid);
%     v1f = v10f(groupid);
%     u2f = u20f(groupid);
%     v2f = v20f(groupid);
%     d1f = p10(groupid, 3);
% 	d2f = p20(groupid, 3);
%     p1 = (p10(groupid, :))';   

%% 1. Compose Observation Vector Zobs and nPoses, nPts:
    sFileFullPath = '\data\';
    [Zobs, nPoses, nPts, PosFt_mat] = fnRdFeature5PBAfile(sFileFullPath);

%% 2. Construct State Vector x:
    [alpha1, beta1, gamma1] = fABGfrmR(R);
    x = [alpha1; beta1; gamma1; T1; p1(:)];%x(1), x(2), x(3), x(4), x(5), x(6)
    x = x + randn(size(x))/6;
    Np = size(u1f, 1) * 2;
	covInv = eye(3); covInv(3,3) = 0.01;
    covInv = inv(covInv);
    CovMatrixInv = kron(eye(Np), covInv);
    fprintf('Initial Value:\n\t X0=[');
    fprintf('%f ', x);
    fprintf(']\n');
    
%% 3. VINS-BA
[x] = fnVI_BA(K, x, nPoses, nPts, dt, Jd, CovMatrixInv, nMaxIter, fLowerbound_e, fLowerbound_dx);

fprintf('Final Value:\n\t Xf=[');
fprintf('%f ', x);
fprintf(']\n');

end
