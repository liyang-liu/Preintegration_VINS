%% The main function.
if 0
    clear;
    close all;
    clc;
end

%% Choose to use simulated data or or real data.
bSimData = 0;% p15-30
% Select one of the real datasets
bMalaga = 0;
bDinuka = 1;%p4-15/nonoise:50-80(+50)


%% Configure the conditions of the problem
nAllposes = 60;%50;%25;%240;%13;%4;%20;%5;%240;%60;% 350;%
nPoseOld = 1;
nAddPoses = 1;%10;%2;%4;%5;%50;%30;%1;%
nPoseNew = 4;%3;%50;%10;%10;%5;%15;%30;%80;%120;%30;%25;%20;%18;%13;%3;%5;%10;%4;%50;%9;%2;%170;%6;%120;%
bPreInt = 1;%1;% Use pre-integration method?
    
bInitPnF5VoU = 1;% Use visual odometry or IMU data to initialize x?
bIMUodo = 1;% Use IMU data to initilize x?
bGNopt = 1;% Use Gauss-Newton method?
bShowFnP = 0;% Show poses and features?
bShowUncertainty = 0;% Show uncertainty of the result?
bAddInitialNoise = 0;% x0 + noise or not
kfspan = 10;%1;%2;%5;%10;%20;%15;%50;%30;% Choose keyframes
kfids = 1:kfspan:1200;
nMinObsTimes = 2;%10;%5;%3;%50;%30;%20;%4;%6;%
fMaxDistance = 1e6;%50;%1e2;%5e2;%80;%30;%20;%10;%84;%60;
nPts = 60000;%15000;%6640;%3243;%800;%14172;%1614;%58404;%
bPerfectIMUdlt = 0;
bVarBias = 0;

bUVonly = 0;% If 1, only UVs are used; otherwise, IMU and UVs are fused.

bf0 = zeros(3,1);%[-0.55;0.6;0.61];
bw0 = zeros(3,1);
uvd_cell = [];
dp = zeros(3,nPoseNew);
dv = dp; 
dphi = dp;
dtIMU = [];
x_old = [];
PBAFeature = [];
if(bSimData == 0)
    nPts1 = 60000;
else
    nPts1 = nPts;
end
fId_FeatureObs = 1;
Feature3D = zeros(nPts1, 50);
Feature3D(:, fId_FeatureObs) = 1:nPts1;
nIMUdata_old = 0;
RptFidSet_old = [];
gtIMUposes = [];
selpids = [];
gtVelfulldir = [];
dtIMU = [];
imufulldata = 0;

% if(bPreInt == 1)
%    bShowFnP = 1;
% else
%    bShowFnP = 0; 
% end

if(bDinuka == 1)
    dtIMU = zeros(nPoseNew, 1);
    nIMUrate = 2e2;    
    sigma_w_real = 0.03;%0.03;%0.01;%0.15;% 5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    sigma_f_real = 0.1;%0.2;%0.01;%0.1;%1;%  
    sigma_w_cov = 2*sigma_w_real;%1;%1;%0.03;%0.1;% 0.01;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    sigma_f_cov = 2*sigma_f_real;%1;%1;%0.2;%0.8;% 0.1;%1e-2;%1e-5;%1;%1e0;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%1e0;%2e-3;%
    sigma_uov_real = 0.1;%0.5;%0.2;% 1e-2;%1e-3;%2e-3;%1e-1;%1;%1e-6;%1e-4;%
    sigma_uov_cov = 3*sigma_uov_real;%1;%1;%1e-1;%
   
    sigma_g_cov = 1e-4;
    sigma_au2c_cov = 1e-4;
    sigma_tu2c_cov = 1e-4;    
	sigma_bf_cov=1*2e-2;
    sigma_bw_cov = 1*20*pi/(180*3600); 
    sigma_tv = 1e-4/(nIMUrate);%1;%*nIMUrate*nIMUrate);
    % Configure pseudo observations
    bAddZg = 0; % Add pseudo observation of g or not
    bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
    bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
    bAddZbf = 1; % Add pseudo observation of bias in acceleration or not
    bAddZbw = 1; % Add pseudo observation of bias in rotation or not    
    
    g0 = [0; 0; 9.81]; % g value in the first key frame
    g_true = [0; 0; 9.81];
    bf_true = [0; 0; 0]; % bias for acceleration
    bw_true = [0; 0; 0]; %[0, 0, 0]'; % bias for rotaion velocity     
    % Directories
    datadir = ['..' filesep 'Dinuka' filesep 'dataset_19_10_15' filesep];%dataset_19_10_15
    imgdir = datadir;
    imufulldir = [datadir 'imudata_nonoise.mat'];% small imudata_nonoise['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
    gtVelfulldir = [datadir 'velocity_ground_truth.mat'];
    gtFile = [datadir 'gtIMUposes.mat'];    
end


addpath(genpath('IMU'));
addpath(genpath('MoSeg_2D'));%addpath(genpath('ms3D'));
addpath(genpath('Ransac'));

save('bVarBias.mat','bVarBias');

% Iteration times and bounds for Gauss-Newton
nMaxIter = 30;%1e3;%50;%100;%15;%5;%10;%50;%3;% 20;% 
fLowerbound_e = 1e-10;%1e-6;%1e-5;%1e-1;
fLowerbound_dx = 1e-10;%1e-6;%

%dtIMU = [];
Jd =[];
Rd = [];

% save the configured data
save('bAddZg.mat','bAddZg');    
save('bAddZau2c.mat','bAddZau2c');
save('bAddZtu2c.mat','bAddZtu2c');
save('bAddZbf.mat','bAddZbf');
save('bAddZbw.mat','bAddZbw');
save('bUVonly.mat', 'bUVonly');    

%% The main switch
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

%     % save the configured data
%     save('bAddZg.mat','bAddZg');    
%     save('bAddZau2c.mat','bAddZau2c');
%     save('bAddZtu2c.mat','bAddZtu2c');
%     save('bAddZbf.mat','bAddZbf');
%     save('bAddZbw.mat','bAddZbw');  
%     save('bUVonly.mat', 'bUVonly');
    
    %%%%%%%%%%%%%%
    % camera observations  

    FeatureObs = zeros(nPts, 500);%[fid, nObs, [pid, ui,vi]]
    fId_FeatureObs = 1; nObsId_FeatureObs = 2;
    FeatureObs(:, fId_FeatureObs) = 1:nPts;

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
   
%     tfids = fidset{1}; % Re-identify the points
%     obsfeatures{1} = [(1:nPts)', ... % obsfeatures{1}(tfids(bestmodel),1)
%         obsfeatures{1}(tfids(bestmodel),2:3)];
%     tfids = fidset{2};
%     obsfeatures{2} = [(1:nPts)', ... % obsfeatures{1}(tfids(bestmodel),1)
%         obsfeatures{2}(tfids(bestmodel),2:3)];    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    nIMUdata = 0;    
    ImuTimestamps = zeros(nAllposes, 1);    
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
            rng('default');
            imufulldata(:,2:4) = imufulldata(:,2:4) + fnGenGaussNoise(nt, 3, sigma_w_real);
            imufulldata(:,5:7) = imufulldata(:,5:7) + fnGenGaussNoise(nt, 3, sigma_f_real);
        end
        utid = 1; %ctid = 1;
        for(ctid = 1:nAllposes)
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
        dataIMU = {};     
        for pid=2:nAllposes
            uid0 = ImuTimestamps(pid-1);
            uid1 = ImuTimestamps(pid)-1;
%             nIMUdata = nIMUdata + ImuTimestamps(pid) - ImuTimestamps(pid-1);
            dtIMU(pid) = imufulldata(ImuTimestamps(pid)) - imufulldata(ImuTimestamps(pid-1));
            if(bMalaga == 1)
                dataIMU{pid} = [IMUparking6L(uid0:uid1, 1), IMUparking6L(uid0:uid1, 7), IMUparking6L(uid0:uid1, 6),...
                    IMUparking6L(uid0:uid1, 5), IMUparking6L(uid0:uid1, 2:4)];
%                 dataIMU{pid} = [IMUparking6L(uid0:uid1, 1), IMUparking6L(uid0:uid1, 5:7), ...
%                     IMUparking6L(uid0:uid1, 2:4)];
            elseif(bDinuka == 1)
                dataIMU{pid} = imufulldata(uid0:uid1, :);                
            end
        end 
        if(bUVonly == 0)%bPreInt == 1)%
        % Generate pre-integration observations based on IMU raw data.          
            dp = zeros(3,nAllposes);
            dv = dp; 
            dphi = dp;    
            for pid=2:nAllposes%1e-2,1e-3+3*sigmaf+3*sigmaw
                [dp(:,pid), dv(:,pid), dphi(:,pid), Jd{pid}, Rd{pid}] = ...
                    fnDeltObsAccu(bf0, bw0, dataIMU{pid}, sigma_w_cov, sigma_f_cov); 
            end
        end
                
        save('ImuTimestamps.mat', 'ImuTimestamps');
        save('dtIMU.mat', 'dtIMU');
    end
    
%% Incrementally construct x, z and cov, then solve them trhough iterations 
    while(nPoseOld < nAllposes)
        nPoses = nPoseNew - nPoseOld;
        if(nPoseOld == 1)
            pid = 1;
            [FeatureObs] = fnCollectfObs5Imgs(bMalaga, bDinuka, ...
                        kfids, pid, imgdir, sigma_uov_real, FeatureObs, ...
                        nObsId_FeatureObs);                              
        end 
        
        for(pid=(nPoseOld+1):nPoseNew)
               [FeatureObs] = fnCollectfObs5Imgs(bMalaga, bDinuka, ...
                    kfids, pid, imgdir, sigma_uov_real, FeatureObs, ...
                    nObsId_FeatureObs); 
%             if(bMalaga == 1)
%                 load(sprintf('%sImage%d.mat', imgdir, pid));
%                 obsfeatures{pid} = Image(2:end, 1:3);
%                 fidset = Image(2:end, 1);
%             elseif(bDinuka == 1)
%                 imid = kfids(pid);%1+(pid-1)*kfspan);
%                 load(sprintf('%simage_%d.mat', imgdir, imid));
%                 obsfeatures{pid} = features;
%                 fidset = features(:, 1);            
%             end
% 
%             FeatureObs(fidset, nObsId_FeatureObs) = ...
%                 FeatureObs(fidset, nObsId_FeatureObs) + 1;
%             for(fid=1:size(fidset,1))
%                 nObs = FeatureObs(fidset(fid), nObsId_FeatureObs);
%                 FeatureObs(fidset(fid), 3*nObs) = pid;
%                 if(bMalaga == 1)
%                     FeatureObs(fidset(fid), (3*nObs+1):(3*nObs+2)) = Image(fid+1, 2:3);
%                 elseif(bDinuka == 1)
%                     FeatureObs(fidset(fid), (3*nObs+1):(3*nObs+2)) = features(fid, 2:3) + ...
%                         fnGenGaussNoise(1, 2, sigma_uov_real);                
%                 end
%             end
        end
        
        if(bMalaga == 1)
            load([datadir 'PBAFeature.mat']);
            RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoseNew, nMinObsTimes));
            RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
        % RptFidSet(586) = [];    
            RptFeatureObs = FeatureObs(RptFidSet, :);
        elseif(bDinuka == 1)
            %load([datadir 'feature_pos.mat']);
            RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoseNew, nMinObsTimes));
            %RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
            RptFeatureObs = FeatureObs(RptFidSet, :);        
        end
        nPts = size(RptFidSet, 1);
               
    
        nIMUdata = ImuTimestamps(nPoseNew) - ImuTimestamps(1);
        %% X---the state vector
        if(bUVonly == 1)
            x = zeros(6*(nPoseNew-1)+3*nPts+3*2,1);
        else
            if(bPreInt == 0)
                x = zeros((nIMUdata)*6+nPts*3+3*(nIMUdata+1)+15+6,1);%???(nPoses-1)*nIMUrate (nPoses-1)*nIMUrate+1 one additional 6 for convenience
            else
                x = zeros((nPoseNew-1)*6+nPts*3+3*nPoseNew+3+6+6, 1); 
            end  
        end   

        xg = x;

        %% Compose the ground truth value 1
        [~, fscaleGT] = fnGetXgroundtruth_general(xg, bUVonly, bVarBias, bMalaga, bDinuka, ...
            datadir, nPoseNew, ImuTimestamps, gtIMUposes, selpids, bPreInt, ...
            nPts, PBAFeature, RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, dp, dv, Au2c, ...
            Ru2c, Tu2c, gtVelfulldir, g_true, bf_true, bw_true);     
    
%% Compose Initial value of X from odometry 
        if(bInitPnF5VoU == 1)
            [x, RptFidSet, RptFeatureObs, nPts] = fnCompX5odometry(nPoseOld, nPoseNew, nPoses, nPts, x_old, bIMUodo, ...
                ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
                RptFidSet_old, dtIMU, g0, dp, dv, dphi, K, bMalaga, RptFeatureObs, ...
                Tu2c, Au2c, Ru2c, fscaleGT, bSimData, kfids, nIMUrate, ...
                bDinuka, bPreInt, bVarBias, x, bf0, bw0, imufulldata, bUVonly);
            
        end
        
        %% Compose the ground truth value 2
        [xg, fscaleGT] = fnGetXgroundtruth_general(xg, bUVonly, bVarBias, bMalaga, bDinuka, ...
            datadir, nPoseNew, ImuTimestamps, gtIMUposes, selpids, bPreInt, ...
            nPts, PBAFeature, RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, dp, dv, Au2c, ...
            Ru2c, Tu2c, gtVelfulldir, g_true, bf_true, bw_true);
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
%         else
%             x = x(1:(size(xg,1)));
        end
        % Display X0
        fprintf('\nInitial Value:\n\t X0=[');
        fprintf('%f ', x(1:20));
        fprintf('...]\n');    
    ie = x-xg;
    [me, id] = max(abs(ie))   
    if(nPoseNew == 25)
        aa = 1;
    end
        %save('Xgt.mat','x');
        % Show Pose-feature graph
        if((bShowFnP == 1) && ((nPoseOld == 1) || (nPoseNew == nAllposes)))
            fnShowFeaturesnPoses(xg, nPoseNew, nPts, nIMUdata, bPreInt, 'Ground Truth Values');
            fnShowFeaturesnPoses(x, nPoseNew, nPts, nIMUdata, bPreInt, 'Initial Values');
        end

        %% Z---the observation vector
        % Firstly allocate a maximal space.
        if(bPreInt == 1)
            Zobs = zeros(2*nPts*nPoseNew+9*(nPoseNew-1), 1);
        else
            Zobs = zeros(2*nPts*nPoseNew+9*(nPoseNew-1), 1);
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
                for pid=2:nPoseNew
                    ndata = size(dataIMU{pid}, 1);
                    tv = [dataIMU{pid}(:,2:7), zeros(ndata,3)]';%wi,ai,0
                    Zobs((idr+1):(idr+ndata*9)) = tv(:);
                    idr = idr+ndata*9;
                end 
                utid = idr;% + (nPoses - 1)*nlenpp;
            else    
                % Add interated IMU observations
                if(bPerfectIMUdlt == 0)
                    Zobs((idr+1):(idr+9*(nPoseNew-1)),1) = reshape([dp(:,2:nPoseNew);dv(:,2:nPoseNew);dphi(:,2:nPoseNew)],[],1);
                else
                    %dt = 1;
                    Zobs((idr+1):(idr+9*(nPoseNew-1)),1) = fnCalPerfectIMUdlt_general(x, nPoseNew, nPts, Jd, dtIMU, bf0, bw0); 
                end
                utid = idr + (nPoseNew - 1)*9;
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
                for(pid=2:(nPoseNew-1))
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
    save('consts.mat','nIMUrate','bPreInt','K','Zobs','nPoseNew','nPts','bf0','bw0','dt','Jd');
    save('RptFeatureObs.mat', 'RptFeatureObs'); 
    %% Covariance matrix
    % Original     
    %     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3))
        if(bUVonly == 1)
            utid = zidend;
            CovMatrixInv = speye(utid + 6 + 1);
        else
            if(bPreInt == 1)
                utid = nUV+(nPoseNew-1)*3*3+15;% initialized with possible maximum size.
            else
                utid = nUV + nIMUdata*9 +15;%(nPoses - 1)*nlenpp
            end
            CovMatrixInv = speye(utid);        
            % Initialize the part corresponding to IMU data
            if(bPreInt == 1)
                for pid = 2:nPoseNew
                    covInv = 1e0*inv(Rd{pid}(1:9,1:9)); %2e0 1e0-1 -2 -4  
                    CovMatrixInv((nUV+9*(pid-2)+1):(nUV+9*(pid-1)), (nUV+9*(pid-2)+1):(nUV+9*(pid-1))) = covInv;
                end
                utid = nUV+(nPoseNew-1)*3*3;
            else
                q = inv(diag([sigma_w_cov^2*ones(3,1); sigma_f_cov^2*ones(3,1); sigma_tv*sigma_tv*ones(3,1)]));%1e-6
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
                CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                   1/(sigma_g_cov*sigma_g_cov)*eye(3);
                utid = utid + 3;
            end
        end 
        CovMatrixInv(1:nUV,1:nUV) = 1/(sigma_uov_cov*sigma_uov_cov)*CovMatrixInv(1:nUV,1:nUV);
        if(bAddZau2c == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                1/(sigma_au2c_cov*sigma_au2c_cov)*eye(3);
            utid = utid + 3;
        end
        if(bAddZtu2c == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                1/(sigma_tu2c_cov*sigma_tu2c_cov)*eye(3);
            utid = utid + 3;
        end
        if(bUVonly == 1)% Add A2, T2 as additional observation
            CovMatrixInv((utid+1), (utid+1)) = 1e8;
            %CovMatrixInv((utid+1):(utid+6), (utid+1):(utid+6)) = 1e8*eye(6);
            utid = utid + 1;%6;        
        elseif(bVarBias == 0)
            if(bAddZbf == 1)
                CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                    1/(sigma_bf_cov*sigma_bf_cov)*eye(3);%1e8
                utid = utid + 3;
            end 
            if(bAddZbw == 1)
                CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                    1/(sigma_bw_cov*sigma_bw_cov)*eye(3);
                utid = utid + 3;
            end
        else
            for(pid=2:(nPoseNew-1))
                tv = eye(6);
                tv(1:3, 1:3) = (1/sigma_bf_cov)^2 * tv(1:3, 1:3);%(bfi-bfi1)
                tv(4:6, 4:6) = (1/(dtIMU(pid)*sigma_bw_cov))^2 * tv(4:6, 4:6);
                CovMatrixInv((utid+1):(utid+6), (utid+1):(utid+6)) = tv;%1e8
                utid = utid + 6; 
            end
        end
        CovMatrixInv = CovMatrixInv(1:utid,1:utid);
        save('CovMatrixInv.mat','CovMatrixInv', '-v7.3');    


    tic
        if(bGNopt == 1)
        %% GN Iterations 
            [x, nReason] = fnVI_BA_general(nUV, K, x, nPoseNew, nPts, Jd, CovMatrixInv, ...
                nMaxIter, fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ...
                ImuTimestamps, dtIMU, RptFeatureObs, bUVonly, bPreInt, ...
                bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
            nReason
        else    
            [x,nReason,Info] = fnleastsquaresLM(nUV, K, x, nPoseNew, nPts, Jd, ...
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
                (nPoseNew-1)*6, (nPoseNew-1)*6+1, ...
                (nPoseNew-1)*6+nPts*3, (nPoseNew-1)*6+nPts*3+1, ...
                (nPoseNew-1)*6+nPts*3+3*nPoseNew);
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
    
    if((nPoseOld == 1) || (nPoseNew == nAllposes))
        Tcam = zeros(3, nPoseNew);
        Timu = Tcam;
        if(bPreInt == 1)
            for(pid = 2:nPoseNew)
                %Rcam = Ru2c*Rimu*Ru2c;
                Rimu = fnR5ABG(x(6*(pid-2)+1), x(6*(pid-2)+2), x(6*(pid-2)+3));
                Timu(:, pid) = x((6*(pid-2)+4):(6*(pid-2)+6),1);
                Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
            end
        else
            for(pid = 2:nPoseNew)
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
            GT_P0 = gtIMUposes(selpids(1:nPoseNew),2:7);
            GT_P0(:,4:6) = GT_P0(:,4:6) - repmat(GT_P0(1,4:6),nPoseNew,1);
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
            err = Tcam(:,1:nPoseNew) - GT_P0(1:nPoseNew, 4:6)';
    %     elseif(bDinuka == 1)
    %         err = Tcam(:,1:nPoses) - gtIMUposes(selpids(1:nPoses), 5:7)';        
    %     end
        ce = complex(err(1,:), err(2,:));
        ce = complex(abs(ce), err(3,:));  
        plot(1:nPoseNew, abs(ce), 'p');
        title('Pose Translational Error');
    %     stop;
    %%%%%%%%%%%%    
        save('x_Jac.mat', 'x');
        %% Show pose-feature graph
        if((bShowFnP == 1) && (nPoseNew == nAllposes))
            fnShowFeaturesnPoses_general(x, nPoseNew, nPts, nIMUdata, bPreInt, 'Final Values');
        %     fnShowFeaturesnPoses(xf, nPoses, nPts, nIMUrate, bPreInt, 'Final Values');
%         end
        %% Show uncertainty
%         if((bShowUncertainty == 1) && (nPoseNew == nAllposes))%
            fnCalnShowUncert_general(bUVonly, nUV, RptFeatureObs, ImuTimestamps, ...
                dtIMU, ef, K, x, nPoseNew, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata, bPreInt, ...
                bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
        end
    end
    
    x_old = x;
    nPoseOld = nPoseNew;
    nPoseNew = nPoseNew + nAddPoses;
    if(nPoseNew > nAllposes)
        nPoseNew = nAllposes;
    end
    RptFidSet_old = RptFidSet;
    nIMUdata_old = nIMUdata;
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


