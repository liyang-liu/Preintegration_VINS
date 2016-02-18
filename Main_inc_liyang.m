%% The main function.
if 0
    close all;    
    clear all;
    clear class;
    clc;
end

run InertialDelta_config_script
global InertialDelta_options

nPoseOld = InertialDelta_options.nPoseOld
nAddPoses = InertialDelta_options.nAddPoses
nPoseNew = InertialDelta_options.nPoseNew
kfids = 1:InertialDelta_options.kfspan:1200;
nPts = InertialDelta_options.nPts
nAllposes = InertialDelta_options.nAllposes

bf0 = zeros(3,1);%[-0.55;0.6;0.61];
bw0 = zeros(3,1);
uvd_cell = [];
dp = zeros(3,nPoseNew);
dv = dp; 
dphi = dp;
dtIMU = [];
x_old = [];
PBAFeature = [];
if(InertialDelta_options.bSimData == 0)
    nPts1 = 60000;
else
    nPts1 = nPts;
end
fId_FeatureObs = 1;
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

if(InertialDelta_options.bDinuka == 1)
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

%save('bVarBias.mat','InertialDelta_options.bVarBias');

% Iteration times and bounds for Gauss-Newton
nMaxIter = 30;%1e3;%50;%100;%15;%5;%10;%50;%3;% 20;% 
fLowerbound_e = 1e-10;%1e-6;%1e-5;%1e-1;
fLowerbound_dx = 1e-10;%1e-6;%

%dtIMU = [];
Jd =[];
Rd = [];

% save the configured data
%save('bAddZg.mat','bAddZg');    
%save('bAddZau2c.mat','bAddZau2c');
%save('bAddZtu2c.mat','bAddZtu2c');
%save('bAddZbf.mat','bAddZbf');
%save('bAddZbw.mat','bAddZbw');
%save('bUVonly.mat', 'bUVonly');    

%% The main switch
    if(InertialDelta_options.bMalaga == 1)
        K = [923.5295, 0, 507.2222; 0, 922.2418, 383.5822; 0, 0, 1];% Left
        % [911.3657, 0, 519.3951; 0, 909.3910, 409.0285; 0, 0, 1]; &Right
    elseif(InertialDelta_options.bDinuka == 1)
       load([datadir 'cam.mat']); 
       K = cam.K;
    end   
    
    
    if(InertialDelta_options.bMalaga == 1)
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
    
    elseif(InertialDelta_options.bDinuka == 1)
        load([datadir 'gtIMUposes.mat']);% ts, Aimu, Timu
        nt = size(gtIMUposes, 1); 
        selpids = 9:(10*InertialDelta_options.kfspan):nt;
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
    %%%%%%%%%%%%%%
    
    % Define data structure for camera observation
    Observation_Def = struct( ...
        'pid', [], ...
        'uv',  zeros(1,2) ...
        );
    FeatureInfo_Def = struct( ...
        'fid',  [], ...
        'nObs', 0, ...
        'obsv', Observation_Def ... % array of observations, size will grow
        );
    FeatureObs = repmat( FeatureInfo_Def, nPts, 1);
    %fids = mat2cell( 1:nPts, 1, ones(1, nPts)); 
    fids = num2cell( 1:nPts );
    [FeatureObs(:).fid] = fids{:};        
    
    % Define data structure for Feature3D
    TriangulateInfo_Def = struct( ...
        'pid1', [], ...
        'pid2', [], ...
        'p3D',  zeros(3,1) ...  % 3D co-ordinates (x, y, z)
        );
    Feature3DInfo_Def = struct( ...
        'fid',  [], ...
        'numTriangs', 0, ...
        'triangs', TriangulateInfo_Def ... % is array of triangulates, size will grow
        );
    Feature3D = repmat( Feature3DInfo_Def, nPts, 1);
    fids = num2cell( 1:nPts );
    [Feature3D(:).fid] = fids{:};        

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    nIMUdata = 0;    
    ImuTimestamps = zeros(nAllposes, 1);    
    if(InertialDelta_options.bUVonly == 0)
        % IMU observations
        if(InertialDelta_options.bMalaga == 1)
            load([imgdir 'KeyframeTimestamps.mat']);
        elseif(InertialDelta_options.bDinuka == 1)
            load([imgdir 'image_time_stamp.mat']);
            KeyframeTimestamps = vis_time(kfids);
        end

        load(imufulldir);
        
        if(InertialDelta_options.bMalaga == 1)
            imufulldata = IMUparking6L;
        elseif(InertialDelta_options.bDinuka == 1)
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

            dtIMU(pid) = imufulldata(ImuTimestamps(pid)) - imufulldata(ImuTimestamps(pid-1));
            if(InertialDelta_options.bMalaga == 1)
                dataIMU{pid} = [IMUparking6L(uid0:uid1, 1), IMUparking6L(uid0:uid1, 7), IMUparking6L(uid0:uid1, 6),...
                    IMUparking6L(uid0:uid1, 5), IMUparking6L(uid0:uid1, 2:4)];
            elseif(InertialDelta_options.bDinuka == 1)
                dataIMU{pid} = imufulldata(uid0:uid1, :);                
            end
        end 
        if(InertialDelta_options.bUVonly == 0)%InertialDelta_options.bPreInt == 1)%
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
        fprintf('\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n');
        fprintf('n%%            PoseOld - %d, nPoseNew - %d                  %%\n', nPoseOld, nPoseNew);
        fprintf('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n');
        
        nPoses = nPoseNew - nPoseOld;
        if(nPoseOld == 1)
            pid = 1;
            [FeatureObs] = fnCollectfObs5Imgs( ...
                                kfids, pid, imgdir, sigma_uov_real, FeatureObs );                              
        end 
        
        for(pid=(nPoseOld+1):nPoseNew)
               [FeatureObs] = fnCollectfObs5Imgs( ...
                                kfids, pid, imgdir, sigma_uov_real, FeatureObs ); 
        end
        
        if(InertialDelta_options.bMalaga == 1)
            load([datadir 'PBAFeature.mat']);
            %RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoseNew, nMinObsTimes));
            %RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
            %RptFeatureObs = FeatureObs(RptFidSet, :);
            RptFidSet = find( [FeatureObs(:).nObs] > 1);
            RptFidSet = RptFidSet(:);
            RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
            RptFeatureObs = FeatureObs(RptFidSet);
        elseif(InertialDelta_options.bDinuka == 1)
            RptFidSet = find( [FeatureObs(:).nObs] >= min(nPoseNew, InertialDelta_options.nMinObsTimes));
            RptFidSet = RptFidSet(:);
            RptFeatureObs = FeatureObs(RptFidSet);
        end
        
        nPts = size(RptFidSet, 1);
                   
        nIMUdata = ImuTimestamps(nPoseNew) - ImuTimestamps(1);
        
        %% X---the state vector
        X_obj = InertialDelta_InitX( nPts, nPoseNew, nIMUdata );
        Xg_obj = X_obj;

        %% Compose the ground truth value 1
        [~, fscaleGT] = fnGetXgroundtruth_general(Xg_obj, ...
            datadir, nPoseNew, ImuTimestamps, gtIMUposes, selpids, ...
            nPts, PBAFeature, RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, dp, dv, Au2c, ...
            Ru2c, Tu2c, gtVelfulldir, g_true, bf_true, bw_true);     
    
%% Compose Initial value of X from odometry 
        if(InertialDelta_options.bInitPnF5VoU == 1)
            [X_obj, RptFidSet, RptFeatureObs, nPts] = fnCompX5odometry(nPoseOld, nPoseNew, nPoses, nPts, x_old, ...
                ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
                RptFidSet_old, dtIMU, g0, dp, dv, dphi, K, RptFeatureObs, ...
                Tu2c, Au2c, Ru2c, fscaleGT, kfids, nIMUrate, ...
                X_obj, bf0, bw0, imufulldata);
            
        end
        
        %% Compose the ground truth value 2
        [Xg_obj, fscaleGT] = fnGetXgroundtruth_general(Xg_obj, ...
            datadir, nPoseNew, ImuTimestamps, gtIMUposes, selpids, ...
            nPts, PBAFeature, RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, dp, dv, Au2c, ...
            Ru2c, Tu2c, gtVelfulldir, g_true, bf_true, bw_true);
        
            
        % Display Xgt
        fprintf('Ground Truth Value:\n\t Xg_obj=[\nAng: ');
        %fprintf('%f ', Xg_obj(1:20));
        fprintf('%f ', [Xg_obj.pose(1).ang.val; Xg_obj.pose(2).ang.val; Xg_obj.pose(3).ang.val]);
        fprintf('\nTrans: ');
        fprintf('%f ', [Xg_obj.pose(1).trans.val; Xg_obj.pose(2).trans.val; Xg_obj.pose(3).trans.val]);
        fprintf('\nFeature_1: ');
        fprintf('%f ', Xg_obj.feature(1).xyz');
        fprintf('...]\n');  
        
        if(InertialDelta_options.bMalaga == 1)
            z2= Xg_obj(4);% Timu(:,4) correspond to Tcam(:,6)==> x-z
        elseif(InertialDelta_options.bDinuka == 1)
            %z2 = Xg_obj(6);
            z2 = Xg_obj.pose(2).trans.val(3);
        end         
        
        if(InertialDelta_options.bInitPnF5VoU == 0)
            x = Xg_obj;
            if(bAddInitialNoise == 1)
                x = x + 1e-2*(rand(size(x)) - 0.5);
            end
%         else
%             x = x(1:(size(xg,1)));
        end
        % Display X0
        fprintf('\nInitial Value:\n\t X0=[\nAng: ');
        %fprintf('%f ', x(1:20));
        fprintf('%f ', [X_obj.pose(1).ang.val; X_obj.pose(2).ang.val; X_obj.pose(3).ang.val]);
        fprintf('\nTrans: ');
        fprintf('%f ', [X_obj.pose(1).trans.val; X_obj.pose(2).trans.val; X_obj.pose(3).trans.val]);
        fprintf('\nFeature_1: ');
        fprintf('%f ', X_obj.feature(1).xyz');
        fprintf('...]\n');    
    %ie = x-xg;
    ie = XObject2Vector( XObjectDiff(X_obj, Xg_obj) );
    [me, id] = max(abs(ie))   
    if(nPoseNew == 25)
        aa = 1;
    end
        %save('Xgt.mat','x');
        % Show Pose-feature graph
        if((InertialDelta_options.bShowFnP == 1) && ((nPoseOld == 1) || (nPoseNew == nAllposes)))
            fnShowFeaturesnPoses(Xg_obj, nPoseNew, nPts, nIMUdata, 'Ground Truth Values');
            fnShowFeaturesnPoses(X_obj, nPoseNew, nPts, nIMUdata, 'Initial Values');
        end

        %% Z---the observation vector
        Zobs = InertialDelta_InitZ( nPoseNew, nPts );
        
        %% camera observations (u,v)
        %zidend = 0; 
        zid = 0;
        zrow = 0;
        for(fid=1:nPts)% local id
            %nObs = RptFeatureObs(fid, nObsId_FeatureObs);
            nObs = RptFeatureObs(fid).nObs;
            for(oid=1:nObs)
                %tv = (RptFeatureObs(fid, (oid*3+1):(oid*3+2)))';
                tv = RptFeatureObs(fid).obsv(oid).uv';
                %zidstart = zidend + 1; zidend = zidend + 2;%*size(tv, 2);
                %Zobs(zidstart:zidend, 1) = tv;%(:);
                zid = zid + 1;
                Zobs.fObs(zid).uv = tv(:);
                Zobs.fObs(zid).row = (1:2) + zrow;
                zrow = zrow + 2;
            end
        end
        Zobs.fObs(zid+1:end) = [];
        nUV = zid*2; %size(Zobs.fObs(1).uv, 1)

        %%%%%%%%%%%%%%%%%
        %utid = zidend;
        %idr = zidend;    
        if(InertialDelta_options.bUVonly == 0)
            %% Put IMU data into observation vector z: 
            if(InertialDelta_options.bPreInt == 0) % Non-pre-integration: put raw data  
                for pid=2:nPoseNew
                    ndata = size(dataIMU{pid}, 1);
                    tv = [dataIMU{pid}(:,2:7), zeros(ndata,3)]';%wi,ai,0
                    Zobs((idr+1):(idr+ndata*9)) = tv(:);
                    idr = idr+ndata*9;
                end 
                utid = idr;% + (nPoses - 1)*nlenpp;
            else    
                % Add interated IMU observations
                if(InertialDelta_options.bPerfectIMUdlt == 0)
                    %Zobs((idr+1):(idr+9*(nPoseNew-1)),1) = reshape([dp(:,2:nPoseNew);dv(:,2:nPoseNew);dphi(:,2:nPoseNew)],[],1);
                    for p = 2 : nPoseNew
                        Zobs.intlDelta(p-1).deltaP.val = dp(:, p);
                        Zobs.intlDelta(p-1).deltaP.row = (1:3) + zrow;
                        zrow = zrow + 3;
                        
                        Zobs.intlDelta(p-1).deltaV.val = dv(:, p);
                        Zobs.intlDelta(p-1).deltaV.row = (1:3) + zrow;
                        zrow = zrow + 3;
                        
                        Zobs.intlDelta(p-1).deltaPhi.val = dphi(:, p);
                        Zobs.intlDelta(p-1).deltaPhi.row = (1:3) + zrow;
                        zrow = zrow + 3;
                    end
                else
                    %dt = 1;
                    %Zobs((idr+1):(idr+9*(nPoseNew-1)),1) = fnCalPerfectIMUdlt_general(x, nPoseNew, nPts, Jd, dtIMU, bf0, bw0); 
                    Zobs.intlDelta = fnCalPerfectIMUdlt_general(X_obj, nPoseNew, nPts, Jd, dtIMU, bf0, bw0); 
                end
                %utid = idr + (nPoseNew - 1)*9;
            end 

         %% Continue filling in Zobs with psedu observations related to IMU
            if(InertialDelta_options.bAddZg == 1)
                % Add pseudo observation of g        
                %Zobs((utid+1):(utid+3)) = g0; 
                %utid = utid + 3;
                Zobs.g.val = g0;
                Zobs.g.row = (1:3) + zrow;
                zrow = zrow + 3;
                
            end
        end
        if(InertialDelta_options.bAddZau2c == 1)
            % Add pseudo observation of Tu2c
            [alpha, beta, gamma] = fnABG5R(Ru2c);
            %Zobs((utid+1):(utid+3)) = [alpha;beta;gamma];
            %utid = utid + 3;
            Zobs.Au2c.val = [alpha; beta; gamma];
            Zobs.Au2c.row = (1:3) + zrow;
            zrow = zrow + 3;
        end
        if(InertialDelta_options.bAddZtu2c == 1)
            % Add pseudo observation of Tu2c
            %Zobs((utid+1):(utid+3)) = Tu2c;
            %utid = utid + 3;
            Zobs.Tu2c.val = Tu2c;
            Zobs.Tu2c.row = (1:3) + zrow;
            zrow = zrow + 3;
        end

        if(InertialDelta_options.bUVonly == 1)% Add A2, T2 as additional observation
            Zobs((utid+1)) = z2;%Zobs((utid+1):(utid+6)) = x(1:6);
            utid = utid + 1; %6       
        else
            if(InertialDelta_options.bVarBias == 0)
                if(InertialDelta_options.bAddZbf == 1)
                    % Add pseudo observation of bf
                    %Zobs((utid+1):(utid+3)) = bf0; 
                    %utid = utid + 3;            
                    Zobs.Bf.val = bf0;
                    Zobs.Bf.row = (1:3) + zrow;
                    zrow = zrow + 3;
                end
                
                if(InertialDelta_options.bAddZbw == 1)
                    % Add pseudo observation of bf
                    %Zobs((utid+1):(utid+3)) = bw0; 
                    %utid = utid + 3;            
                    Zobs.Bw.val = bw0;
                    Zobs.Bw.row = (1:3) + zrow;
                    zrow = zrow + 3;
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
        %Zobs = [Zobs(1:utid)];%(idr+9*(nPoses-1))    

        %% Covariance Matrix
    %% Save data for nonlin method.
    save('initX.mat','X_obj');
    %((dataIMU{2}(2, 1) - dataIMU{2}(1, 1)))*size(dataIMU{2},1);
    save('consts.mat','nIMUrate','K','Zobs','nPoseNew','nPts','bf0','bw0','dt','Jd');
    save('Zobs.mat', 'Zobs'); 
    save('RptFeatureObs.mat', 'RptFeatureObs'); 
    %% Covariance matrix
    % Original     
    %     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3))
        if(InertialDelta_options.bUVonly == 1)
            utid = zidend;
            CovMatrixInv = speye(utid + 6 + 1);
        else
            if(InertialDelta_options.bPreInt == 1)
                utid = nUV+(nPoseNew-1)*3*3+15;% initialized with possible maximum size.
            else
                utid = nUV + nIMUdata*9 +15;%(nPoses - 1)*nlenpp
            end
            CovMatrixInv = speye(utid);        
            % Initialize the part corresponding to IMU data
            if(InertialDelta_options.bPreInt == 1)
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
            if(InertialDelta_options.bAddZg == 1)
                CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                   1/(sigma_g_cov*sigma_g_cov)*eye(3);
                utid = utid + 3;
            end
        end 
        CovMatrixInv(1:nUV,1:nUV) = 1/(sigma_uov_cov*sigma_uov_cov)*CovMatrixInv(1:nUV,1:nUV);
        if(InertialDelta_options.bAddZau2c == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                1/(sigma_au2c_cov*sigma_au2c_cov)*eye(3);
            utid = utid + 3;
        end
        if(InertialDelta_options.bAddZtu2c == 1)
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                1/(sigma_tu2c_cov*sigma_tu2c_cov)*eye(3);
            utid = utid + 3;
        end
        if(InertialDelta_options.bUVonly == 1)% Add A2, T2 as additional observation
            CovMatrixInv((utid+1), (utid+1)) = 1e8;
            %CovMatrixInv((utid+1):(utid+6), (utid+1):(utid+6)) = 1e8*eye(6);
            utid = utid + 1;%6;        
        elseif(InertialDelta_options.bVarBias == 0)
            if(InertialDelta_options.bAddZbf == 1)
                CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                    1/(sigma_bf_cov*sigma_bf_cov)*eye(3);%1e8
                utid = utid + 3;
            end 
            if(InertialDelta_options.bAddZbw == 1)
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
        if(InertialDelta_options.bGNopt == 1)
        %% GN Iterations 
            [X_obj, nReason] = fnVI_BA_general(K, X_obj, nPoseNew, nPts, Jd, CovMatrixInv, ...
                            nMaxIter, fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ...
                            ImuTimestamps, dtIMU, RptFeatureObs );
            nReason
        else    
            [X_obj,nReason,Info] = fnleastsquaresLM(nUV, K, X_obj, nPoseNew, nPts, Jd, ...
                CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs);        
        end        
        
        toc
        
        if(InertialDelta_options.bPreInt == 0)
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

        ef = XObject2Vector( XObjectDiff( X_obj, Xg_obj ));
        [maxe, idx] = max(abs(ef));
        fprintf('Final Error: maxXef=%f, idx=%d\n', maxe, idx);

        if(nReason < 0)
            return;
        end
    
    if((nPoseOld == 1) || (nPoseNew == nAllposes))
        Tcam = zeros(3, nPoseNew);
        Timu = Tcam;
        
        if(InertialDelta_options.bPreInt == 1)
            for(pid = 2:nPoseNew)
                %Rcam = Ru2c*Rimu*Ru2c;
                %Rimu = fnR5ABG(x(6*(pid-2)+1), x(6*(pid-2)+2), x(6*(pid-2)+3));
                %Timu(:, pid) = x((6*(pid-2)+4):(6*(pid-2)+6),1);
                Rimu = fnR5ABG( X_obj.pose(pid-1).ang(1), X_obj.pose(pid-1).ang(2), X_obj.pose(pid-1).ang(3));
                Timu(:, pid) = X_obj.pose(pid-1).trans;
                Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
            end
            
        else
            
            for(pid = 2:nPoseNew)
                cid = ImuTimestamps(pid)-ImuTimestamps(1);
                Rimu = fnR5ABG(X_obj(6*(cid-1)+1), X_obj(6*(cid-1)+2), X_obj(6*(cid-1)+3));
                Timu(:, pid) = X_obj((6*(cid-1)+4):(6*(cid-1)+6),1);
                Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
            end        
        end
        
        load(gtFile);        
        figure(); hold on;
        if(InertialDelta_options.bMalaga == 1)
            % plot(GT_P0(:,4),GT_P0(:,6),'-+r');
            % plot(Tcam(1,:),Tcam(3,:),'-*b');
        elseif(InertialDelta_options.bDinuka == 1)
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
        save('x_Jac.mat', 'X_obj');
        
        %% Show pose-feature graph
        if((InertialDelta_options.bShowFnP == 1) && (nPoseNew == nAllposes))
            fnShowFeaturesnPoses_general(X_obj, nPoseNew, nPts, nIMUdata, 'Final Values');
            
            %% Show uncertainty
            fnCalnShowUncert_general(nUV, RptFeatureObs, ImuTimestamps, ...
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
    
