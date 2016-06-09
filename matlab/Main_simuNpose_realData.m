%% The main function.
clear;
close all;
clc;

%% Choose to use simulated data or or real data.
global PreIntegration_options
PreIntegration_options.bInc = 0;
PreIntegration_options.bSimuNpose = 1;
run PreIntegration_config_script

global Data_config
run Data_config_script

%% Configure the conditions of the problem
nPoses = PreIntegration_options.nPoses;
kfids = 1:PreIntegration_options.kfspan:1200;
nPts = 60000;%15000;%6640;%3243;%800;%14172;%1614;%58404;%
fMaxDistance = 1e2;%50;%5e2;%80;%30;%20;%10;%84;%60;


% if(bPreInt == 1)
%    bShowFnP = 1;
% else
%    bShowFnP = 0; 
% end

%% Configure paramters according to the chosen dataset
if(PreIntegration_options.bMalaga == 1)
    dtIMU                       = [];
    nIMUrate                    = 1e2;
    SLAM_Params.sigma_w_real    = 1e1*(5e-2)*pi/180;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    SLAM_Params.sigma_f_real    = 1e1*2e-3;    
    SLAM_Params.sigma_w_cov     = 2 * SLAM_Params.sigma_w_real;%5e1*(5e-2)*pi/180;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    SLAM_Params.sigma_f_cov     = 2 * SLAM_Params.sigma_f_real;%5e1*2e-3;%1e-2;%1e-5;%1;%1e0;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%1e0;%2e-3;%
    SLAM_Params.sigma_uov_real  = 1e-3;%1;%1e-6;%1e-4;%
    SLAM_Params.sigma_uov_cov   = 2 * SLAM_Params.sigma_uov_real;%1;%1e-3;%
    SLAM_Params.sigma_g_cov     = 1e-1;
    SLAM_Params.sigma_au2c_cov  = 1e-1;
    SLAM_Params.sigma_tu2c_cov  = 1e-1; 
    SLAM_Params.sigma_bf_cov    = 1 * 2e-2;
    SLAM_Params.sigma_bw_cov    = 1 * 20 * pi / (180 * 3600); 
    SLAM_Params.sigma_tv        = 1e-4 / (nIMUrate);
    % Configure pseudo observations
    
    SLAM_Params.g0              = [0; 0; -9.8]; % g value in the first key frame
    SLAM_Params.g_true          = [0; 0; -9.8];
    SLAM_Params.bf0             = zeros(3, 1); %[-0.55;0.6;0.61];
    SLAM_Params.bw0             = zeros(3, 1);
    SLAM_Params.bf_true         = [1.0; 2.0; 3.0]; % bias for acceleration
    SLAM_Params.bw_true         = [1.0; 2.0; 3.0]; %[0, 0, 0]'; % bias for rotaion velocity     
    % Directories
elseif(PreIntegration_options.bDinuka == 1)
    dtIMU                       = zeros( nPoses, 1 );
    nIMUrate                    = 2e2;    
    SLAM_Params.sigma_w_real    = 0.0;%0.03;%0.01;%0.15;% 5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    SLAM_Params.sigma_f_real    = 0;%0.2;%0.1;%1;%  
    SLAM_Params.sigma_w_cov     = 1;%2*sigma_w_real;%0.03;%0.1;% 0.01;%5e-1;%1;%1e-2;%1;%1e-2;%5e-1;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%5e-1;%5e-2;%
    SLAM_Params.sigma_f_cov     = 1;%2*sigma_f_real;%0.2;%0.8;% 0.1;%1e-2;%1e-5;%1;%1e0;%4.5e-3;%2.5e-1;%2e-1;%3.5e-1;%1e0;%2e-3;%
    SLAM_Params.sigma_uov_real  = 0;%1e-2;%2e-3;%1e-1;%1;%1e-6;%1e-3;%1e-4;%
    SLAM_Params.sigma_uov_cov   = 1;%2*sigma_uov_real;%1e-2;%
   
    SLAM_Params.sigma_g_cov     = 1e-4;
    SLAM_Params.sigma_au2c_cov  = 1e-4;
    SLAM_Params.sigma_tu2c_cov  = 1e-4;
    SLAM_Params.sigma_bf_cov    = 1 * 2e-2;
    SLAM_Params.sigma_bw_cov    = 1 * 20 * pi / (180*3600); 
    SLAM_Params.sigma_tv        = 1e-4 / (nIMUrate);%1;%*nIMUrate*nIMUrate);
    % Configure pseudo observations
    
    SLAM_Params.g0              = [0; 0; 9.81]; % g value in the first key frame
    SLAM_Params.g_true          = [0; 0; 9.81];
    SLAM_Params.bf0             = zeros(3, 1); %[-0.55;0.6;0.61];
    SLAM_Params.bw0             = zeros(3, 1);
    SLAM_Params.bf_true         = [0; 0; 0]; % bias for acceleration
    SLAM_Params.bw_true         = [0; 0; 0]; %[0, 0, 0]'; % bias for rotaion velocity     
    % Directories
end
% Iteration times and bounds for Gauss-Newton
SLAM_Params.nMaxIter = 1e3;%50;%30;%100;%15;%5;%10;%50;%3;% 20;% 
SLAM_Params.fLowerbound_e = 1e-10;%1e-6;%1e-5;%1e-1;
SLAM_Params.fLowerbound_dx = 1e-10;%1e-6;%
SLAM_Params.fLowerbound_chi2Cmpr    = 1e-4;


addpath( genpath( 'IMU' ));
addpath( genpath( 'MoSeg_2D' ));%addpath(genpath('ms3D'));
addpath( genpath( 'Ransac' ));

%save( [ Data_config.TEMP_DIR 'bVarBias.mat' ],'bVarBias');

uvd_cell    = [];
dp          = zeros(3, nPoses);
dv          = zeros(3, nPoses);
dphi        = zeros(3, nPoses);
Jd          = [];
Rd          = [];


%% The main switch
    if( PreIntegration_options.bMalaga == 1 )
        K = [923.5295, 0, 507.2222; 0, 922.2418, 383.5822; 0, 0, 1];% Left
          % [911.3657, 0, 519.3951; 0, 909.3910, 409.0285; 0, 0, 1]; &Right
    elseif( PreIntegration_options.bDinuka == 1 )
       load( [ Data_config.DATA_DIR 'cam.mat' ] ); 
       K = cam.K;
    end   
    
    
    if(PreIntegration_options.bMalaga == 1)
        SLAM_Params.Au2c    = [-87.23; -2.99; -88.43] * pi / 180;%[0;0;0];%[-86.19;-3.53;-90.31]*pi/180;%
        SLAM_Params.Ru2c    = fn_RFromAngVec( SLAM_Params.Au2c );
        SLAM_Params.Tu2c    = [ 2.2 - 0.25; -0.427 - 0.029; 0.025 + (23 - 13.9) * 1e-3 ];%[0;0;0];%
        
        dt      = 1e-2;
        %     bPerfectIMUdlt = 1;   
        %         bAddZg = 1; % Add pseudo observation of g or not
        %         bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
        %         bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
        %         bAddZbf = 0; % Add pseudo observation of bias in acceleration or not
        %         bAddZbw = 0; % Add pseudo observation of bias in rotation or not    
        gtIMUposes = [];
        selpids = [];
        
    elseif(PreIntegration_options.bDinuka == 1)
        load( [ Data_config.DATA_DIR 'gtIMUposes.mat' ] );% ts, Aimu, Timu
        nt = size( gtIMUposes, 1 ); 
        selpids = 9 : (10 * PreIntegration_options.kfspan) : nt;
        SLAM_Params.Ru2c = ( [ 0, 1, 0; 0, 0, 1; 1, 0, 0 ] ); 
        SLAM_Params.Au2c = zeros(3,1);
        [ SLAM_Params.Au2c(1), SLAM_Params.Au2c(2), SLAM_Params.Au2c(3) ]= fn_ABGFromR(SLAM_Params.Ru2c);
        SLAM_Params.Tu2c = zeros(3,1);
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


    %%%%%%%%%%%%%%
    % camera observations  

    [ FeatureObs, Feature3D, imufulldata, dataIMU, ImuTimestamps, dtIMU, nIMUdata, dp, dv, dphi, Jd, Rd ] = ...
                                            LoadData( nPts, nPoses, kfids, SLAM_Params );
    for pid = 1 : nPoses
        [FeatureObs] = fn_CollectfObsFromImgs( ...
                            kfids, pid, Data_config.imgdir, SLAM_Params.sigma_uov_real, FeatureObs );
    end

    [ RptFidSet, RptFeatureObs, PBAFeature ] = fn_GetUniqueFeatures( FeatureObs, nPoses, fMaxDistance );

    save( [ Data_config.TEMP_DIR, 'RptFeatureObs.mat' ], 'RptFeatureObs');

    nPts = size(RptFidSet, 1);
        
    
    %% X---the state vector
        X_obj = SLAM_X_Define( nPts, nPoses, nIMUrate );
        Xg_obj = X_obj;        
       [X_obj, Xg_obj, Feature3D ] = fn_Build_Xinit_and_Xgt( ...
                                        X_obj, Xg_obj, RptFeatureObs, RptFidSet, Feature3D, gtIMUposes, selpids, PBAFeature, ...
                                        nPoses, nPts, nIMUdata, nIMUrate, dtIMU, ...
                                        ImuTimestamps, imufulldata, dp, dv, dphi, K, dt, SLAM_Params );
       
        if ( PreIntegration_options.bTestIntegrity == 1 )
            %--------------------------------------------------
            % Hack: force X_obj to have same pose as GT
            %--------------------------------------------------    
            X_obj = Xg_obj;
            %--------------------------------------------------
        end

        X_init = X_obj;    
        save([ Data_config.TEMP_DIR 'X_init.mat'],'X_init');
        
        % Show initial value X0
        fprintf('\nInitial Value:\n\t X0=[\nAng: ');
        %fprintf('%f ', x(1:20));
        fprintf('%f ', [X_obj.pose(1).ang.val; X_obj.pose(2).ang.val; X_obj.pose(3).ang.val]);
        fprintf('\nTrans: ');
        fprintf('%f ', [X_obj.pose(1).trans.xyz; X_obj.pose(2).trans.xyz; X_obj.pose(3).trans.xyz]);
        fprintf('\nFeature_1: ');
        fprintf('%f ', X_obj.feature(1).xyz');
        fprintf('...]\n');    
        
    if 0
        ie = x-xg;
        [me, id] = max(abs(ie))            
    end
    
    % Show Pose-feature graph
    if ( PreIntegration_options.bShowFnP == 1 )
        fn_ShowFeaturesnPoses( Xg_obj, nPoses, nPts, nIMUdata, 'Ground Truth Values');
        %	x(1:6*(nPoses-1)) = camposes; % x(1:6*nIMUrate(nPoses-1)) = camposes;
        fn_ShowFeaturesnPoses( X_obj, nPoses, nPts, nIMUdata, 'Initial Values');
    end

    
    %% Z---the observation vector
    Zobs = fn_Collect_Zobs_realdata( RptFeatureObs, Xg_obj, dataIMU, nPoses, nPts, nIMUrate, dp, dv, dphi, SLAM_Params );
    
    %% Covariance Matrix
    %((dataIMU{2}(2, 1) - dataIMU{2}(1, 1)))*size(dataIMU{2},1);
    save( [ Data_config.TEMP_DIR 'consts.mat' ], 'nIMUrate', 'K', 'Zobs', 'nPoses', 'nPts', 'SLAM_Params', 'dt', 'Jd' );
    save( [ Data_config.TEMP_DIR 'Zobs.mat' ], 'Zobs' ); 
    save( [ Data_config.TEMP_DIR 'RptFeatureObs.mat' ], 'RptFeatureObs' ); 

    %% Covariance matrix
    CovMatrixInv = fn_Generate_CovMatrixInv_realdata( nPoses, nPts, length(Zobs.fObs)*2, nIMUdata, Rd, SLAM_Params );
    save( [ Data_config.TEMP_DIR, 'CovMatrixInv.mat' ],'CovMatrixInv', '-v7.3');        
   
tic
    if ( PreIntegration_options.bGNopt == 1 )
        %% GN Iterations 
        [X_obj, nReason] = fn_GaussNewton_GraphSLAM( K, X_obj, nPoses, nPts, Jd, CovMatrixInv, ...
                        nIMUrate, nIMUdata,  ImuTimestamps, dtIMU, RptFeatureObs, SLAM_Params );

    else    
        [x,Reason,Info] = fn_LeastSqrLM_GraphSLAM( nUV, K, x, nPoses, nPts, Jd, ...
                        CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
                        bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);
    end   
    X_final = X_obj;
toc
    if(PreIntegration_options.bPreInt == 0)
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

    ef = SLAM_X_Object2Vector( SLAM_X_ObjectDiff( X_final, Xg_obj ));
    [maxe, idx] = max(abs(ef));
    fprintf('Final Error: maxXef=%f, idx=%d\n', maxe, idx);
    
    if(nReason < 0)
        return;
    end
    Tcam = zeros(3, nPoses);
    Timu = Tcam;
    if(PreIntegration_options.bPreInt == 1)
        for(pid = 2:nPoses)
            %Rcam = Ru2c*Rimu*Ru2c;
            if 1
                Rimu = fn_RFromAngVec( X_obj.pose(pid-1).ang.val );
                Timu(:, pid) = X_obj.pose(pid-1).trans.xyz;
                Tcam(:, pid) = SLAM_Params.Ru2c * ( Timu(:, pid) - SLAM_Params.Tu2c + Rimu' * SLAM_Params.Tu2c );
            else
                Rimu = fn_RFromABG(x(6*(pid-2)+1), x(6*(pid-2)+2), x(6*(pid-2)+3));
                Timu(:, pid) = x((6*(pid-2)+4):(6*(pid-2)+6),1);
                Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
            end
        end
    else
        for(pid = 2:nPoses)
            cid = ImuTimestamps(pid) - ImuTimestamps(1);
            if 1
                Rimu = fn_RFromAngVec( X_obj.pose(cid).ang.val );
                Timu(:, pid) = X_obj.pose(cid).trans.xyz;
                Tcam(:, pid) = SLAM_Params.Ru2c * ( Timu(:, pid) - SLAM_Params.Tu2c + Rimu' * SLAM_Params.Tu2c );
            else
                Rimu = fnR5ABG(x(6*(cid-1)+1), x(6*(cid-1)+2), x(6*(cid-1)+3));
                Timu(:, pid) = x((6*(cid-1)+4):(6*(cid-1)+6),1);
                Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
            end
        end        
    end
    load( Data_config.gtFile );
    figure(); hold on;
    if(PreIntegration_options.bMalaga == 1)
        %	plot(GT_P0(:,4),GT_P0(:,6),'-+r');
        %   plot(Tcam(1,:),Tcam(3,:),'-*b');
    elseif(PreIntegration_options.bDinuka == 1)
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
        err = Tcam(:,1:nPoses) - GT_P0(1:nPoses, 4:6)';
    ce = complex(err(1,:), err(2,:));
    ce = complex(abs(ce), err(3,:));  
    plot(1:nPoses, abs(ce), 'p');
    title('Pose Translational Error');
    %     stop;
    %%%%%%%%%%%%    
    save( [ Data_config.TEMP_DIR, 'X_Jac.mat' ], 'X_final');
    %% Show pose-feature graph
    if ( PreIntegration_options.bShowFnP == 1 )
        fnShowFeaturesnPoses_general(x, nPoses, nPts, nIMUdata, 'Final Values');
        %     fnShowFeaturesnPoses(xf, nPoses, nPts, nIMUrate, bPreInt, 'Final Values');
    end
    %% Show uncertainty
    if ( PreIntegration_options.bShowUncertainty == 1 )
        fnCalnShowUncert_general( RptFeatureObs, ImuTimestamps, ...
            dtIMU, ef, K, X_obj, nPoses, nPts, Jd, CovMatrixInv, nIMUrate, nIMUdata );
    end
    
    fn_ShowFeaturesnPoses_superimpose( Xg_obj, X_init, X_final );

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
	   Data_config.DATA_DIR = [dataDir_root, 'data', filesep];% cluster %(HP)'/mnt/B0A18DDEEC101C79/uDocs/Research/MyPapers/acra2013/code/SegData/data10/'; %
       nfiles = size(dir(Data_config.DATA_DIR),1)-3;%2;%5;%
	   %saveDir = ['../datasetinfo/', datsetname, '/unix/'];%/home/yowang/Documents/segDataset/
	else % my pc
	   %Data_config.DATA_DIR = ['E:\uDocs\Research\MyPapers\acra2013\code\SegData\', datasetname, filesep]; 
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
        sfrgb1= sprintf('%s%010d.png', Data_config.DATA_DIR, nPose);
        %	sfrgb1= sprintf('%srgb%04d.png', Data_config.DATA_DIR, nPose);%ni{ndg}()
        % 	sfd1= sprintf('%sdepth%04d.png', Data_config.DATA_DIR, nPose);%ni{ndg}()
		if(~exist(fullfile(saveDir, sprintf('%010d.mat', nPose))))            
		    %sfrgb2= sprintf('%srgb%04d.png', dir, nPose+1);%ni{ndg}()
		    %sfd2= sprintf('%sdepth%04d.png', dir, nPose+1);%ni{ndg}()

            %	[fn, ~] = Collectimgfs(sfrgb1, sfd1, nPose, dfactor, K, ...
            % 		fMthreshold, nSlevel, cSift, cDataset, dmin, dmax);
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
        %	descr1=uint8(512*fn(:, 8:end)) ;
        %   descr2=uint8(512*fk(:, 8:end)) ;
        %   matchpairs = siftmatch(descr1', descr2');%(fn(:, 8:end))', (fk(:, 8:end))');
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

