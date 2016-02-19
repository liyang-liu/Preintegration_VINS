%%
% Inertial Delta SLAM options
% Modify options here to experiment with different test option
%% 

%% Choose to use simulated data or or real data.
InertialDelta_options.bSimData = 0; % p15-30

% Select one of the real datasets
InertialDelta_options.bMalaga = 0;
InertialDelta_options.bDinuka = 1;%p4-15/nonoise:50-80(+50)

%% Configure the conditions of the problem
InertialDelta_options.nAllposes = 60;%50;%25;%240;%13;%4;%20;%5;%240;%60;% 350;%
InertialDelta_options.bPreInt = 1;%1;% Use pre-integration method?
    
InertialDelta_options.bInitPnF5VoU = 1;% Use visual odometry or IMU data to initialize x?
InertialDelta_options.bIMUodo = 1;% Use IMU data to initilize x?
InertialDelta_options.bGNopt = 1;% Use Gauss-Newton method?
InertialDelta_options.bShowFnP = 0;% Show poses and features?
InertialDelta_options.bShowUncertainty = 0;% Show uncertainty of the result?
InertialDelta_options.bAddInitialNoise = 0;% x0 + noise or not
InertialDelta_options.bPerfectIMUdlt = 0;
InertialDelta_options.bVarBias = 0;

InertialDelta_options.bUVonly = 0;% If 1, only UVs are used; otherwise, IMU and UVs are fused.
InertialDelta_options.bAddZg = 0; % Add pseudo observation of g or not
InertialDelta_options.bAddZau2c = 1; % Add pseudo observation of the relative rotation from IMU to camera or not
InertialDelta_options.bAddZtu2c = 1; % Add pseudo observation of the relative translation from IMU to camera or not   
InertialDelta_options.bAddZbf = 1; % Add pseudo observation of bias in acceleration or not
InertialDelta_options.bAddZbw = 1; % Add pseudo observation of bias in rotation or not    

InertialDelta_options.nPoseOld = 1;
InertialDelta_options.nAddPoses = 1;%10;%2;%4;%5;%50;%30;%1;%
InertialDelta_options.nPoseNew = 4;%3;%50;%10;%10;%5;%15;%30;%80;%120;%30;%25;%20;%18;%13;%3;%5;%10;%4;%50;%9;%2;%170;%6;%120;%
InertialDelta_options.kfspan = 10;%1;%2;%5;%10;%20;%15;%50;%30;% Choose keyframes
InertialDelta_options.nMinObsTimes = 2;%10;%5;%3;%50;%30;%20;%4;%6;%
InertialDelta_options.fMaxDistance = 1e6;%50;%1e2;%5e2;%80;%30;%20;%10;%84;%60;
InertialDelta_options.nPts = 60000;%15000;%6640;%3243;%800;%14172;%1614;%58404;%
