%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% e = f(X) - Zobs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function e = SLAM_CalcPredictionError( X, nPoses, nPts )

    global PreIntegration_options Data_config
    
    addpath(genpath('IMU'));
    load( [ Data_config.TEMP_DIR 'consts.mat' ]); 
    %%%%
    %%%% Zobs is loaded
    %%%%
    
    
    load( [ Data_config.TEMP_DIR 'RptFeatureObs.mat' ] );%obsfeatures
    
    if(PreIntegration_options.bUVonly == 0)
        load( [ Data_config.TEMP_DIR 'ImuTimestamps.mat' ] );
        load( [ Data_config.TEMP_DIR 'dtIMU.mat' ] );
    else
        ImuTimestamps = [];
        dtIMU = [];
    end

    e = Zobs;

    
    %ZobsPIDelta = struct ( ...
    %    'preInt',   Zobs.preInt, ...
    %    'Au2c',     Zobs.Au2c, ...
    %    'Tu2c',     Zobs.Tu2c, ...
    %    'Bf',       Zobs.Bf, ...
    %    'Bw',       Zobs.Bw ...
    %    );

    %nPoses = length( Zobs.fObs ) + 1;
    %nPts = length( X.feature );
    
    % 1. UVD error:
    e  = fn_CalcPredictionError_Zuv(RptFeatureObs, K, X, Zobs, nPoses, nPts, nIMUrate, ImuTimestamps );

    % 2. IMU dlt error:
    ePreInt = fn_CalcPredictionError_ZintlDelta(X, Zobs, nPoses, nPts, SLAM_Params.bf0, ...
                        SLAM_Params.bw0, dtIMU, inertialDelta.Jd, nIMUrate, ImuTimestamps );
    
    if ( PreIntegration_options.bPreInt == 1 )
        e.intlDelta =  ePreInt.intlDelta;
    else
        e.imu       = ePreInt.imu;
    end
    
    if ( PreIntegration_options.bAddZg == 1 )
        e.g         =  ePreInt.g;
    end
    
    e.Au2c          =  ePreInt.Au2c;
    e.Tu2c          =  ePreInt.Tu2c;
    e.Bf            =  ePreInt.Bf;
    e.Bw            =  ePreInt.Bw;
    
    



