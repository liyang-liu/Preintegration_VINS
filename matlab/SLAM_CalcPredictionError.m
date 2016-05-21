%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% e = f(X) - Zobs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function e = SLAM_CalcPredictionError( X )

    global PreIntegration_options Data_config
    
    addpath(genpath('IMU'));
    load( [ Data_config.TEMP_DIR 'consts.mat' ]); 
    %%%%
    %%%% Zobs is loaded
    %%%%
    
    %load( [ Data_config.TEMP_DIR, 'bAddZg.mat' ]);% Use the latest value
    %load( [ Data_config.TEMP_DIR, 'bAddZau2c.mat' ]);
    %load( [ Data_config.TEMP_DIR, 'bAddZtu2c.mat' ]);
    %load( [ Data_config.TEMP_DIR, 'bAddZbf.mat' ]);
    %load( [ Data_config.TEMP_DIR, 'bAddZbw.mat' ]);
    %load( [ Data_config.TEMP_DIR, 'bUVonly.mat' ]);
    %load( [ Data_config.TEMP_DIR, 'bVarBias.mat' ]);
    
    load( [ Data_config.TEMP_DIR 'RptFeatureObs.mat' ] );%obsfeatures
    
    if(PreIntegration_options.bUVonly == 0)
        load( [ Data_config.TEMP_DIR 'ImuTimestamps.mat' ] );
        load( [ Data_config.TEMP_DIR 'dtIMU.mat' ] );
    else
        ImuTimestamps = [];
        dtIMU = [];
    end

    %e = zeros(size(Zobs));
    e = Zobs;

    % 1. UVD error:
    %[e, nUV] = fnUVDErr_C1U_genral(RptFeatureObs, K, X, Zobs, nPoseNew, nPts, ImuTimestamps );
    % 2. IMU dlt error:
    %[e((nUV+1):end,1)] = fnIMUdltErr_general(X, Zobs((nUV+1):end,1), nPoseNew, nPts, bf0, ...
    %    bw0, dtIMU, Jd, nIMUrate, ImuTimestamps );
    
    %ZobsPIDelta = struct ( ...
    %    'preInt',   Zobs.preInt, ...
    %    'Au2c',     Zobs.Au2c, ...
    %    'Tu2c',     Zobs.Tu2c, ...
    %    'Bf',       Zobs.Bf, ...
    %    'Bw',       Zobs.Bw ...
    %    );
    %[e((nUV+1):end,1)] = fnIMUdltErr_general(X, Zobs, nPoseNew, nPts, bf0, ...
    %    bw0, dtIMU, Jd, nIMUrate, ImuTimestamps );

    nPoses = length( X.pose ) + 1;
    nPts = length( X.feature );
    
    % 1. UVD error:
    e  = fn_CalcPredictionError_Zuv(RptFeatureObs, K, X, Zobs, nPoses, nPts, ImuTimestamps );

    % 2. IMU dlt error:
    ePreInt = fn_CalcPredictionError_ZintlDelta(X, Zobs, nPoses, nPts, SLAM_Params.bf0, ...
                            SLAM_Params.bw0, dtIMU, Jd, nIMUrate, ImuTimestamps );
    
    e.intlDelta =  ePreInt.intlDelta;
    
    if ( PreIntegration_options.bAddZg == 1 )
        e.g     =  ePreInt.g;
    end
    
    e.Au2c      =  ePreInt.Au2c;
    e.Tu2c      =  ePreInt.Tu2c;
    e.Bf        =  ePreInt.Bf;
    e.Bw        =  ePreInt.Bw;
    
    



