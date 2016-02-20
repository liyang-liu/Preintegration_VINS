function [ FeatureObs, Feature3D, imufulldata, ImuTimestamps, dtIMU, dp, dv, dphi, Jd, Rd ] = LoadData( nPts, nAllposes, kfids, SLAM_Params )

    global InertialDelta_options Data_config

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
    dtIMU = [];
    
    if(InertialDelta_options.bUVonly == 0)
        % IMU observations
        if(InertialDelta_options.bMalaga == 1)
            load([Data_config.imgdir 'KeyframeTimestamps.mat']);
        elseif(InertialDelta_options.bDinuka == 1)
            fname = [Data_config.imgdir 'image_time_stamp.mat'];
            load(fname);
            KeyframeTimestamps = vis_time(kfids);
        end

        load(Data_config.imufulldir);
        
        if(InertialDelta_options.bDinuka == 1)
            nt = size(imudata,1);
            imufulldata = imudata;% ts, wb,fb
            rng('default');
            imufulldata(:,2:4) = imufulldata(:,2:4) + fnGenGaussNoise(nt, 3, SLAM_Params.sigma_w_real);
            imufulldata(:,5:7) = imufulldata(:,5:7) + fnGenGaussNoise(nt, 3, SLAM_Params.sigma_f_real);
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
            
            %%%%%%%%%%%%%%
            % Pre Integration
            %%%%%%%%%%%%%%
            
            dp = zeros(3,nAllposes);
            dv = dp; 
            dphi = dp;    
            for pid=2:nAllposes%1e-2,1e-3+3*sigmaf+3*sigmaw
                [dp(:,pid), dv(:,pid), dphi(:,pid), Jd{pid}, Rd{pid}] = ...
                    fnDeltObsAccu(SLAM_Params.bf0, SLAM_Params.bw0, dataIMU{pid}, SLAM_Params.sigma_w_cov, SLAM_Params.sigma_f_cov); 
            end
        end
                
        save( [ Data_config.TEMP_DIR 'ImuTimestamps.mat' ], 'ImuTimestamps' );
        save( [ Data_config.TEMP_DIR 'dtIMU.mat' ], 'dtIMU' );
    end
