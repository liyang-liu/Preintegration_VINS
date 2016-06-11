function [  imufulldata, dataIMU, ImuTimestamps, dtIMU, nIMUdata, dp, dv, dphi, Jd, Rd ] = ...
                    LoadData( nPts, nAllposes, kfids, SLAM_Params )

    global PreIntegration_options Data_config
    
    % Reset random number generator seed
    rng( 'default' );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    nIMUdata = 0;    
    ImuTimestamps = zeros( nAllposes, 1 );    
    dtIMU = [];
    
    if(PreIntegration_options.bUVonly == 0)
        % IMU observations
        if(PreIntegration_options.bMalaga == 1)
            load( [ Data_config.imgdir 'KeyframeTimestamps.mat' ] );
        elseif(PreIntegration_options.bDinuka == 1)
            fname = [ Data_config.imgdir 'image_time_stamp.mat' ];
            load( fname );
            KeyframeTimestamps = vis_time(kfids);
        end

        load(Data_config.imufulldir);
        
        if(PreIntegration_options.bMalaga == 1)
            imufulldata = IMUparking6L;
        elseif(PreIntegration_options.bDinuka == 1)
            nt = size( imudata, 1 );
            imufulldata = imudata;% ts, wb,fb
            imufulldata(:,2:4) = imufulldata(:,2:4) + fn_GenGaussNoise(nt, 3, SLAM_Params.sigma_w_real);
            imufulldata(:,5:7) = imufulldata(:,5:7) + fn_GenGaussNoise(nt, 3, SLAM_Params.sigma_f_real);
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
        nIMUdata = ImuTimestamps(nAllposes) - ImuTimestamps(1);
        dataIMU = {};     
        for pid=2:nAllposes
            uid0 = ImuTimestamps(pid-1);
            uid1 = ImuTimestamps(pid)-1;

            dtIMU(pid) = imufulldata(ImuTimestamps(pid)) - imufulldata(ImuTimestamps(pid-1));
            if(PreIntegration_options.bMalaga == 1)
                dataIMU{pid} = [IMUparking6L(uid0:uid1, 1), IMUparking6L(uid0:uid1, 7), IMUparking6L(uid0:uid1, 6),...
                    IMUparking6L(uid0:uid1, 5), IMUparking6L(uid0:uid1, 2:4)];
            elseif(PreIntegration_options.bDinuka == 1)
                dataIMU{pid} = imufulldata(uid0:uid1, :);                
            end
        end 
        if(PreIntegration_options.bUVonly == 0 )
            % Generate pre-integration observations based on IMU raw data.         
            
            %%%%%%%%%%%%%%
            % Pre Integration
            %%%%%%%%%%%%%%
            
            dp = zeros(3,nAllposes);
            dv = dp; 
            dphi = dp;    
            for pid=2:nAllposes%1e-2,1e-3+3*sigmaf+3*sigmaw
                [dp(:,pid), dv(:,pid), dphi(:,pid), Jd{pid}, Rd{pid}] = ...
                    fn_DeltObsAccu(SLAM_Params.bf0, SLAM_Params.bw0, dataIMU{pid}, SLAM_Params.sigma_w_cov, SLAM_Params.sigma_f_cov); 
            end
        end
                
        save( [ Data_config.TEMP_DIR 'ImuTimestamps.mat' ], 'ImuTimestamps' );
        save( [ Data_config.TEMP_DIR 'dtIMU.mat' ], 'dtIMU' );
    end
