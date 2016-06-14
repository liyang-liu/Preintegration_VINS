function [Zobs, inertialDelta] = fn_CollectZobs( RptFeatureObs, imuData_cell, nPoses, nPts, nIMUrate, inertialDelta, SLAM_Params )
    global PreIntegration_options

    %% Z: Put camera observations into Z
    %     for pid=1:nPoses % Old sequence: pose oriented
    %         t = uvd_cell{pid}(1:2, :);  
    %         Zobs(((pid-1)*2*nPts+1):(pid*2*nPts),1) = t(:);%uvd_cell{pid}
    % %     	Zobs(((pid-1)*3*nPts+1):(pid*3*nPts),1) = uvd_cell{pid}(:);
    %     end 
    %     RptFeatureObs(:, fId_FeatureObs) = (1:nPts)';
    %     for(pid=1:nPoses)
    %         obsfeatures_i1 = uvd_cell{pid}(1:2, :);
    %         fidset = (1:nPts)';
    %         RptFeatureObs(fidset, nObsId_FeatureObs) = ...
    %             RptFeatureObs(fidset, nObsId_FeatureObs) + 1;
    %         for(fid=1:size(fidset,1))
    %             nObs = RptFeatureObs(fidset(fid), nObsId_FeatureObs);
    %             RptFeatureObs(fidset(fid), 3*nObs) = pid;
    %             RptFeatureObs(fidset(fid), (3*nObs+1):(3*nObs+2)) = (uvd_cell{pid}(1:2, fid))';
    %         end
    %     end   
    %save('RptFeatureObs.mat', 'RptFeatureObs');

    Zobs = SLAM_Z_Define( nPoses, nPts, nIMUrate );
    
    % Order UVs according to fid
    zid = 0;
    zrow = 0;
    
    for(fid=1:nPts)% local id
        nObs = RptFeatureObs(fid).nObs;
        
        for(oid=1:nObs)
            tv = RptFeatureObs(fid).obsv(oid).uv';
            zid = zid + 1;
            Zobs.fObs(zid).uv = tv(:);
            Zobs.fObs(zid).row = (1:2) + zrow;
            zrow = zrow + 2;
        end
    end
    Zobs.fObs(zid+1:end) = []; % delete unused spaces
    nUV = zrow; %num UV rows
    
    %nUV = zidend;
    %idr = zidend;
    
    %     %% Prepare IMU observations  
    %     % Average the raw data
    %     if(bChIMUd == 1)
    %         for pid=2:nPoses
    %             tv = imuData_cell{pid}.samples(:, 2:7);
    %             imuData_cell{pid}.samples(1:(end-1), 2:7) = 0.5 * (tv(2:end, :) + tv(1:(end-1), :));
    %         end
    %     end  

    % Put IMU data into observation vector z: 
    if(PreIntegration_options.bPreInt == 0) % Non-pre-integration: put raw data  
        idx_ImuObs = 0;
        for pid=2:nPoses
            tv = [imuData_cell{pid}.samples(:, 2:7), zeros(nIMUrate,3)]';
            for j=1:nIMUrate
                idx_ImuObs = idx_ImuObs + 1;
                Zobs.imu(idx_ImuObs).w.val = tv(1:3, j); % linear accleartion observation
                Zobs.imu(idx_ImuObs).w.row = (1:3) + zrow; zrow = zrow + 3;
                Zobs.imu(idx_ImuObs).acc.val = tv(4:6, j); %angular rate observation
                Zobs.imu(idx_ImuObs).acc.row = (1:3) + zrow; zrow = zrow + 3;
                Zobs.imu(idx_ImuObs).deltaT.val = tv(7:9, j); % a constraint enforcing 0 = T(i+1,j) - T(i,j) - v(i,j)*deltaT, set to zero before for-loop
                Zobs.imu(idx_ImuObs).deltaT.row = (1:3) + zrow; zrow = zrow + 3;
                
            end
        end 
    else  % Generate pre-integration observations based on IMU raw data.   
        % Add interated IMU observations
        if(PreIntegration_options.bPerfectIMUdlt == 1)
            inertialDelta = fn_CalPerfectIMUdlt(X_obj, nPoses, nPts, inertialDelta, SLAM_Params );
        end
        for p = 2 : nPoses
            Zobs.intlDelta(p-1).deltaP.val = inertialDelta.dp(:, p);
            Zobs.intlDelta(p-1).deltaP.row = (1:3) + zrow;
            zrow = zrow + 3;

            Zobs.intlDelta(p-1).deltaV.val = inertialDelta.dv(:, p);
            Zobs.intlDelta(p-1).deltaV.row = (1:3) + zrow;
            zrow = zrow + 3;

            Zobs.intlDelta(p-1).deltaPhi.val = inertialDelta.dphi(:, p);
            Zobs.intlDelta(p-1).deltaPhi.row = (1:3) + zrow;
            zrow = zrow + 3;
        end
        
    end

    %% Continue filling in Zobs with psedu observations related to IMU

    if(PreIntegration_options.bAddZg == 1)
        % Add pseudo observation of g        
        Zobs.g.val = SLAM_Params.g0;
        Zobs.g.row = (1:3) + zrow;
        zrow = zrow + 3;                    
    end
    
    if(PreIntegration_options.bAddZau2c == 1)
        % Add pseudo observation of Tu2c
        [alpha, beta, gamma] = fn_ABGFromR(SLAM_Params.Ru2c);
        Zobs.Au2c.val = [alpha; beta; gamma];
        Zobs.Au2c.row = (1:3) + zrow;
        zrow = zrow + 3;        
    end
    
    if(PreIntegration_options.bAddZtu2c == 1)
        % Add pseudo observation of Tu2c
        Zobs.Tu2c.val = SLAM_Params.Tu2c;
        Zobs.Tu2c.row = (1:3) + zrow;
        zrow = zrow + 3;
        
    end
    
    if(PreIntegration_options.bAddZbf == 1)
        % Add pseudo observation of bf
        Zobs.Bf.val = SLAM_Params.bf0;
        Zobs.Bf.row = (1:3) + zrow;
        zrow = zrow + 3;
        
    end
    if(PreIntegration_options.bAddZbw == 1)
        % Add pseudo observation of bf
        Zobs.Bw.val = SLAM_Params.bw0;
        Zobs.Bw.row = (1:3) + zrow;
        zrow = zrow + 3;
    end

    % %% Add noise to the uv part of Z    
    % if(bZnoise == 1)    
    %     % Only add noise to UV here, because IMU data has been added before.
    % %     Zobs(1:idr) = Zobs(1:idr) + randn(size(Zobs(1:idr,1)));% fZnoisescale;%
    %     nr = idr; nc = 1;
    %     [gns] = fnGenGaussNoise(nr, nc, 1);
    %     Zobs(1:idr) = Zobs(1:idr) + gns;
    % end

    