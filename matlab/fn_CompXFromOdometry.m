function [X, RptFidSet, RptFeatureObs, nPts] = fn_CompXFromOdometry( ...
                    nPoseOld, nPoseNew, nPoses, nPts, x_old, ...
                    ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
                    RptFidSet_old, dtIMU, dp, dv, dphi, K, RptFeatureObs, ...
                    fscaleGT, kfids, nIMUrate, X, SLAM_Params, imufulldata )
   
    global PreIntegration_options
    
    xcol = 0;

    nPts_old = size(RptFidSet_old,1);
    if(nPoseOld == 1)
        R0imu = eye(3);
        T0imu = zeros(3,1);
        v0imu = zeros(3,1);
        
    elseif(nPoseOld > 1)
        
        nIMUdata_old = ImuTimestamps(nPoseOld) - ImuTimestamps(1);
        R0imu = zeros(3,3,nPoseOld);
        R0imu(:,:,1) = eye(3);
        T0imu = zeros(3,nPoseOld);
        v0imu = zeros(3,nPoseOld);             

        pids = (1:nPoseOld)';
        idx_v = 6*(nPoseOld-1)+3*nPts_old + 3*(pids-1);

        for(id=1:size(pids,1))%2:pid_end)
            
            if(id > 1)
                pid = pids(id);
                idx = 6*(pid-2);
                A0imu = x_old.pose(pid-1).ang.val;
                R0imu(:, :, id) = fn_RFromABG(A0imu(1), A0imu(2), A0imu(3));
                T0imu(:, id) = x_old.pose(pid-1).trans.val; 
            end
            
            v0imu(:, id) = x_old.velocity(id).xyz; 
            
        end
    else
        fprintf('\n nPoseOld == %d? Unexpected!\n', nPoseOld);
        return;
    end    
    
    % Intial values of poses and IMU states
     if(PreIntegration_options.bIMUodo == 1)
         
        %% Obtain initial poses from IMU data
        [Rcam, ~, Tcam, vimu, Feature3D, RptFidSet, RptFeatureObs] = fn_GetPosesFromIMUdata_Inc(nPoseOld, ...
            nPoseNew, nPoses, R0imu, T0imu, v0imu, dtIMU, dp, dv, dphi, ...
            K, Feature3D, RptFidSet, RptFeatureObs, SLAM_Params );
        nPts = size(RptFidSet,1);
        
     else
        
        %% obtain relative poses from visual odometry
        [Rcam, ~, Tcam, Feature3D] = fn_GetPosesFromMatchedFeatures(nPoseNew, nPts, ...
                    K, fscaleGT, RptFeatureObs, kfids);  
                
     end
            
    ABGimu = zeros(3, nPoseNew);%nPoses+1);
    Timu = zeros(3, nPoseNew);%nPoses+1); % ABGimu = zeros(3, nPoses);            

    for(pid=(nPoseOld+1):nPoseNew)%(nPoses+1))% correspond to pose 1...n
        Rimu = SLAM_Params.Ru2c' * Rcam(:,:,pid) * SLAM_Params.Ru2c;
        [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fn_ABGFromR(Rimu);
        Timu(:, pid) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, pid) - Rimu' * SLAM_Params.Tu2c;
    end

    %% Combine old and new poses into X        
    idend = 0;
    if(nPoseOld > 1)
        for p = 1:nPoseOld-1
            X.pose(p).ang.val = x_old.pose(p).ang.val;
            X.pose(p).ang.col = (1:3) + xcol; xcol = xcol + 3;

            X.pose(p).trans.val = x_old.pose(p).trans.val;
            X.pose(p).trans.col = (1:3) + xcol; xcol = xcol + 3;
        end
    end
    for p = nPoseOld-1+1:nPoseNew-1
        X.pose(p).ang.val = ABGimu(:,p+1);
        X.pose(p).ang.col = (1:3) + xcol; xcol = xcol + 3;

        X.pose(p).trans.val = Timu(:,p+1);
        X.pose(p).trans.col = (1:3) + xcol; xcol = xcol + 3;
    end

    % Intial values of features at the initial IMU pose
    % Given Pf1c, Pf1u can be calculated as:
    % Pf1u = Ru2c'*Pf1c + Tu2c
    
    actualNumFeatures = size(RptFidSet);
    for fidx=1:actualNumFeatures
        fid = RptFidSet(fidx);
        tv = Feature3D(fid).triangs(1).p3D ; 
        Pf1u = SLAM_Params.Ru2c' * tv + SLAM_Params.Tu2c;
        
        X.feature(fidx).xyz = Pf1u;
        X.feature(fidx).col = (1:3) + xcol;
        xcol = xcol + 3;
    end
    X.feature(actualNumFeatures+1:end) = [];%delete unnecessary features 
    
    if(nPoseOld > 1)
        [~, idrptf, idrptf_old] = intersect(RptFidSet, RptFidSet_old);
        for(fidx=1:size(idrptf,1))
            fid = idrptf(fidx);
            fid_old = idrptf_old(fidx);
            
            X.feature(fid).xyz = x_old.feature(fid_old).xyz;
        end
    end

    %% Initial values of velocity        
    if(nPoseOld > 1)
        for i=1:nPoseOld
            X.velocity(i).xyz = x_old.velocity(i).xyz;
            X.velocity(i).col = (1:3) + xcol;
            xcol = xcol + 3;
        end
    end % if nPoseOld > 1

    [tv,xcol,~] = fn_CalcVFromKposes_Inc( nPoseNew, nPoseOld, ...
                    nPoses, nPts, nIMUdata, ImuTimestamps, nIMUrate, ...
                    X, xcol, dtIMU, dp, dv, SLAM_Params,imufulldata );

    if (nPoseOld == 1)
        v_range = 1:nPoseNew;
    else
        v_range = nPoseOld+1:nPoseNew;
    end
    X.velocity(v_range) = tv;

    % Intial values of g 
    X.g.val = SLAM_Params.g0;
    X.g.col = (1:3) + xcol;
    xcol = xcol + 3;

    
    %% Au2c, Tu2c
    X.Au2c.val = SLAM_Params.Au2c;
    X.Au2c.col = (1:3) + xcol ;    xcol = xcol + 3;
    
    X.Tu2c.val = SLAM_Params.Tu2c;
    X.Tu2c.col = (1:3) + xcol ;    xcol = xcol + 3;
    
    %% bf, bw
    if(PreIntegration_options.bVarBias == 0)
        X.Bf.val = SLAM_Params.bf0;
        X.Bf.col = (1:3) + xcol ;    xcol = xcol + 3;

        X.Bw.val = SLAM_Params.bw0;
        X.Bw.col = (1:3) + xcol ;    xcol = xcol + 3;
    else
        for ( pid = 2:nPoseNew )
            X.Bf.iter(pid-1).val = SLAM_Params.bf0;
            X.Bf.iter(pid-1).col = (1:3) + xcol;    xcol = xcol + 3;
        end
        
        for ( pid = 2:nPoseNew )
            X.Bw.iter(pid-1).val = SLAM_Params.bw0;
            X.Bw.iter(pid-1).col = (1:3) + xcol;    xcol = xcol + 3;
        end        
    end
        