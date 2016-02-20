function [X, RptFidSet, RptFeatureObs, nPts] = fnCompX5odometry( ...
                    nPoseOld, nPoseNew, nPoses, nPts, x_old, ...
                    ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
                    RptFidSet_old, dtIMU, dp, dv, dphi, K, RptFeatureObs, ...
                    fscaleGT, kfids, nIMUrate, X, SLAM_Params, imufulldata )
   
    global InertialDelta_options
    
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
        if(InertialDelta_options.bPreInt == 1)               

            pids = (1:nPoseOld)';
            idx_v = 6*(nPoseOld-1)+3*nPts_old + 3*(pids-1);
        else    
            pids = ImuTimestamps(1:nPoseOld) - ImuTimestamps(1)+1;
            idx_v = 6*nIMUdata_old+3*nPts_old + 3*(pids-1);
        end

        for(id=1:size(pids,1))%2:pid_end)
            if(id > 1)
                pid = pids(id);
                idx = 6*(pid-2);
                %R0imu(:, :, id) = fnR5ABG(x_old(idx+1), x_old(idx+2), x_old(idx+3));
                %T0imu(:, id) = x_old((idx+4):(idx+6)); 
                A0imu = x_old.pose(pid-1).ang.val;
                R0imu(:, :, id) = fnR5ABG(A0imu(1), A0imu(2), A0imu(3));
                T0imu(:, id) = x_old.pose(pid-1).trans.val; 
            end
            if(InertialDelta_options.bUVonly == 0)
                %v0imu(:, id) = x_old((idx_v(id)+1):(idx_v(id)+3)); 
                v0imu(:, id) = x_old.velocity(id).xyz; 
            end
        end
    else
        fprintf('\n nPoseOld == %d? Unexpected!\n', nPoseOld);
        return;
    end    
    
    % Intial values of poses and IMU states
     if(InertialDelta_options.bIMUodo == 1)
        %% Obtain initial poses from IMU data
        [Rcam, ~, Tcam, vimu, Feature3D, RptFidSet, RptFeatureObs] = fnGetPoses5IMUdata_Inc(nPoseOld, ...
            nPoseNew, nPoses, R0imu, T0imu, v0imu, dtIMU, dp, dv, dphi, ...
            K, Feature3D, RptFidSet, RptFeatureObs, SLAM_Params );
        nPts = size(RptFidSet,1);
    else
        %% obtain relative poses from visual odometry
        [Rcam, ~, Tcam, Feature3D] = fnGetPoses5MatchedFeatures(nPoseNew, nPts, ...
                    K, fscaleGT, RptFeatureObs, kfids);  
     end
            
    ABGimu = zeros(3, nPoseNew);%nPoses+1);
    Timu = zeros(3, nPoseNew);%nPoses+1); % ABGimu = zeros(3, nPoses);            

    for(pid=(nPoseOld+1):nPoseNew)%(nPoses+1))% correspond to pose 1...n
        Rimu = SLAM_Params.Ru2c' * Rcam(:,:,pid) * SLAM_Params.Ru2c;
        [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
        Timu(:, pid) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, pid) - Rimu' * SLAM_Params.Tu2c;
    end

    %% Combine old and new poses into X        
    idend = 0;
    if(InertialDelta_options.bPreInt == 1)
        if(nPoseOld > 1)
            %idstart = idend + 1;                
            %idend = idend + 6*(nPoseOld-1); 
            %X(idstart:idend) = x_old(idstart:idend);
            for p = 1:nPoseOld-1
                X.pose(p).ang.val = x_old.pose(p).ang.val;
                X.pose(p).ang.col = (1:3) + xcol; xcol = xcol + 3;
                
                X.pose(p).trans.val = x_old.pose(p).trans.val;
                X.pose(p).trans.col = (1:3) + xcol; xcol = xcol + 3;
            end
        end
        %idstart  = idend + 1; 
        %idend = 6*(nPoseNew - 1);
        %tv = [ABGimu(:,(nPoseOld+1):end); Timu(:,(nPoseOld+1):end)];%
        %X(idstart:idend) = tv(:);
        for p = nPoseOld-1+1:nPoseNew-1
            X.pose(p).ang.val = ABGimu(:,p+1);
            X.pose(p).ang.col = (1:3) + xcol; xcol = xcol + 3;
            
            X.pose(p).trans.val = Timu(:,p+1);
            X.pose(p).trans.col = (1:3) + xcol; xcol = xcol + 3;
        end
    else % PreInt == 0
        if(nPoseOld > 1)
            idstart = idend + 1; 
            idend = idend + 6*nIMUdata_old; 
            x(idstart:idend) = x_old(idstart:idend);
            idend = idend - 6;
            idstart = idend + 1;
            idend = idend + 6;
            ABGimu0 = x_old(idstart:(idstart+2));
        else
            ABGimu0 = zeros(3,1);
        end
        idstart  = idend + 1; 
        idend = 6*nIMUdata; 
        [upos, uvel] = fnIMUnonPreIntPoses_Inc(nPoseNew, nPoseOld, ABGimu0, ...
            T0imu(:,nPoseOld), v0imu(:, nPoseOld), imufulldata, ... % Timu0, vimu0
            ImuTimestamps, nIMUrate, SLAM_Params );
        X(idstart:idend) = upos;
    end % if PreInt

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
        %if(InertialDelta_options.bPreInt == 1)
        %    idstart_old = 6*(nPoseOld-1)+1;
        %else                        
        %    idstart_old = 6*nIMUdata_old+1;
        %end    
        for(fidx=1:size(idrptf,1))
            %X((idstart_new+(idrptf(fidx)-1)*3):(idstart_new+(idrptf(fidx)-1)*3+2)) = x_old(...
            %    (idstart_old+(idrptf_old(fidx)-1)*3):(idstart_old+(idrptf_old(fidx)-1)*3+2));
            fid = idrptf(fidx);
            fid_old = idrptf_old(fidx);
            
            X.feature(fid).xyz = x_old.feature(fid_old).xyz;
            %X.feature(fid).xcol = (1:3) + xcol;
            %xcol = xcol + 3;
        end
    end

    %% Initial values of velocity
    if(InertialDelta_options.bUVonly == 0)
        
        if(nPoseOld > 1)
            if(InertialDelta_options.bPreInt == 1)
                %idstart = idend + 1;
                %idend = idend + 3*nPoseOld;
                %idend_old = 6*(nPoseOld-1) + 3*size(RptFidSet_old,1);
                %idstart_old = idend_old + 1;
                %idend_old = idend_old + 3*nPoseOld;
                %X(idstart:idend) = x_old(idstart_old:idend_old);
                for i=1:nPoseOld
                    X.velocity(i).xyz = x_old.velocity(i).xyz;
                    X.velocity(i).col = (1:3) + xcol;
                    xcol = xcol + 3;
                end
            else
                idstart = idend + 1;
                idend = idend + 3*(nIMUdata_old+1);
                idend_old = 6*nIMUdata_old + 3*size(RptFidSet_old,1);
                idstart_old = idend_old + 1;
                idend_old = idend_old + 3*(nIMUdata_old+1);
                X(idstart:idend) = x_old(idstart_old:idend_old);
            end
        end % if nPoseOld > 1
        
        [tv,xcol,~] = fnCalV5Kposes_Inc( nPoseNew, nPoseOld, ...
                        nPoses, nPts, nIMUdata, ImuTimestamps, nIMUrate, ...
                        X, xcol, dtIMU, dp, dv, SLAM_Params,imufulldata );
                    
        if(InertialDelta_options.bPreInt == 1)
            %idstart = idend + 1;
            %idend = idend + 3*nPoses;
            %if(nPoseOld == 1)
            %    idend = idend + 3;
            %end
            %X(idstart:idend) = tv;
            if (nPoseOld == 1)
                v_range = 1:nPoseNew;
            else
                v_range = nPoseOld+1:nPoseNew;
            end
            X.velocity(v_range) = tv;
        else
            idstart = idend + 1;
            if(nPoseOld == 1)
                idend = idend + 3*(nIMUdata+1);
            else
                idend = idend + 3*(nIMUdata - nIMUdata_old);
            end
            X(idstart:idend) = uvel;            
        end

        % Intial values of g 
        %idstart = idend + 1; idend = idend + 3;
        %X(idstart:idend) = g0;%[0,0,-9.8]';           
        X.g.val = SLAM_Params.g0;
        X.g.col = (1:3) + xcol;
        xcol = xcol + 3;
        
    end % if UVOnly == 0
    
    %% Au2c, Tu2c
    %idstart = idend + 1; idend = idend + 6;
    %X(idstart:idend) = [Au2c;Tu2c];
    X.Au2c.val = SLAM_Params.Au2c;
    X.Au2c.col = (1:3) + xcol ;    xcol = xcol + 3;
    
    X.Tu2c.val = SLAM_Params.Tu2c;
    X.Tu2c.col = (1:3) + xcol ;    xcol = xcol + 3;
    if(InertialDelta_options.bUVonly == 0)
        %% bf, bw
        if(InertialDelta_options.bVarBias == 0)
            %idstart = idend + 1; idend = idend + 6;
            %X(idstart:idend) = [bf0;bw0];%zeros(6,1);  
            X.Bf.val = SLAM_Params.bf0;
            X.Bf.col = (1:3) + xcol ;    xcol = xcol + 3;
            
            X.Bw.val = SLAM_Params.bw0;
            X.Bw.col = (1:3) + xcol ;    xcol = xcol + 3;
        else
            idstart = idend + 1; idend = idend + 6*(nPoseNew-1);                
            X(idstart:idend) = repmat([bf0;bw0],nPoseNew-1, 1);%zeros(6,1); 
        end
    end 
    %X = X(1:idend);
        