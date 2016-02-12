function [x, RptFidSet, RptFeatureObs, nPts] = fnCompX5odometry( ...
                    nPoseOld, nPoseNew, nPoses, nPts, x_old, ...
                    ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
                    RptFidSet_old, dtIMU, g0, dp, dv, dphi, K, RptFeatureObs, ...
                    Tu2c, Au2c, Ru2c, fscaleGT, kfids, nIMUrate, ...
                    x, bf0, bw0, imufulldata)
   
    global InertialDelta_options

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
                R0imu(:, :, id) = fnR5ABG(x_old(idx+1), x_old(idx+2), x_old(idx+3));
                T0imu(:, id) = x_old((idx+4):(idx+6)); 
            end
            if(InertialDelta_options.bUVonly == 0)
                v0imu(:, id) = x_old((idx_v(id)+1):(idx_v(id)+3)); 
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
            nPoseNew, nPoses, R0imu, T0imu, v0imu, dtIMU, g0, dp, dv, dphi, ...
            K, Feature3D, RptFidSet, RptFeatureObs, Tu2c, Ru2c);
        nPts = size(RptFidSet,1);
    else
        %% obtain relative poses from visual odometry
        [Rcam, ~, Tcam, Feature3D] = fnGetPoses5MatchedFeatures(nPoseNew, nPts, ...
                    K, fscaleGT, RptFeatureObs, kfids);  
     end
            
    ABGimu = zeros(3, nPoseNew);%nPoses+1);
    Timu = zeros(3, nPoseNew);%nPoses+1); % ABGimu = zeros(3, nPoses);            

    for(pid=(nPoseOld+1):nPoseNew)%(nPoses+1))% correspond to pose 1...n
        Rimu = Ru2c'*Rcam(:,:,pid)*Ru2c;
        [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
        Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
    end

    %% Combine old and new poses into x        
    idend = 0; 
    if(InertialDelta_options.bPreInt == 1)
        if(nPoseOld > 1)
            idstart = idend + 1;                
            idend = idend + 6*(nPoseOld-1); 
            x(idstart:idend) = x_old(idstart:idend); 
        end
        idstart  = idend + 1; 
        idend = 6*(nPoseNew - 1);
        tv = [ABGimu(:,(nPoseOld+1):end); Timu(:,(nPoseOld+1):end)];%
        x(idstart:idend) = tv(:);
    else
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
            ImuTimestamps, nIMUrate, bf0, bw0, g0);
        x(idstart:idend) = upos;
    end 

    % Intial values of features at the initial IMU pose
    % Given Pf1c, Pf1u can be calculated as:
    % Pf1u = Ru2c'*Pf1c + Tu2c
    idstart = idend + 1; 
    idstart_new = idstart;
    idend = idend + 3*nPts;
    tv =  (Feature3D(RptFidSet, 5:7))';% select the first group      
    Pf1u = Ru2c'*tv + repmat(Tu2c, 1, nPts);                
    x(idstart:idend) = Pf1u(:);
    if(nPoseOld > 1)
        [~, idrptf, idrptf_old] = intersect(RptFidSet, RptFidSet_old);
        if(InertialDelta_options.bPreInt == 1)
            idstart_old = 6*(nPoseOld-1)+1;
        else                        
            idstart_old = 6*nIMUdata_old+1;
        end    
        for(fid=1:size(idrptf,1))
            x((idstart_new+(idrptf(fid)-1)*3):(idstart_new+(idrptf(fid)-1)*3+2)) = x_old(...
                (idstart_old+(idrptf_old(fid)-1)*3):(idstart_old+(idrptf_old(fid)-1)*3+2));
        end
    end

    %% Initial values of velocity
    if(InertialDelta_options.bUVonly == 0)
%             [x,idend] = fnCalV5Kposes(nIMUdata, ImuTimestamps, nIMUrate, ...
%                 x, nPoseNew, dtIMU, idend, dp, dv, g0, bf0, imufulldata);
        if(nPoseOld > 1)
            if(InertialDelta_options.bPreInt == 1)
                idstart = idend + 1;
                idend = idend + 3*nPoseOld;
                idend_old = 6*(nPoseOld-1) + 3*size(RptFidSet_old,1);
                idstart_old = idend_old + 1;
                idend_old = idend_old + 3*nPoseOld;
                x(idstart:idend) = x_old(idstart_old:idend_old);
            else
                idstart = idend + 1;
                idend = idend + 3*(nIMUdata_old+1);
                idend_old = 6*nIMUdata_old + 3*size(RptFidSet_old,1);
                idstart_old = idend_old + 1;
                idend_old = idend_old + 3*(nIMUdata_old+1);
                x(idstart:idend) = x_old(idstart_old:idend_old);
            end
        end
        [tv,~] = fnCalV5Kposes_Inc(nPoseNew, nPoseOld, ...
                        nPoses, nPts, nIMUdata, ImuTimestamps, nIMUrate, ...
                        x, dtIMU, dp, dv, g0, bf0, imufulldata);
        if(InertialDelta_options.bPreInt == 1)
            idstart = idend + 1;
            idend = idend + 3*nPoses;
            if(nPoseOld == 1)
                idend = idend + 3;
            end
            x(idstart:idend) = tv;
        else
            idstart = idend + 1;
            if(nPoseOld == 1)
                idend = idend + 3*(nIMUdata+1);
            else
                idend = idend + 3*(nIMUdata - nIMUdata_old);
            end
            x(idstart:idend) = uvel;            
        end

        % Intial values of g 
        idstart = idend + 1; idend = idend + 3;
        x(idstart:idend) = g0;%[0,0,-9.8]';           
    end
    %% Au2c, Tu2c
    idstart = idend + 1; idend = idend + 6;
    x(idstart:idend) = [Au2c;Tu2c];

    if(InertialDelta_options.bUVonly == 0)
        %% bf, bw
        if(InertialDelta_options.bVarBias == 0)
            idstart = idend + 1; idend = idend + 6;
            x(idstart:idend) = [bf0;bw0];%zeros(6,1);  
        else
            idstart = idend + 1; idend = idend + 6*(nPoseNew-1);                
            x(idstart:idend) = repmat([bf0;bw0],nPoseNew-1, 1);%zeros(6,1); 
        end
    end 
    x = x(1:idend);
        