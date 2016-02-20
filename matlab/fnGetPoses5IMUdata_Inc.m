function [Rcam, Acam, Tcam, vimu, Feature3D, RptFidSet, RptFeatureObs] = fnGetPoses5IMUdata_Inc(nPoseOld, ...
                nPoseNew, nPoses, R0imu, T0imu, v0imu, dtIMU, dp, dv, dphi, ...
                K, Feature3D, RptFidSet, RptFeatureObs, SLAM_Params)
            
    global InertialDelta_options
    

    Timu = zeros(3,nPoses);
    vimu = zeros(3,nPoses);    
    Rimu = zeros(3,3,nPoses);

    Tcam = zeros(3,nPoses);
    vcam = zeros(3,nPoses);
    Acam = zeros(3,nPoses);
    Rcam = zeros(3,3,nPoses);
    Timu = [T0imu, Timu];
    vimu = [v0imu, vimu];
    Rimu = cat(3, R0imu, Rimu);
    
    for(pid=1:nPoseOld)
        Tcam(:, pid) = SLAM_Params.Ru2c * (T0imu(:,pid) - SLAM_Params.Tu2c + (R0imu(:,:,pid))' * SLAM_Params.Tu2c);
        Rcam(:,:,pid) = SLAM_Params.Ru2c * R0imu(:,:,pid) * SLAM_Params.Ru2c';
        [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = fnABG5R(Rcam(:,:,pid));  
    end
    
    for(pid = (nPoseOld+1):nPoseNew)%2:(nPoses+1))
        
       vimu(:, pid) = vimu(:, pid-1) + dtIMU(pid) * SLAM_Params.g0 + ...
           (Rimu(:,:,pid-1))' * dv(:,pid);
       Timu(:, pid) = Timu(:, pid-1) + dtIMU(pid)*vimu(:, pid-1)...// (+vimu(:, pid))
          + 0.5 * dtIMU(pid) * dtIMU(pid) * SLAM_Params.g0 + ...
           (Rimu(:,:,pid-1))' * dp(:, pid);       
       dR = fnR5ABG(dphi(1, pid), dphi(2, pid), dphi(3, pid));
       Rimu(:,:,pid) = dR * Rimu(:,:,pid-1);

       Tcam(:, pid) = SLAM_Params.Ru2c * (Timu(:, pid) - SLAM_Params.Tu2c + (Rimu(:,:,pid))' * SLAM_Params.Tu2c);
       Rcam(:,:,pid) = SLAM_Params.Ru2c * Rimu(:,:,pid) * SLAM_Params.Ru2c';
       [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = fnABG5R(Rcam(:,:,pid));
       
    end

    fId_FeatureObs = 1; nTrianguTimes = 2;
    smallAngleSet = [];
    nRptfs = size(RptFeatureObs, 1);
    
    % Select poses to triangulate features: distant frames, maximal changes
    % in UVs
    for(fidx=1:nRptfs)
       nObs = 1;
       if(RptFeatureObs(fidx).obsv(nObs).pid >= nPoseNew)
           continue;
       end
       
       while( ( RptFeatureObs(fidx).obsv(nObs).pid < nPoseNew ) && ...
               ( nObs < RptFeatureObs(fidx).nObs ) )
           nObs = nObs + 1;
       end

       tv = RptFeatureObs(fidx).obsv; 

       %%%%%
       pids = [tv.pid];
       fpdis = Tcam(:, pids) - repmat(Tcam(:, pids(1)), 1, nObs);
       fpdis = complex(fpdis(1,:), fpdis(2,:));
       fpdis = abs(fpdis);
       [~, idx] = sort(fpdis);
       idx = idx(end);

       comid = RptFeatureObs(fidx).fid;
       %%%

       pid1 = tv(1).pid;%1;%
       pid2 = tv(idx).pid;%2;%
       %%%

       comfeatures = [tv(1).uv, tv(idx).uv]';
       [p3d] = fnTrianguFeatures(K, Rcam(:, :, pid1), Tcam(:, pid1), ...
                            Rcam(:, :, pid2), Tcam(:, pid2), comfeatures);

        p1f = p3d' - Tcam(:, pid1); p2f = p3d' - Tcam(:, pid2);
        fangle = 180*acos(p1f'*p2f/(norm(p1f)*norm(p2f)))/pi;

        if(fangle < 1)%0.5)%1.5)%
           fprintf('P%d-P%d: Angle(%d) = %f!\n', pid1, pid2, comid, fangle); 
           smallAngleSet = [smallAngleSet, fidx];
           continue;
        end           
       
        Feature3D(comid).numTriangs = Feature3D(comid).numTriangs + 1;
        ntrigtms = Feature3D(comid).numTriangs;
        Feature3D(comid).triangs(ntrigtms) = ...
                struct(  'pid1', pid1, ...
                         'pid2', pid2, ...
                         'p3D',  -p3d' ...
                );
    end 
    
    fprintf('\n%d/%d to be removed. \n', ...
        size(smallAngleSet, 2), size(RptFidSet,1));
    
    RptFidSet(smallAngleSet) = [];
    RptFeatureObs(smallAngleSet) = [];
        
end
                