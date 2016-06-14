function [ X_obj, xcol ] = fnCalcVFromKposes( nIMUdata, ImuTimestamps, nIMUrate, dtIMU, nPoses, ...
                                                imufulldata, inertialDelta, SLAM_Params, X_obj, xcol )
    global PreIntegration_options
   
    X_obj.velocity(1).xyz = ( X_obj.pose(1).trans.xyz - 0.5 * dtIMU(2) * dtIMU(2) * SLAM_Params.g0 - inertialDelta.dp(:,2) ) / ( dtIMU(2) );
    X_obj.velocity(1).col = (1:3) + xcol;   xcol = xcol + 3;
    
    pidloopmax = nPoses - 1;
    for pid = 2 : pidloopmax
      %idstart = idend + 1;
      %idend = idend + 3;
        Ri = fn_RFromAngVec( X_obj.pose( pid - 1 ).ang.val );
        X_obj.velocity(pid).xyz =   ( ...
                                        X_obj.pose(pid).trans.xyz - X_obj.pose(pid-1).trans.xyz ...
                                        - 0.5 * ( dtIMU(pid+1) ^ 2 ) * SLAM_Params.g0 ...
                                        - Ri' * inertialDelta.dp(:,(pid+1)) ...
                                    )  / dtIMU(pid+1);
        X_obj.velocity(pid).col = (1:3) + xcol;     xcol = xcol + 3;
    end
    % The velocity of the last pose.
    Ri = fn_RFromAngVec( X_obj.pose( nPoses - 1 ).ang.val );
    X_obj.velocity(nPoses).xyz = X_obj.velocity(nPoses-1).xyz ...
                                + dtIMU(nPoses) * SLAM_Params.g0 ...
                                + Ri' * inertialDelta.dv(:,nPoses);
    X_obj.velocity(nPoses).col = (1:3) + xcol;  xcol = xcol + 3;
    