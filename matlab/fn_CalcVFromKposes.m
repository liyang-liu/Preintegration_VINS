function [ X_obj, xcol ] = fnCalcVFromKposes( nIMUdata, ImuTimestamps, nIMUrate, dtIMU, nPoses, ...
                                                imufulldata, dp, dv, SLAM_Params, X_obj, xcol )
    global PreIntegration_options

    global PreIntegration_options

   

    %idstart = idend + 1;
    %idend = idend + 3; 
    % The velocity of the first pose.
    %x(idstart:idend, 1) = (x(4:6,1)-0.5*dtIMU(2)*dtIMU(2)*g0-dp(:,2))/(dtIMU(2));
    X_obj.velocity(1).xyz = ( X_obj.pose(1).trans.xyz - 0.5 * dtIMU(2) * dtIMU(2) * SLAM_Params.g0 - dp(:,2) ) / ( dtIMU(2) );
    X_obj.velocity(1).col = (1:3) + xcol;   xcol = xcol + 3;
    
    pidloopmax = nPoses - 1;
    for pid = 2 : pidloopmax
      %idstart = idend + 1;
      %idend = idend + 3;
      if 1 
        Ri = fn_RFromAngVec( X_obj.pose( pid - 1 ).ang.val );
        X_obj.velocity(pid).xyz =   ( ...
                                        X_obj.pose(pid).trans.xyz - X_obj.pose(pid-1).trans.xyz ...
                                        - 0.5 * ( dtIMU(pid+1) ^ 2 ) * SLAM_Params.g0 ...
                                        - Ri' * dp(:,(pid+1)) ...
                                    )  / dtIMU(pid+1);
        X_obj.velocity(pid).col = (1:3) + xcol;     xcol = xcol + 3;
      else
          Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
          x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dtIMU(pid+1)...
              *dtIMU(pid+1)*g0-Ri'*dp(:,(pid+1)))/(dtIMU(pid+1));%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
      end
    end
    % The velocity of the last pose.
    %idstart = idend + 1;
    %idend = idend + 3;
    if 1
        Ri = fn_RFromAngVec( X_obj.pose( nPoses - 1 ).ang.val );
        X_obj.velocity(nPoses).xyz = X_obj.velocity(nPoses-1).xyz ...
                                    + dtIMU(nPoses) * SLAM_Params.g0 ...
                                    + Ri' * dv(:,nPoses);
        X_obj.velocity(nPoses).col = (1:3) + xcol;  xcol = xcol + 3;
    else
        Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
        x(idstart:idend) = x((idstart-3):(idend-3), 1)+dtIMU(nPoses)*g0+Ri'*dv(:,nPoses);             
    end
    