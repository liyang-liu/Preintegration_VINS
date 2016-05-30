function [X_obj, xcol] = fn_InitVelocity( nFrames, X_obj, xcol, dp, dv, dtIMU, imuData_cell, nIMUdata, nIMUrate, dt, SLAM_Params )

   global PreIntegration_options
 
    if(PreIntegration_options.bPreInt == 1)            
        
        %% The velocity of the first pose.
        X_obj.velocity(1).xyz = ...
            ( X_obj.pose(1).trans.xyz - 0.5 * dtIMU(2) * dtIMU(2) * SLAM_Params.g0  - dp(:,2) ) ...
            /  ( dtIMU(2) ); 
        X_obj.velocity(1).col = (1:3) + xcol;   xcol = xcol + 3;    

        %% velocity from pose 2 to 2nd last pose
        pidloopmax = nFrames - 1;            
        for(pid=2:pidloopmax)
            Ri = fn_RFromAngVec( X_obj.pose(pid-1).ang.val );
            X_obj.velocity(pid).xyz = ...
                ( X_obj.pose(pid).trans.xyz - X_obj.pose(pid-1).trans.xyz ...
                - 0.5 * dtIMU(pid+1) * dtIMU(pid+1) * SLAM_Params.g0 ...
                - Ri' * dp(:,(pid+1)) ) / ( dtIMU(pid+1) ) ;
            X_obj.velocity(pid).col = (1:3) + xcol;  xcol = xcol + 3;    
        end
        
        %% The velocity of the last pose.
        Ri = fn_RFromAngVec( X_obj.pose(nFrames-1).ang.val ); 
        X_obj.velocity(nFrames).xyz = X_obj.velocity(nFrames-1).xyz + dtIMU(nFrames) * SLAM_Params.g0 + Ri' * dv(:, nFrames);
        X_obj.velocity(nFrames).col = (1:3) + xcol;     xcol = xcol + 3;     
        
    else
            
            %% The velocity of the first pose.IMUparking6L
            X_obj.velocity(1).xyz = ( X_obj.pose(1).trans.xyz ...
                                        - 0.5 * dt * dt * SLAM_Params.g0 ...
                                        - 0.5 * dt * dt * ( (imuData_cell{2}.samples(1, 5:7) )' - SLAM_Params.bf0 ) ...
                                     ) / dt;
            X_obj.velocity(1).col = (1:3) + xcol; xcol = xcol + 3;
            %% velocity from pose 2 to 2nd last pose
            pidloopmax = nIMUdata;
            for pid = 1 : (pidloopmax - 1)
                remains = rem( pid, nIMUrate );
                cid = 2 + (pid - remains) / nIMUrate;

                Ri = fn_RFromAngVec( X_obj.pose(pid).ang.val );
                X_obj.velocity( pid + 1 ).xyz = ( ...
                                    X_obj.pose( pid + 1 ).trans.xyz - X_obj.pose( pid ).trans.xyz ...
                                    - 0.5 * dt * dt * SLAM_Params.g0 ...
                                    - Ri' * 0.5 * dt * dt * ( (imuData_cell{cid}.samples( remains + 1, 5:7))' - SLAM_Params.bf0 ) ...
                                 ) / dt;
                X_obj.velocity( pid + 1 ).col = (1:3) + xcol; xcol = xcol + 3;
            end
            
            %% The velocity of the last pose.
            Ri = fn_RFromAngVec( X_obj.pose(end).ang.val );
            X_obj.velocity( nIMUdata + 1 ).xyz = X_obj.velocity(nIMUdata).xyz + dt * SLAM_Params.g0 ...
                                        + Ri' * dt * (( imuData_cell{nFrames}.samples( remains + 1, 5:7 ))' - SLAM_Params.bf0 );
            X_obj.velocity( nIMUdata + 1 ).col = (1:3) + xcol; xcol = xcol + 3;
    end 
        