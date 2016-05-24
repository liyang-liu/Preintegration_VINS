function [X_obj, xcol] = fn_InitVelocity(nPoses, X_obj, xcol, dp, dv, dtIMU, imuData_cell, nIMUdata, nIMUrate, dt, SLAM_Params )

   global PreIntegration_options
 
    if(PreIntegration_options.bPreInt == 1)            
        
        %% The velocity of the first pose.
        X_obj.velocity(1).xyz = ...
            ( X_obj.pose(1).trans.val - 0.5 * dtIMU(2) * dtIMU(2) * SLAM_Params.g0  - dp(:,2) ) ...
            /  ( dtIMU(2) ); 
        X_obj.velocity(1).col = (1:3) + xcol;   xcol = xcol + 3;    

        %% velocity from pose 2 to 2nd last pose
        pidloopmax = nPoses - 1;            
        for(pid=2:pidloopmax)
            Ri = fn_RFromAngVec( X_obj.pose(pid-1).ang.val );
            X_obj.velocity(pid).xyz = ...
                ( X_obj.pose(pid).trans.val - X_obj.pose(pid-1).trans.val ...
                - 0.5 * dtIMU(pid+1) * dtIMU(pid+1) * SLAM_Params.g0 ...
                - Ri' * dp(:,(pid+1)) ) / ( dtIMU(pid+1) ) ;
            X_obj.velocity(pid).col = (1:3) + xcol;  xcol = xcol + 3;    
        end
        
        %% The velocity of the last pose.
        Ri = fn_RFromAngVec( X_obj.pose(nPoses-1).ang.val ); 
        X_obj.velocity(nPoses).xyz = X_obj.velocity(nPoses-1).xyz + dtIMU(nPoses) * SLAM_Params.g0 + Ri' * dv(:, nPoses);
        X_obj.velocity(nPoses).col = (1:3) + xcol;     xcol = xcol + 3;     
        
    else
            %idend = idend + 3*nIMUdata;%(nPoses-1)*nIMUrate
            %idstart = idend + 1;
            %idend = idend + 3; 
            
            %% The velocity of the first pose.IMUparking6L
            %x(idstart:idend, 1) = (x(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imuData_cell{2}.samples(1, 5:7))'-bf0))/dt; 
            X_obj.velocity(1).xyz = ( X_obj.pose(1).trans.val ...
                                - 0.5 * dt * dt * SLAM_Params.g0 ...
                                - 0.5 * dt * dt * ( (imuData_cell{2}.samples(1, 5:7) )' - SLAM_Params.bf0 ) ...
                             ) / dt;
            X_obj.velocity(1).col = (1:3) + xcol; xcol = xcol + 3;
            %% velocity from pose 2 to 2nd last pose
            pidloopmax = nIMUdata;
            for pid = 1 : (pidloopmax - 1)
                %idstart = idend + 1;
                %idend = idend + 3;
                remains = rem( pid, nIMUrate );
                cid = 2 + (pid - remains) / nIMUrate;

                Ri = fn_RFromAngVec( X_obj.pose(pid).ang.val );
                X_obj.velocity( pid + 1 ).xyz = ( ...
                                    X_obj.pose( pid + 1 ).trans.val - X_obj.pose( pid ).trans.val ...
                                    - 0.5 * dt * dt * SLAM_Params.g0 ...
                                    - Ri' * 0.5 * dt * dt * ( (imuData_cell{cid}.samples( remains + 1, 5:7))' - SLAM_Params.bf0 ) ...
                                 ) / dt;
                X_obj.velocity( pid + 1 ).col = (1:3) + xcol; xcol = xcol + 3;
                %x(idstart:idend, 1) = (x((6*(pid)+4):(6*(pid)+6), 1) - x((6*(pid-1)+4):(6*(pid-1)+6), 1)-0.5*dt...
                %  *dt*g0-Ri'*0.5*dt*dt*((imuData_cell{cid}.samples(remains+1, 5:7))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
            end
            
            %% The velocity of the last pose.
            %idstart = idend + 1;
            %idend = idend + 3;
            %Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
            Ri = fn_RFromAngVec( X_obj.pose(end).ang.val );
            %x(idstart:idend) = x((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imuData_cell{nPoses}.samples(remains+1, 5:7))'-bf0);            
            X_obj.velocity( nIMUdata + 1 ).xyz = X_obj.velocity(nIMUdata).xyz + dt * SLAM_Params.g0 ...
                                        + Ri' * dt * (( imuData_cell{nPoses}.samples( remains+1, 5:7 ))' - SLAM_Params.bf0 );
            X_obj.velocity( nIMUdata + 1 ).col = (1:3) + xcol; xcol = xcol + 3;
    end 
        