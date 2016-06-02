function [x] = fnInitVelocity_Inc(nPoseNew, nPoseOld, nPoses, nPts, bPreInt,...
            g0, bf0, dp, dv, dtIMU, imuData_cell, nIMUrate, dt, x0)

        idend = 0;
%         nPoses = nPoseNew - nPoseOld;
        x = zeros(nPoses*nIMUrate, 1);
        if(bPreInt == 1)
            % nPoseOld + 1 
            if(nPoseOld == 1)
                % The velocity of the first pose.            
                idstart = idend + 1;
                idend = idend + 3;
                x(idstart:idend, 1) = (x0(4:6,1)-0.5*dtIMU(2)*dtIMU(2)*g0-dp(:,2))/(dtIMU(2)); 
                pid_start = nPoseOld+1; 
            else
                pid_start = nPoseOld+1;
            end
%             if((nPoseNew == nAllposes) || (nPoseNew == 2))
                pid_end = nPoseNew-1;
%             else
%                 pid_end = nPoseNew;
%             end
            % (nPoseOld+2):(nPoseNew-1)
            for(pid=pid_start:pid_end)
                idstart = idend + 1;
                idend = idend + 3;
                Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
                x(idstart:idend, 1) = (x0((6*(pid-1)+4):(6*(pid-1)+6), 1) - x0((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dtIMU(pid+1)...
                  *dtIMU(pid+1)*g0-Ri'*dp(:,(pid+1)))/(dtIMU(pid+1));%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
            end
            % nPoseNew  
%             if((nPoseNew == nAllposes) || (nPoseNew == 2))
                % The velocity of the last pose.            
                idstart = idend + 1;
                idend = idend + 3;
                Ri = fnR5ABG(x0(6*(nPoseNew-2)+1),x0(6*(nPoseNew-2)+2),x0(6*(nPoseNew-2)+3));
%                 x(idstart:idend) = x((idstart-3):(idend-3), 1)+dtIMU(nPoseNew)*g0+Ri'*dv(:,nPoseNew); 
                if(idstart > 3)
                    x(idstart:idend) = x((idstart-3):(idend-3), 1)+dtIMU(nPoseNew)*g0...
                        +Ri'*dv(:,nPoseNew);             
                else
                   idx = 6*(nPoseNew-1)+3*nPts+3*(nPoseNew-2);
                   x(idstart:idend) = x((idx+1):(idx+3), 1)+dtIMU(nPoseNew)*g0...
                        +Ri'*dv(:,nPoseNew);    
                end
%             end          
        else 
            if(nPoseOld == 1)
                % The velocity of the first pose. IMUparking6L
                idstart = idend + 1;
                idend = idend + 3;
                x(idstart:idend, 1) = (x0(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imuData_cell{2}.samples(1, 5:7))'-bf0))/dt;
            end
            pid_start  = 1;
%             if((nPoseNew == nAllposes) || (nPoseNew == 2))
                pid_end = nIMUrate*nPoses - 1;
%             else
%                 pid_end = nIMUrate*nPoses;
%             end
%             pidloopmax = nIMUdata;
            for(pid=pid_start:pid_end)
              idstart = idend + 1;
              idend = idend + 3;
              remainds = rem(pid, nIMUrate);
              cid = 2 + (pid-remainds)/nIMUrate;
              if(remainds == 99)
                 tt = 1; 
              end
              Ri = fnR5ABG(x0(6*(pid-1)+1),x0(6*(pid-1)+2),x0(6*(pid-1)+3));
              x(idstart:idend, 1) = (x0((6*(pid)+4):(6*(pid)+6), 1) - x0((6*(pid-1)+4):(6*(pid-1)+6), 1)-0.5*dt...
                  *dt*g0-Ri'*0.5*dt*dt*((imuData_cell{cid}.samples(remainds+1, 5:7))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
            end
%             if((nPoseNew == nAllposes) || (nPoseNew == 2))
                % The velocity of the last pose.
                idstart = idend + 1;
                idend = idend + 3;
                Ri = fnR5ABG(x0(6*(nPoses-2)+1),x0(6*(nPoses-2)+2),x0(6*(nPoses-2)+3));
                x(idstart:idend) = x((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imuData_cell{nPoses}.samples(remainds+1, 5:7))'-bf0);  
%             end
        end 
        x= x(1:idend);