function [x, idend] = fnInitVelocity(nPoses, bPreInt, x, idend, g0, bf0, dp, dv, dtIMU, imuData_cell, nIMUdata, nIMUrate, dt)

        if(bPreInt == 1)            
            idstart = idend + 1;
            idend = idend + 3; 
            % The velocity of the first pose.
            x(idstart:idend, 1) = (x(4:6,1)-0.5*dtIMU(2)*dtIMU(2)*g0-dp(:,2))/(dtIMU(2));               
            pidloopmax = nPoses - 1;
            for(pid=2:pidloopmax)
              idstart = idend + 1;
              idend = idend + 3;
              Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
              x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dtIMU(pid+1)...
                  *dtIMU(pid+1)*g0-Ri'*dp(:,(pid+1)))/(dtIMU(pid+1));%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
            end
            % The velocity of the last pose.
            idstart = idend + 1;
            idend = idend + 3;
            Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
            x(idstart:idend) = x((idstart-3):(idend-3), 1)+dtIMU(nPoses)*g0+Ri'*dv(:,nPoses);             
%             idp1s = (nPoses-1)*6+nPts*3+1;
%             x((idp1s):(idp1s+2)) = x((idp1s+3):(idp1s+2+3));            
        else
            %idend = idend + 3*nIMUdata;%(nPoses-1)*nIMUrate
            idstart = idend + 1;
            idend = idend + 3; 
            % The velocity of the first pose.IMUparking6L
            x(idstart:idend, 1) = (x(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imuData_cell{2}.samples(1, 5:7))'-bf0))/dt; 
            pidloopmax = nIMUdata;
            for(pid=1:(pidloopmax-1))
              idstart = idend + 1;
              idend = idend + 3;
              remainds = rem(pid, nIMUrate);
              cid = 2 + (pid-remainds)/nIMUrate;
              if(remainds == 99)
                 tt = 1; 
              end
              Ri = fnR5ABG(x(6*(pid-1)+1),x(6*(pid-1)+2),x(6*(pid-1)+3));
              x(idstart:idend, 1) = (x((6*(pid)+4):(6*(pid)+6), 1) - x((6*(pid-1)+4):(6*(pid-1)+6), 1)-0.5*dt...
                  *dt*g0-Ri'*0.5*dt*dt*((imuData_cell{cid}.samples(remainds+1, 5:7))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
            end
            % The velocity of the last pose.
            idstart = idend + 1;
            idend = idend + 3;
            Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
            x(idstart:idend) = x((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imuData_cell{nPoses}.samples(remainds+1, 5:7))'-bf0);            
        end 