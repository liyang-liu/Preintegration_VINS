function [x,idend] = fnCalV5Kposes(nIMUdata, ImuTimestamps, nIMUrate, x, nPoses, dtIMU, idend, dp, dv, g0, bf0, imufulldata)
    global InertialDelta_options

    if(InertialDelta_options.bPreInt == 1)            
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
        dt = 1.0/nIMUrate;
        % The velocity of the first pose.IMUparking6L
        if(InertialDelta_options.bDinuka == 1)
            imufulldata = [imufulldata(:,1), imufulldata(:,5:7), imufulldata(:,2:4)];% ts, fb, wb
        end
        x(idstart:idend, 1) = (x(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imufulldata(ImuTimestamps(1), 2:4))'-bf0))/dt; 
        pidloopmax = nIMUdata;
        for(pid=2:pidloopmax)
          idstart = idend + 1;
          idend = idend + 3;
          Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
          x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dt...
              *dt*g0-Ri'*0.5*dt*dt*((imufulldata(ImuTimestamps(1)+pid-1, 2:4))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
        end
        % The velocity of the last pose.
        idstart = idend + 1;
        idend = idend + 3;
        Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
        x(idstart:idend) = x((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imufulldata(ImuTimestamps(1)+nIMUdata, 2:4))'-bf0);            
    end        