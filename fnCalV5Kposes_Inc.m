function [tv,idend] = fnCalV5Kposes_Inc(nPoseNew, nPoseOld, ...
    nPoses, nPts, nIMUdata, ImuTimestamps, nIMUrate, bDinuka, ...
    bPreInt, x, dtIMU, dp, dv, g0, bf0, imufulldata)    

    idend = 0;
    tv = zeros(3*(nIMUdata+1), 1);
    if(bPreInt == 1)   
        if(nPoseOld == 1)
            % The velocity of the first pose.
            idstart = idend + 1;
            idend = idend + 3;
            tv(idstart:idend, 1) = (x(4:6,1)-0.5*dtIMU(2)*dtIMU(2)*g0-dp(:,2))/(dtIMU(2)); 
%             pid_start = nPoseOld+2;
        end
        pid_start = nPoseOld+1;           
        pid_end = nPoseNew-1;
        for(pid=pid_start:pid_end)
          idstart = idend + 1;
          idend = idend + 3;
          Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
          tv(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dtIMU(pid+1)...
              *dtIMU(pid+1)*g0-Ri'*dp(:,(pid+1)))/(dtIMU(pid+1));%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
        end
        % The velocity of the last pose.
        idstart = idend + 1;
        idend = idend + 3;
        Ri = fnR5ABG(x(6*(nPoseNew-2)+1),x(6*(nPoseNew-2)+2),x(6*(nPoseNew-2)+3));
        if(idstart > 3)
            tv(idstart:idend) = tv((idstart-3):(idend-3), 1)+dtIMU(nPoseNew)*g0...
                +Ri'*dv(:,nPoseNew);             
        else
           idx = 6*(nPoseNew-1)+3*nPts+3*(nPoseNew-2);
           tv(idstart:idend) = x((idx+1):(idx+3), 1)+dtIMU(nPoseNew)*g0...
                +Ri'*dv(:,nPoseNew);    
        end
%             idp1s = (nPoses-1)*6+nPts*3+1;
%             x((idp1s):(idp1s+2)) = x((idp1s+3):(idp1s+2+3));            
    else
        %idend = idend + 3*nIMUdata;%(nPoses-1)*nIMUrate
        dt = 1.0/nIMUrate;
        % The velocity of the first pose.IMUparking6L
        if(bDinuka == 1)
            imufulldata = [imufulldata(:,1), imufulldata(:,5:7), imufulldata(:,2:4)];% ts, fb, wb
        end
        if(nPoseOld == 1)
            idstart = idend + 1;
            idend = idend + 3; 
            tv(idstart:idend, 1) = (x(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imufulldata(ImuTimestamps(1), 2:4))'-bf0))/dt; 
%             pid_start = nIMUdata_Old+2;
        end
        nIMUdata_Old = ImuTimestamps(nPoseOld) - ImuTimestamps(1)+1;
        pid_start = nIMUdata_Old+1;
        pid_end = nIMUdata;
        for(pid=pid_start:pid_end)
          idstart = idend + 1;
          idend = idend + 3;
          Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
          tv(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dt...
              *dt*g0-Ri'*0.5*dt*dt*((imufulldata(ImuTimestamps(1)+pid, 2:4))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
        end
        % The velocity of the last pose.
        idstart = idend + 1;
        idend = idend + 3;
        Ri = fnR5ABG(x(6*(nIMUdata-2)+1),x(6*(nIMUdata-2)+2),x(6*(nIMUdata-2)+3));
        tv(idstart:idend) = tv((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imufulldata(ImuTimestamps(1)+nIMUdata, 2:4))'-bf0);            
    end       
    tv = tv(1:idend);