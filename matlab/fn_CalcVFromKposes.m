function [x,idend] = fnCalcVFromKposes(nIMUdata, ImuTimestamps, nIMUrate, x, nPoses, dtIMU, idend, dp, dv, g0, bf0, imufulldata)
    global PreIntegration_options


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
