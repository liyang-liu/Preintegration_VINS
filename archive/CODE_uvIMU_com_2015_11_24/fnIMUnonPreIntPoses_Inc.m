function [upos, uvel] = fnIMUnonPreIntPoses_Inc(nPoseNew, nPoseOld, ABGimu0, Timu0, vimu0, ...
                imufulldata, ImuTimestamps, nIMUrate, bf0, bw0, g0)
            
        nIMUdata_delta = ImuTimestamps(nPoseNew) - ImuTimestamps(nPoseOld);
        nIMUdata_old = ImuTimestamps(nPoseOld) - ImuTimestamps(1);

        Timu = zeros(3, nIMUdata_delta+1);
        vimu = zeros(3, nIMUdata_delta+1);
        Rimu = zeros(3, 3, nIMUdata_delta+1);
        phiimu = zeros(3, nIMUdata_delta+1);
        upos = zeros(6*nIMUdata_delta, 1);
        uvel = zeros(3*nIMUdata_delta, 1);
        uposidend = 0;
        uvelidend = 0;
        pid = nPoseOld;
        ai = ABGimu0(1);
        bi = ABGimu0(2);
        gi = ABGimu0(3);
        Rimu(:,:,1) = fnR5ABG(ai, bi, gi);
        Timu(:, 1) = Timu0;
        vimu(:, 1) = vimu0;
        phiimu(:, 1) = [ai; bi; gi];
        dt = 1.0/nIMUrate;
        nStartIMUdataId = ImuTimestamps(nPoseOld)-1;

        for(pid=2:(nIMUdata_delta+1))% correspond to Pose u(o+1)...u(n)
           fn = (Rimu(:,:,pid-1))'*((imufulldata(nStartIMUdataId+pid-1, 5:7))' - bf0);
           vimu(:, pid) = vimu(:, pid-1) + dt*(g0 + fn);
           Timu(:, pid) = Timu(:, pid-1) + 0.5*dt*(vimu(:, pid-1)+...
               vimu(:, pid));  
           Eb0 = Jac_koInv(phiimu(:, pid-1));
           phiimu(:, pid) = phiimu(:, pid-1) + dt*Eb0*...
               ((imufulldata(nStartIMUdataId+pid-1, 2:4))' - bw0);
           Rimu(:,:,pid) = fnR5ABG(phiimu(1, pid), phiimu(2, pid), ...
               phiimu(3, pid));
           upidstart = uposidend + 1;
           uposidend = uposidend + 6;
           upos(upidstart:uposidend) = [phiimu(1, pid); phiimu(2, pid); ...
               phiimu(3, pid); Timu(:, pid)];
           uvelidstart = uvelidend + 1;
           uvelidend = uvelidend + 3;    
           uvel(uvelidstart:uvelidend) = vimu(:, pid);
        end 
        
        if(nPoseOld == 1)
            uvel = [vimu0; uvel];
        end       
