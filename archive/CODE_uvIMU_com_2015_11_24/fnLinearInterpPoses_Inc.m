function [x] = fnLinearInterpPoses_Inc(nPoseNew, nPoseOld, ABGimu, Timu, ...
                ImuTimestamps)
            
        nIMUdata = ImuTimestamps(nPoseNew) - ImuTimestamps(nPoseOld);
        x = zeros(6*nIMUdata, 1);
        idend = 0;
        pid = nPoseOld;
        ai = ABGimu(1,pid);
        bi = ABGimu(2,pid);
        gi = ABGimu(3,pid);
        Ri = fnR5ABG(ai, bi, gi);
        Ti = Timu(:,pid);
        for(pid=(nPoseOld+1):(nPoseNew))% correspond to Pose 1...n
            ai1 = ABGimu(1,pid);
            bi1 = ABGimu(2,pid);
            gi1 = ABGimu(3,pid);
            Ri1 = fnR5ABG(ai1, bi1, gi1);
            Ti1 = Timu(:,pid);
            dR = Ri1*Ri';
            [da, db, dg] = fnABG5R(dR);        
            dT = Ti1 - Ti; 
            %overallpid = pid + nPoseOld;
            nImu = ImuTimestamps(pid) - ImuTimestamps(pid-1);            
            ts = 1:nImu;
            ait = ai + da*ts/nImu;
            bit = bi + db*ts/nImu;
            git = gi + dg*ts/nImu;
            Tit = repmat(Ti,1,nImu) + repmat(dT,1,nImu).*repmat(ts,3,1)/nImu;
            idstart = idend + 1; 
            idend = idend + nImu*6; 
            tv = [ait;bit;git;Tit];
            x(idstart:idend) = tv(:);            
            Ri = Ri1; Ti = Ti1;
            ai = ai1; bi = bi1; gi = gi1;            
        end    