function [x] = fnLinearInterpPoses(nPoses, ABGimu, Timu, ImuTimestamps,x)

        idend = 0;
        ai = 0; bi = 0; gi = 0;
        Ri = eye(3); 
        Ti = zeros(3,1);
        for(pid=2:(nPoses))% correspond to Pose 2...n
            ai1 = ABGimu(1,pid);
            bi1 = ABGimu(2,pid);
            gi1 = ABGimu(3,pid);
            Ri1 = fnR5ABG(ai1, bi1, gi1);
            Ti1 = Timu(:,pid);
            dR = Ri1*Ri';
            [da, db, dg] = fnABG5R(dR);        
            dT = Ti1 - Ti; 
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