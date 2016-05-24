function [X_obj, xcol] = fn_LinearInterpPoses( nPoses, ABGimu, Timu, ImuTimestamps, X_obj, xcol )

    global PreIntegration_options
    
    %idend = 0;
    ai = 0; bi = 0; gi = 0;
    Ri = eye(3); 
    Ti = zeros(3,1);    
    end_pose = 0;
    
    for pid=2:nPoses % correspond to Pose 2...n
        ai1 = ABGimu(1, pid);
        bi1 = ABGimu(2, pid);
        gi1 = ABGimu(3, pid);
        
        Ri1 = fn_RFromABG( ai1, bi1, gi1 );
        Ti1 = Timu(:,pid);
        
        dR = Ri1 * Ri';
        [da, db, dg] = fn_ABGFromR( dR );        
        
        dT = Ti1 - Ti;
        
        nImu = ImuTimestamps(pid) - ImuTimestamps(pid-1);            
        
        ts = 1:nImu;
        
        ait = ai + da * ts / nImu;
        bit = bi + db * ts / nImu;
        git = gi + dg * ts / nImu;
        Tit = repmat(Ti,1,nImu) + repmat(dT,1,nImu) .* repmat(ts,3,1) / nImu;
        %idstart = idend + 1; 
        %idend = idend + nImu*6;         
        %tv = [ait; bit; git; Tit];
        %x(idstart:idend) = tv(:);            
         
        ang_vec = [ ait; bit; git ];
        ang_vec = ang_vec(:);
        trans_vec = Tit(:);
        
        %[ X_obj.pose( end_pose + 1 : end_pose + nImu ).ang.val ] =  { ang_vec };
        %[ X_obj.pose( end_pose + 1 : end_pose + nImu ).trans.val ] = { trans_vec };
        for i = 1 : nImu 
          X_obj.pose( end_pose + i ).ang.val = ang_vec( (i-1)*3 + 1 : (i-1)*3 + 3 );  
          X_obj.pose( end_pose + i ).ang.col = (1:3) + xcol;   xcol = xcol + 3;
          X_obj.pose( end_pose + i ).trans.val = trans_vec( (i-1)*3 + 1 : (i-1)*3 + 3 );  
          X_obj.pose( end_pose + i ).trans.col = (1:3) + xcol;   xcol = xcol + 3;
        end
    
        end_pose = end_pose + nImu;
        Ri = Ri1; Ti = Ti1;
        ai = ai1; bi = bi1; gi = gi1;            
    end    
    