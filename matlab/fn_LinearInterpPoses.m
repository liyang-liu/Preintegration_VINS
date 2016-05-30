function [X_obj, xcol] = fn_LinearInterpPoses( nFrames, ABGimu, Timu, ImuTimestamps, X_obj, xcol )

    global PreIntegration_options
    
    %% Interpolate for IMUrate number of poses between every adjacent image frame pose
    
    %idend = 0;
    ai = 0; bi = 0; gi = 0;
    Ri = eye(3);        % current frame R
    Ti = zeros(3,1);    % current frame T
    end_pose = 0;
    
    for pid = 2 : nFrames % correspond to Pose 2...n
        ai1 = ABGimu(1, pid);
        bi1 = ABGimu(2, pid);
        gi1 = ABGimu(3, pid);
        
        Ri1 = fn_RFromABG( ai1, bi1, gi1 ); % next frm's Rotation
        Ti1 = Timu( :, pid );               % next frm's Translation
        
        dR = Ri1 * Ri';                     % change in Rotation between frames
        [da, db, dg] = fn_ABGFromR( dR );   % change in Euler angles between frames
        
        dT = Ti1 - Ti;                      % change in Translation betwwen frames
        
        nImu = ImuTimestamps(pid) - ImuTimestamps(pid - 1);            
        
        ts = 1:nImu;
        
        ait = ai + da * ts / nImu;          % every imu sample's Euler angle
        bit = bi + db * ts / nImu;
        git = gi + dg * ts / nImu;
        Tit = repmat(Ti,1,nImu) + repmat(dT,1,nImu) .* repmat(ts,3,1) / nImu;   % every imu sample's Translation
         
        ang_vec = [ ait; bit; git ];
        ang_vec = ang_vec(:);
        trans_vec = Tit(:);

        % fill out poses in structure array with size == IMUrate
        for i = 1 : nImu 
          X_obj.pose( end_pose + i ).ang.val = ang_vec( (i-1)*3 + 1 : (i-1)*3 + 3 );  
          X_obj.pose( end_pose + i ).ang.col = (1:3) + xcol;   xcol = xcol + 3;
          X_obj.pose( end_pose + i ).trans.xyz = trans_vec( (i-1)*3 + 1 : (i-1)*3 + 3 );  
          X_obj.pose( end_pose + i ).trans.col = (1:3) + xcol;   xcol = xcol + 3;
        end
    
        end_pose = end_pose + nImu;        
        Ri = Ri1; Ti = Ti1;             % next frame becomes current
        ai = ai1; bi = bi1; gi = gi1;            
    end    
    