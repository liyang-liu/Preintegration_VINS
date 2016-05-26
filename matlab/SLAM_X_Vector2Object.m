function X_obj = SLAM_X_Vector2Object( X_vec, X_obj )
    
    numPose = size(X_obj.pose, 1) + 1;
    numFeature = size(X_obj.feature, 1);
    
    % pose    
    for i=1:numPose-1
        %X_vec = [X_vec; X_obj.pose(i).ang; X_obj.pose(i).trans];
        X_obj.pose(i).ang.val = X_vec( (i-1)*6 + 1 : (i-1)*6 + 3);
        X_obj.pose(i).trans.xyz = X_vec( (i-1)*6 + 4 : (i-1)*6 + 6);
    end
    X_vec(1: (numPose-1)*6 ) = []; % delete
    
    % feature
    for i=1:numFeature
        %X_vec = [X_vec; X_obj.feature(i).xyz];
        X_obj.feature(i).xyz = X_vec( (i-1)*3 + 1 : (i-1)*3 + 3);
    end
    X_vec(1: numFeature*3 ) = []; % delete
    
    % velocity
    for i=1:numPose
        %X_vec = [X_vec; X_obj.velocity(i).xyz];
        X_obj.velocity(i).xyz = X_vec( (i-1)*3+1 : (i-1)*3 + 3);
    end
    X_vec(1: numPose*3 ) = []; % delete
    
    %
    %X_vec = [ X_vec; X_obj.g; X_obj.Au2c; X_obj.Tu2c; X_obj.Bf; X_obj.Bw ];
    X_obj.g.val = X_vec(1:3); 
    X_vec(1:3) = []; % delete
    
    X_obj.Au2c.val = X_vec(1:3);
    X_vec(1:3) = []; % delete
          
    X_obj.Tu2c.val = X_vec(1:3);
    X_vec(1:3) = []; % delete
    
    X_obj.Bf.val = X_vec(1:3);
    X_vec(1:3) = []; % delete
    
    X_obj.Bw.val = X_vec(1:3);
    X_vec(1:3) = []; % delete

    