function X_diff = SLAM_X_Object2Vector( X_obj1, X_obj2 )
    
    X_diff = X_obj1;
    
    numPose = size(X_obj1.pose, 1) + 1;
    numFeature = size(X_obj1.feature, 1);
    
    % pose
    for i=1:numPose-1
        X_diff.pose(i).ang.val = X_obj1.pose(i).ang.val - X_obj2.pose(i).ang.val;
        X_diff.pose(i).trans.val = X_obj1.pose(i).trans.val - X_obj2.pose(i).trans.val;
    end
    
    % feature
    for i=1:numFeature
        X_diff.feature(i).xyz = X_obj1.feature(i).xyz - X_obj2.feature(i).xyz;
    end
    
    % velocity
    for i=1:numPose
        X_diff.velocity(i).xyz = X_obj1.velocity(i).xyz - X_obj2.velocity(i).xyz;
    end
    
    X_diff.g.val = X_obj1.g.val - X_obj2.g.val;
    
    X_diff.Au2c.val = X_obj1.Au2c.val - X_obj2.Au2c.val;    
    X_diff.Tu2c.val = X_obj1.Tu2c.val - X_obj2.Tu2c.val;
    
    X_diff.Bf.val = X_obj1.Bf.val - X_obj2.Bf.val;
    X_diff.Bw.val = X_obj1.Bw.val - X_obj2.Bw.val;
          
    
    

    