function X_diff = XObject2Vector( X_obj1, X_obj2 )
    
    X_diff = X_obj1;
    
    numPose = size(X_obj1.pose, 1) + 1;
    numFeature = size(X_obj1.feature, 1);
    
    % pose
    for i=1:numPose-1
        X_diff.pose(i).ang = X_obj1.pose(i).ang - X_obj2.pose(i).ang;
        X_diff.pose(i).trans = X_obj1.pose(i).trans - X_obj2.pose(i).trans;
    end
    
    % feature
    for i=1:numFeature
        X_diff.feature(i).xyz = X_obj1.feature(i).xyz - X_obj2.feature(i).xyz;
    end
    
    % velocity
    for i=1:numPose
        X_diff.velocity(i).xyz = X_obj1.velocity(i).xyz - X_obj2.velocity(i).xyz;
    end
    
    X_diff.g = X_obj1.g - X_obj2.g;
    
    X_diff.Au2c = X_obj1.Au2c - X_obj2.Au2c;    
    X_diff.Tu2c = X_obj1.Tu2c - X_obj2.Tu2c;
    
    X_diff.Bf = X_obj1.Bf - X_obj2.Bf;
    X_diff.Bw = X_obj1.Bw - X_obj2.Bw;
          
    
    

    