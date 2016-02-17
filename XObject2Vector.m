function X_vec = XObject2Vector( X_obj )
    
    X_vec = [];
    
    numPose = size(X_obj.pose, 1) + 1;
    numFeature = size(X_obj.feature, 1);
    
    % pose
    for i=1:numPose-1
        X_vec = [X_vec; X_obj.pose(i).ang; X_obj.pose(i).trans];
    end
    
    % feature
    for i=1:numFeature
        X_vec = [X_vec; X_obj.feature(i).xyz];
    end
    
    % velocity
    for i=1:numPose
        X_vec = [X_vec; X_obj.velocity(i).xyz];
    end
    
    X_vec = [ X_vec;        ... 
              X_obj.g;      ... 
              X_obj.Au2c;  ...
              X_obj.Tu2c;  ...
              X_obj.Bf;    ...
              X_obj.Bw    ...
              ];
          
    
    

    