function X_vec = XObject2Vector( X_obj )
    
    X_vec = [];
    
    
    % pose
    numPose = length( X_obj.pose ) + 1;
    for i=1:numPose-1
        X_vec = [X_vec; X_obj.pose(i).ang.val; X_obj.pose(i).trans.val];
    end
    
    % feature
    numFeature = length( X_obj.feature );
    for i=1:numFeature
        X_vec = [X_vec; X_obj.feature(i).xyz];
    end
    
    % velocity
    for i=1:numPose
        X_vec = [X_vec; X_obj.velocity(i).xyz];
    end
    
    X_vec = [ X_vec;        ... 
              X_obj.g.val;      ... 
              X_obj.Au2c.val;  ...
              X_obj.Tu2c.val;  ...
              X_obj.Bf.val;    ...
              X_obj.Bw.val    ...
              ];
          
    
    

    