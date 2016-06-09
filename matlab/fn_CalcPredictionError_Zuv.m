function e = fn_CalcPredictionError_Zuv(RptFeatureObs, K, X, Zobs, nPoses, nPts, nIMUrate, ImuTimestamps)

    global PreIntegration_options

    %nObsId_FeatureObs = 2;
    fx = K(1,1); cx0 = K(1,3); fy = K(2,2); cy0 = K(2,3);
    
    % Feature co-ords in world frame
    p3d0 = X.feature;
    
    % initilize e data structure using Zobs
    e = SLAM_Z_Define( nPoses, nPts, nIMUrate, length( Zobs.fObs ));
    
    % Au2c, Tu2c    
    alpha = X.Au2c.val(1); beta = X.Au2c.val(2); gamma = X.Au2c.val(3);
    Ru2c = fn_RFromABG(alpha, beta, gamma);
    Tu2c = X.Tu2c.val;
    nUV = 0;
    
    % Reprojection at each pose
    nfs = size(RptFeatureObs,1);
    
    for(fid=1:nfs)        
        nObs = RptFeatureObs(fid).nObs;
        
        for(oid=1:nObs)            
            
            frm_id = RptFeatureObs(fid).obsv(oid).pid; 
            
            if(frm_id > nPoses)
                
                break;
                
            elseif(frm_id > 1)
                if((PreIntegration_options.bUVonly == 1) ||(PreIntegration_options.bPreInt == 1))
                    pid = frm_id - 1;
                else
                    pid = ImuTimestamps(frm_id)-ImuTimestamps(1);
                end
                Au = X.pose(pid).ang.val;
                Tu = X.pose(pid).trans.xyz;

                alpha = Au(1); beta = Au(2); gamma = Au(3);
                Ru = fn_RFromABG(alpha, beta, gamma);%Rx(alpha) * fRy (beta) * fRz(gamma);
                
            else % Pose 1 is special                
                
                Tu = zeros(3,1); 
                Ru = eye(3);                
                
            end
            
            Rc = Ru2c * Ru;
            Tc = Tu + (Ru)' * Tu2c;
    
            % Feature co-ords in camera frame
            p3d1 = Rc * ( p3d0(fid).xyz - Tc );
            u1 = fx * p3d1(1) / p3d1(3) + cx0;
            v1 = fy * p3d1(2) / p3d1(3) + cy0;
        
            nUV = nUV + 1;
            e.fObs(nUV).uv = [u1;v1] - Zobs.fObs(nUV).uv;        
        end
    end
    