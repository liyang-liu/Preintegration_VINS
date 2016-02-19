function [xg] = fnGetXgroundtruth(nPoseNew, nAllposes, nPts, bPreInt, Ru_cell, Tu_cell, ...
    uvd_cell, cx0, cy0, f, imuData_cell, nIMUrate, vu, Ru2c, Tu2c, g_true, bf_true, bw_true)

%% 1. Ground truth poses
    if(bPreInt == 1)
        for pid=2:(nPoseNew-1)
            % Pick out R & T corresponding to the current pose       
            [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid}{1});%Rc,Tc
            xg(((pid-2)*6+1):((pid-1)*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,1)];       
        end
        % The final pose
        pid = nPoseNew;
        if(nPoseNew == nAllposes)
            [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid});%Rc,Tc
            xg(((nPoseNew-2)*6+1):((nPoseNew-1)*6),1) = [alpha; beta; gamma; Tu_cell{pid}];  
        else
            [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid}{1});%Rc,Tc
            xg(((pid-2)*6+1):((pid-1)*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,1)];            
        end
        idx = (nPoseNew - 1)*6;
    else
        for pid=1:(nPoseNew-1)
            for(k=1:nIMUrate)
                % Pick out R & T corresponding to the current pose       
                [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid}{k});% Pick up the first key frame as well, so we need to get rid of it at L146
                xg(((pid-1)*nIMUrate*6+6*(k-1)+1):((pid-1)*nIMUrate*6+k*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,k)];       
            end
        end
        % The final pose
        pid = nPoseNew;
        if(nPoseNew == nAllposes)        
            [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid});%Rc,Tc
            xg(((nPoseNew - 1)*nIMUrate*6+1):((nPoseNew - 1)*nIMUrate*6+6),1) = [alpha; beta; gamma; Tu_cell{pid}];
        else
            for(k=1:nIMUrate)
                [alpha, beta, gamma] = fABGfrmR(Ru_cell{pid}{k});% Pick up the first key frame as well, so we need to get rid of it at L146
                xg(((pid-1)*nIMUrate*6+6*(k-1)+1):((pid-1)*nIMUrate*6+k*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,k)];   
            end
        end
        xg = xg(7:end); % remove the initial camera pose.
        idx = (nPoseNew - 1)*nIMUrate*6;
    end

%% 2. f_ui: Extract feature positions at the initial IMU pose.     
    uvd1 = uvd_cell{1}; 
    fp_c1 = uvd1; 
    fp_c1(1, :) = (uvd1(1, :) - cx0) .* fp_c1(3, :) / f;
    fp_c1(2, :) = (uvd1(2, :) - cy0) .* fp_c1(3, :) / f;
    % Ru2c * (Pui - Tu2c) = Pci  ==> Pui = Ru2c' * Pci + Tu2c
    fp_u1 = Ru2c' * fp_c1 + repmat(Tu2c, 1, size(fp_c1, 2));
    xg((idx+1):(idx+nPts*3)) = fp_u1(:);
    
%% 3. Vi: Initial velocity for each pose
    idx = idx+nPts*3;
    % Special case for Pose 1:        
    if(bPreInt == 1)
        xg((idx+1):(idx+3)) = imuData_cell{2}.initstates(7:9);
        for pid = 2:nPoseNew
            idx = idx + 3;
            xg((idx+1):(idx+3)) = imuData_cell{pid}.initstates(7:9);
        end
        idx = idx + 3;
    else
        vu = vu';
        xg((idx+1):(idx+((nPoseNew-1)*nIMUrate+1)*3)) = vu(:);
        idx = idx+((nPoseNew-1)*nIMUrate+1)*3;
    end

    %% 4. g:%(nPoses-1)*6+nPts*3+3*nPoses (nPoses-1)*6+nPts*3+3*nPoses 
    xg((idx+1):(idx+3)) = g_true;
    idx = idx +3;
    %% 5. Ru2c, Tu2c:(nPoses-1)*6+nPts*3+3*nPoses+4:(nPoses-1)*6+nPts*3+3*nPoses + 9
    [alpha, beta, gamma] = fABGfrmR(Ru2c);
    xg((idx+1):(idx+6)) = [alpha;beta;gamma;Tu2c];
    idx = idx + 6;
    %% 6. bf,bw %(nPoses-1)*6+nPts*3+3*nPoses+10
    xg((idx+1):(idx+6)) = [bf_true;bw_true];
    idx = idx + 6;
    xg = xg(1:idx);
    
end