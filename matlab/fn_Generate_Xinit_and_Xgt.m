function [X_obj, Xg_obj, Feature3D ] = fn_Generate_Xinit_and_Xgt( X_obj, Xg_obj, RptFeatureObs, imuData_cell, uvd_cell, Ru_cell, Tu_cell, nIMUdata, nIMUrate, dtIMU, dp, dv, dphi, K, cx0, cy0, focal_len, dt, SLAM_Params )

    global PreIntegration_options
    
    %% 1. Pose_ui: Obtain the initial estimation of IMU poses (camera has the
    % following relation with IMU;
    %         Rc = Ru2c * Ru_cell{cid};% Left multiply R to comply with the vectors to be multiplied on the right.
    %         Tc = Tu_cell{cid} + (Ru_cell{cid})' * Tu2c;    

    xcol = 0;    
    xgcol = 0;

    nPts = length( RptFeatureObs );
    nPoses = length( X_obj.pose ) + 1;
    
    if(PreIntegration_options.bInitPnF5VoU == 1)
        for pid=2:(nPoses-1)
            % Pick out T corresponding to the current pose 
            Timu = Tu_cell{pid}(:,1) - Tu_cell{pid-1}(:,1);
            fscaleGT(pid) = norm(Timu);
        end
        % The final pose
        Timu = Tu_cell{nPoses} - Tu_cell{nPoses-1}(:,1);
        fscaleGT(nPoses) = norm(Timu);     
     
        if(PreIntegration_options.bIMUodo == 1)
            %% Obtain initial poses from IMU data
            [Rcam, Acam, Tcam, Feature3D] = fn_GetPosesFromIMUdata(nPoses, nPts, dtIMU, dp, dv, dphi, ...
                                                K, RptFeatureObs, SLAM_Params );               
        else
            %% obtain relative poses from visual odometry
            [Rcam, Acam, Tcam, Feature3D] = fn_GetPosesFromMatchedFeatures( nPoses, nPts, ...
                        K, fscaleGT, RptFeatureObs, kfids);  
        end

        ABGimu = zeros(3, nPoses);
        Timu = zeros(3, nPoses);
        
        for(pid=1:(nPoses))% correspond to pose 2...n
            %	Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
            Rimu = SLAM_Params.Ru2c' * Rcam(:,:,pid) * SLAM_Params.Ru2c;
            [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fn_ABGFromR( Rimu );
            Timu(:, pid) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, pid) - Rimu' * SLAM_Params.Tu2c;
        end
        
        if(PreIntegration_options.bPreInt == 1)
            for p = 1-1+1:nPoses-1
                X_obj.pose(p).ang.val = ABGimu(:,p+1);
                X_obj.pose(p).ang.col = (1:3) + xcol; xcol = xcol + 3;

                X_obj.pose(p).trans.val = Timu(:,p+1);
                X_obj.pose(p).trans.col = (1:3) + xcol; xcol = xcol + 3;
            end
        
            if 0
                idstart  = 1; idend = 6*(nPoses - 1);
                tv = [ABGimu(:,2:end); Timu(:,2:end)];
                x(idstart:idend) = tv(:);
                %	idx = (nPoses - 1)*6;
            end
        else            
            [x] = fnLinearInterpPoses(nPoses, ABGimu, Timu, ImuTimestamps,x);
            %	idx = (nPoses - 1)*nIMUrate*6;
        end
    end

    %% Ground truth poses
    if(PreIntegration_options.bPreInt == 1)
        for pid=2:(nPoses-1)
            % Pick out R & T corresponding to the current pose       
            [alpha, beta, gamma] = fn_ABGFromR(Ru_cell{pid}{1});%Rc,Tc
            %xg(((pid-2)*6+1):((pid-1)*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,1)];       
            Xg_obj.pose(pid-1).ang.val = [alpha; beta; gamma]; 
            Xg_obj.pose(pid-1).ang.cols = (1:3) + xgcol; xgcol = xgcol + 3;
            Xg_obj.pose(pid-1).trans.val = Tu_cell{pid}(:,1); 
            Xg_obj.pose(pid-1).trans.cols = (1:3) + xgcol; xgcol = xgcol + 3;
            
        end
        % The final pose
        [alpha, beta, gamma] = fn_ABGFromR(Ru_cell{nPoses});%Rc,Tc
        %xg(((nPoses-2)*6+1):((nPoses-1)*6),1) = [alpha; beta; gamma; Tu_cell{nPoses}];        
        Xg_obj.pose(nPoses-1).ang.val = [alpha; beta; gamma]; 
        Xg_obj.pose(pid-1).ang.cols = (1:3) + xgcol; xgcol = xgcol + 3;
        Xg_obj.pose(nPoses-1).trans.val = Tu_cell{nPoses};
        Xg_obj.pose(pid-1).trans.cols = (1:3) + xgcol; xgcol = xgcol + 3;
        %% idx = (nPoses - 1)*6;
    else
        for pid=1:(nPoses)
            for(k=1:nIMUrate)
                % Pick out R & T corresponding to the current pose       
                [alpha, beta, gamma] = fn_ABGFromR( Ru_cell{pid}{k} );% Pick up the first key frame as well, so we need to get rid of it at L146
                p = (pid-1)*nIMUrate + k;
                Xg_obj.pose(p).ang.val = [ alpha; beta; gamma ];
                Xg_obj.pose(p).ang.cols = (1:3) + xgcol; xgcol = xgcol + 3;
                Xg_obj.pose(p).trans.xyz = Tu_cell{pid}(:,k);
                Xg_obj.pose(p).trans.cols = (1:3) + xgcol; xgcol = xgcol + 3;
                
                %xg(((pid-1)*nIMUrate*6+6*(k-1)+1):((pid-1)*nIMUrate*6+k*6),1) = [alpha; beta; gamma; Tu_cell{pid}(:,k)];       
            end
        end
        % The final pose
        [alpha, beta, gamma] = fn_ABGFromR(Ru_cell{nPoses});%Rc,Tc
        %xg(((nPoses - 1)*nIMUrate*6+1):((nPoses - 1)*nIMUrate*6+6),1) = [alpha; beta; gamma; Tu_cell{nPoses}];
        %p = (pid)*nIMUrate + k;
        Xg_obj.pose(p).ang.val = [ alpha; beta; gamma ];
        Xg_obj.pose(p).ang.cols = (1:3) + xgcol; xgcol = xgcol + 3;
        Xg_obj.pose(p).trans.xyz = Tu_cell{pid}(:,k);
        Xg_obj.pose(p).trans.cols = (1:3) + xgcol; xgcol = xgcol + 3;                
        % remove the initial camera pose.
        Xg_obj.pose = Xg_obj.pose(2:end);
        %xg = xg(7:end); % remove the initial camera pose.
        %idx = (nPoses - 1)*nIMUrate*6;
    end

    
    %% 2. f_ui: Extract feature positions at the initial IMU pose.
    if(PreIntegration_options.bInitPnF5VoU == 1)    
        for fid=1:length(Feature3D)
            %fid = Feature3D(fidx).fid;
            fp_c1 = Feature3D(fid).triangs(1).p3D ; % select the first group 
            fp_u1 = SLAM_Params.Ru2c' * fp_c1 + SLAM_Params.Tu2c;

            X_obj.feature(fid).xyz = fp_u1(:);
            X_obj.feature(fid).col = (1:3) + xcol;	xcol = xcol + 3;
        end
        X_obj.feature(nPts+1:end) = [];%delete unnecessary                     
    end

    uvd1 = uvd_cell{1}; 
    fp_c1 = uvd1; 
    fp_c1(1, :) = (uvd1(1, :) - cx0) .* fp_c1(3, :) / focal_len;
    fp_c1(2, :) = (uvd1(2, :) - cy0) .* fp_c1(3, :) / focal_len;
    fp_u1 = SLAM_Params.Ru2c' * fp_c1 + repmat(SLAM_Params.Tu2c, 1, size(fp_c1, 2));
    for (f=1:nPts)    
        Xg_obj.feature(f).xyz = fp_u1(:, f);
        Xg_obj.feature(f).col = (1:3) + xgcol;	xgcol = xgcol + 3;
    end
    %xg((idx+1):(idx+nPts*3)) = fp_u1(:);

    %% 3. Vi: Initial velocity for each pose
    %idx = idx+nPts*3;
    if(PreIntegration_options.bInitPnF5VoU == 1)
       [X_obj, xcol] = fn_InitVelocity(nPoses, X_obj, xcol, dp, dv, ...
           dtIMU, imuData_cell, nIMUdata, nIMUrate, dt, SLAM_Params ); 
    end
    
    % Special case for Pose 1:        
    if(PreIntegration_options.bPreInt == 1)
        Xg_obj.velocity(1).xyz = imuData_cell{2}.initstates(7:9);
        Xg_obj.velocity(1).col = (1:3) + xgcol;                
        xgcol = xgcol + 3;    
        for pid = 2:nPoses
            %%idx = idx + 3;
            %%xg((idx+1):(idx+3)) = imuData_cell{pid}.initstates(7:9);
            Xg_obj.velocity(pid).xyz = imuData_cell{pid}.initstates(7:9);
            Xg_obj.velocity(pid).col = (1:3) + xgcol; xgcol = xgcol + 3;    
        end
        %%idx = idx + 3;
    else
        vu = vu';
        for p = 1: nPoses*nIMUrate
            Xg_obj.pose(p).ang.val = vu(1:3:end);
            Xg_obj.velocity(pid).col = (1:3) + xgcol; xgcol = xgcol + 3;   
            Xg_obj.pose(p).trans.val = vu(4:3:end);
            Xg_obj.pose(p).trans.col = (1:3) + xgcol; xgcol = xgcol + 3;
        end
    end

    %% 4. g:%(nPoses-1)*6+nPts*3+3*nPoses (nPoses-1)*6+nPts*3+3*nPoses 
    X_obj.g.val = SLAM_Params.g0;
    X_obj.g.col = (1:3) + xcol; xcol = xcol + 3;
    Xg_obj.g.val = SLAM_Params.g_true;
    Xg_obj.g.col = (1:3) + xgcol; xgcol = xgcol + 3;
    %%x((idx+1):(idx+3)) = g0;
    %%xg((idx+1):(idx+3)) = g_true;
    
    %%idx = idx +3;
    %% 5. Ru2c, Tu2c:(nPoses-1)*6+nPts*3+3*nPoses+4:(nPoses-1)*6+nPts*3+3*nPoses + 9
    [alpha, beta, gamma] = fn_ABGFromR( SLAM_Params.Ru2c );
    X_obj.Au2c.val = [alpha; beta; gamma];
    X_obj.Au2c.col = (1:3) + xcol; xcol = xcol + 3;
    X_obj.Tu2c.val = SLAM_Params.Tu2c;
    X_obj.Tu2c.col = (1:3) + xcol; xcol = xcol + 3;
    Xg_obj.Au2c.val = [alpha; beta; gamma];
    Xg_obj.Au2c.col = (1:3) + xgcol; xgcol = xgcol + 3;
    Xg_obj.Tu2c.val = SLAM_Params.Tu2c;
    Xg_obj.Tu2c.col = (1:3) + xgcol; xgcol = xgcol + 3;
    
    %%x((idx+1):(idx+6)) = [alpha;beta;gamma;Tu2c];
    %%xg((idx+1):(idx+6)) = [alpha;beta;gamma;Tu2c];
    %%idx = idx + 6;
    
    %% 6. bf,bw %(nPoses-1)*6+nPts*3+3*nPoses+10
    X_obj.Bf.val = SLAM_Params.bf0;
    X_obj.Bf.col = (1:3) + xcol; xcol = xcol + 3;
    X_obj.Bw.val = SLAM_Params.bw0;
    X_obj.Bw.col = (1:3) + xcol; xcol = xcol + 3;
    Xg_obj.Bf.val = SLAM_Params.bf_true;
    Xg_obj.Bf.col = (1:3) + xgcol; xgcol = xgcol + 3;
    Xg_obj.Bw.val = SLAM_Params.bw_true;
    Xg_obj.Bw.col = (1:3) + xgcol; xgcol = xgcol + 3;
    
    %%x((idx+1):(idx+6)) = [bf0;bw0]; 
    %%xg((idx+1):(idx+6)) = [bf_true;bw_true];
    %idx = idx + 6;
    %xg = xg(1:idx);
    %x = x(1:idx);
    %% Display Xgt
    fprintf('Ground Truth Value:\n\t Xg_obj=[\nAng: ');
    %fprintf('%f ', Xg_obj(1:20));
    fprintf('%f ', [Xg_obj.pose(1).ang.val; Xg_obj.pose(2).ang.val; Xg_obj.pose(3).ang.val]);
    fprintf('\nTrans: ');
    fprintf('%f ', [Xg_obj.pose(1).trans.val; Xg_obj.pose(2).trans.val; Xg_obj.pose(3).trans.val]);
    fprintf('\nFeature_1: ');
    fprintf('%f ', Xg_obj.feature(1).xyz');
    fprintf('...]\n');  
    % Show Pose-feature graph
    
    if(PreIntegration_options.bInitPnF5VoU == 0)
        %%x = xg;
        X_obj = Xg_obj;
    end
    
    % Add noise to the state vector x
    if(PreIntegration_options.bAddInitialNoise == 1) 
        nc = 1; idst = 0;    
        % 1. Add to IMU poses
        if(PreIntegration_options.bPreInt == 1)
            nr = 6*(nPoses - 1);
        else
            nr = 6*nIMUrate*(nPoses - 1);
        end
        [gns] = fn_GenGaussNoise(nr, nc, fXnoisescale);
        %%x((idst+1):(idst+nr)) = x((idst+1):(idst+nr)) + gns; % x = x + gns;[nr,nc] = size(x((idr+1):end));    
        [X_obj.pose(:).ang.val] = {gns(1:3:end)};
        [X_obj.pose(:).trans.xyz] = {gns(4:3:end)};
        %%idst = idst + nr + 3*nPts; % Already added noises to feature positions    
        
        % 2. Add to IMU velocity
        if(PreIntegration_options.bPreInt == 1)
            nr = 3*nPoses;
        else
            nr = 3*(nIMUrate*(nPoses-1)+1);
        end
        [gns] = fn_GenGaussNoise(nr, nc, fXnoisescale);
        %%x((idst+1):(idst+nr)) = x((idst+1):(idst+nr)) + gns;    
        [X_obj.velocity(:).xyz] = {gns};        
        %%idst = idst + nr;
        
        % 3. Add to g, Au2c, Tu2c, bf, bw
        nr = 3*5;
        [gns] = fn_GenGaussNoise(nr, nc, fXnoisescale);
        %%x((idst+1):(idst+nr)) = x((idst+1):(idst+nr)) + gns;  
        X_obj.g.val = X_obj.g.val + gns(1:3);        
        X_obj.Au2c.val = X_obj.Au2c.val + gns(4:6);        
        X_obj.Tu2c.val = X_obj.Tu2c.val + gns(7:9);        
        X_obj.Bf.val = X_obj.bf.val + gns(10:12);        
        X_obj.Bw.val = X_obj.bw.val + gns(13:15);        
        
    end
    
    % Show initial value X0
    fprintf('\nInitial Value:\n\t X0=[\nAng: ');
    %fprintf('%f ', x(1:20));
    fprintf('%f ', [X_obj.pose(1).ang.val; X_obj.pose(2).ang.val; X_obj.pose(3).ang.val]);
    fprintf('\nTrans: ');
    fprintf('%f ', [X_obj.pose(1).trans.val; X_obj.pose(2).trans.val; X_obj.pose(3).trans.val]);
    fprintf('\nFeature_1: ');
    fprintf('%f ', X_obj.feature(1).xyz');
    fprintf('...]\n');    
