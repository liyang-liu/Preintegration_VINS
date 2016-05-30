function [X_obj, Xg_obj, Feature3D ] = fn_Generate_Xinit_and_Xgt( X_obj, Xg_obj, RptFeatureObs, imuData_cell, uvd_cell, noisefree_imuData_cell, noisefree_uvd_cell, Ru_cell, Tu_cell, ...
                                                    nIMUdata, nIMUrate, ImuTimestamps, dtIMU, dp, dv, dphi, K, cx0, cy0, focal_len, dt, vu, SLAM_Params )

    global PreIntegration_options
    
    %% 1. Pose_ui: Obtain the initial estimation of IMU poses (camera has the
    % following relation with IMU;
    %         Rc = Ru2c * Ru_cell{cid}; % Left multiply R to comply with the vectors to be multiplied on the right.
    %         Tc = Tu_cell{cid} + (Ru_cell{cid})' * Tu2c;    

    xcol = 0;    
    xgcol = 0;

    nPts = length( RptFeatureObs );
    if ( PreIntegration_options.bPreInt == 1 )
        nFrames = length( X_obj.pose ) + 1;
    else
        nFrames = round( nIMUdata / nIMUrate ) + 1;
    end
    
    
    if(PreIntegration_options.bInitPnF5VoU == 1)
        for frm_id=2:(nFrames-1)
            % Pick out T corresponding to the current frame
            Timu = Tu_cell{frm_id}(:,1) - Tu_cell{frm_id - 1}(:,1);
            fscaleGT(frm_id) = norm(Timu);
        end
        % The final frame
        Timu = Tu_cell{nFrames} - Tu_cell{nFrames-1}(:,1);
        fscaleGT(nFrames) = norm(Timu);     
     
        %% Get Camera's R, T for each frame
        if(PreIntegration_options.bIMUodo == 1)
            %% Obtain initial poses from IMU data
            [ Rcam, Acam, Tcam, Feature3D ] = fn_GetPosesFromIMUdata( nFrames, nPts, dtIMU, dp, dv, dphi, ...
                                                K, RptFeatureObs, SLAM_Params );               
        else
            %% obtain relative poses from visual odometry,
            [ Rcam, Acam, Tcam, Feature3D ] = fn_GetPosesFromMatchedFeatures( nFrames, nPts, ...
                                                K, fscaleGT, RptFeatureObs, kfids);  
        end

        %% Get IMU's R, T for each frame
        ABGimu = zeros( 3, nFrames );
        Timu = zeros( 3, nFrames );        
        for frm_id = 1 : nFrames % correspond to pose 2...n
            Rimu = SLAM_Params.Ru2c' * Rcam(:,:,frm_id) * SLAM_Params.Ru2c;
            [ ABGimu(1, frm_id), ABGimu(2, frm_id), ABGimu(3, frm_id) ] = fn_ABGFromR( Rimu );
            Timu(:, frm_id) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, frm_id) - Rimu' * SLAM_Params.Tu2c;
        end
        
        if(PreIntegration_options.bPreInt == 1)
            for frm = 1-1+1:nFrames-1
                X_obj.pose(frm).ang.val = ABGimu(:,frm+1);
                X_obj.pose(frm).ang.col = (1:3) + xcol; xcol = xcol + 3;

                X_obj.pose(frm).trans.xyz = Timu(:,frm+1);
                X_obj.pose(frm).trans.col = (1:3) + xcol; xcol = xcol + 3;
            end        
        else            
            [X_obj, xcol] = fn_LinearInterpPoses( nFrames, ABGimu, Timu, ImuTimestamps, X_obj, xcol );
        end
    end

    %% Ground truth poses
    if(PreIntegration_options.bPreInt == 1)
        for frm=2:(nFrames-1)
            % Pick out R & T corresponding to the current pose       
            [alpha, beta, gamma] = fn_ABGFromR(Ru_cell{frm}{1});%Rc,Tc
            Xg_obj.pose(frm-1).ang.val = [alpha; beta; gamma]; 
            Xg_obj.pose(frm-1).ang.col = (1:3) + xgcol; xgcol = xgcol + 3;
            Xg_obj.pose(frm-1).trans.xyz = Tu_cell{frm}(:,1); 
            Xg_obj.pose(frm-1).trans.col = (1:3) + xgcol; xgcol = xgcol + 3;
            
        end
        % The final pose
        [alpha, beta, gamma] = fn_ABGFromR( Ru_cell{nFrames} );%Rc,Tc
        Xg_obj.pose(nFrames-1).ang.val = [alpha; beta; gamma]; 
        Xg_obj.pose(nFrames-1).ang.col = (1:3) + xgcol; xgcol = xgcol + 3;
        Xg_obj.pose(nFrames-1).trans.xyz = Tu_cell{nFrames};
        Xg_obj.pose(nFrames-1).trans.col = (1:3) + xgcol; xgcol = xgcol + 3;
    else
        xgcol = xgcol - 6; % since we will do a shift backwards in the end
        for frm = 1:( nFrames - 1 )
            for k = 1 : nIMUrate
                % Pick out R & T corresponding to the current pose       
                [alpha, beta, gamma] = fn_ABGFromR( Ru_cell{frm}{k} );% Pick up the first key frame as well, so we need to get rid of it at L146
                p =  (frm-1) * nIMUrate + k;
                Xg_obj.pose(p).ang.val = [ alpha; beta; gamma ];
                Xg_obj.pose(p).ang.col = (1:3) + xgcol; xgcol = xgcol + 3;
                Xg_obj.pose(p).trans.xyz = Tu_cell{frm}(:,k);
                Xg_obj.pose(p).trans.col = (1:3) + xgcol; xgcol = xgcol + 3;
            end
        end
        
        %% The final pose
        [alpha, beta, gamma] = fn_ABGFromR( Ru_cell{nFrames} );%Rc,Tc
        p = (nFrames - 1) * nIMUrate + 1;
        Xg_obj.pose(p).ang.val = [ alpha; beta; gamma ];
        Xg_obj.pose(p).ang.col = (1:3) + xgcol; xgcol = xgcol + 3;
        Xg_obj.pose(p).trans.xyz = Tu_cell{nFrames};
        Xg_obj.pose(p).trans.col = (1:3) + xgcol; xgcol = xgcol + 3;                
        % Shift all poses to left to remove the initial camera pose which
        % has to be zero
        Xg_obj.pose = Xg_obj.pose(2 : end);
    end

    %% 2. f_ui: Extract feature positions at the initial IMU pose.
    if(PreIntegration_options.bInitPnF5VoU == 1)    
        for fid = 1 : nPts
            fp_c1 = Feature3D(fid).triangs(1).p3D ; % select the first group 
            fp_u1 = SLAM_Params.Ru2c' * fp_c1 + SLAM_Params.Tu2c;

            X_obj.feature(fid).xyz = fp_u1(:);
            X_obj.feature(fid).col = (1:3) + xcol;	xcol = xcol + 3;
        end
        X_obj.feature(nPts + 1:end) = [];%delete unnecessary                     
    end

    uvd1 = noisefree_uvd_cell{1}; 
    fp_c1 = uvd1; 
    fp_c1(1, :) = ( uvd1(1, :) - cx0 ) .* fp_c1(3, :) / focal_len;
    fp_c1(2, :) = ( uvd1(2, :) - cy0 ) .* fp_c1(3, :) / focal_len;
    fp_u1 = SLAM_Params.Ru2c' * fp_c1 + repmat(SLAM_Params.Tu2c, 1, size(fp_c1, 2));
    for f = 1 : nPts
        Xg_obj.feature(f).xyz = fp_u1(:, f);
        Xg_obj.feature(f).col = (1:3) + xgcol;	xgcol = xgcol + 3;
    end
    
    %% 3. Vi: Initial velocity for each pose
    if(PreIntegration_options.bInitPnF5VoU == 1)
       [X_obj, xcol] = fn_InitVelocity(nFrames, X_obj, xcol, dp, dv, ...
                                dtIMU, imuData_cell, nIMUdata, nIMUrate, dt, SLAM_Params ); 
    end
    
    if(PreIntegration_options.bPreInt == 1)
        % Special case for Pose 1:        
        Xg_obj.velocity(1).xyz = imuData_cell{2}.initstates(7:9);
        Xg_obj.velocity(1).col = (1:3) + xgcol;     xgcol = xgcol + 3;    
        for frm_id = 2:nFrames
            Xg_obj.velocity(frm_id).xyz = imuData_cell{frm_id + 1}.initstates(7:9);
            Xg_obj.velocity(frm_id).col = (1:3) + xgcol; xgcol = xgcol + 3;    
        end
    else
        vu = vu';
        for p = 1 : (nFrames - 1) * nIMUrate + 1
            Xg_obj.velocity(p).xyz = vu( (p-1)*3 + 1 : (p-1)*3 + 3 );
            Xg_obj.velocity(p).col = (1:3) + xgcol; xgcol = xgcol + 3;   
        end
    end

    %% 4. g
    X_obj.g.val = SLAM_Params.g0;
    X_obj.g.col = (1:3) + xcol; xcol = xcol + 3;
    Xg_obj.g.val = SLAM_Params.g_true;
    Xg_obj.g.col = (1:3) + xgcol; xgcol = xgcol + 3;
    
    %% 5. Ru2c, Tu2c
    [alpha, beta, gamma] = fn_ABGFromR( SLAM_Params.Ru2c );
    X_obj.Au2c.val = [alpha; beta; gamma];
    X_obj.Au2c.col = (1:3) + xcol; xcol = xcol + 3;
    X_obj.Tu2c.val = SLAM_Params.Tu2c;
    X_obj.Tu2c.col = (1:3) + xcol; xcol = xcol + 3;
    Xg_obj.Au2c.val = [alpha; beta; gamma];
    Xg_obj.Au2c.col = (1:3) + xgcol; xgcol = xgcol + 3;
    Xg_obj.Tu2c.val = SLAM_Params.Tu2c;
    Xg_obj.Tu2c.col = (1:3) + xgcol; xgcol = xgcol + 3;
        
    %% 6. bf,bw %(nFrames-1)*6+nPts*3+3*nFrames+10
    X_obj.Bf.val = SLAM_Params.bf0;
    X_obj.Bf.col = (1:3) + xcol; xcol = xcol + 3;
    X_obj.Bw.val = SLAM_Params.bw0;
    X_obj.Bw.col = (1:3) + xcol; xcol = xcol + 3;
    Xg_obj.Bf.val = SLAM_Params.bf_true;
    Xg_obj.Bf.col = (1:3) + xgcol; xgcol = xgcol + 3;
    Xg_obj.Bw.val = SLAM_Params.bw_true;
    Xg_obj.Bw.col = (1:3) + xgcol; xgcol = xgcol + 3;
    
    %% Display Xgt
    fprintf('Ground Truth Value:\n\t Xg_obj=[\nAng: ');
    %fprintf('%f ', Xg_obj(1:20));
    fprintf('%f ', [Xg_obj.pose(1).ang.val; Xg_obj.pose(2).ang.val; Xg_obj.pose(3).ang.val]);
    fprintf('\nTrans: ');
    fprintf('%f ', [Xg_obj.pose(1).trans.xyz; Xg_obj.pose(2).trans.xyz; Xg_obj.pose(3).trans.xyz]);
    fprintf('\nFeature_1: ');
    fprintf('%f ', Xg_obj.feature(1).xyz');
    fprintf('...]\n');  
    
    if(PreIntegration_options.bInitPnF5VoU == 0)
        %%x = xg;
        X_obj = Xg_obj;
    end
    
    % Add noise to the state vector x
    if(PreIntegration_options.bAddInitialNoise == 1) 
        nc = 1; idst = 0;    
        % 1. Add to IMU poses
        if 1
            nr = length( X_obj.pose );
            if(PreIntegration_options.bPreInt == 1)
                nr = 6*(nFrames - 1);
            else
                nr = 6*nIMUrate*(nFrames - 1);
            end
        end
        [gns] = fn_GenGaussNoise(nr, nc, fXnoisescale);
        [X_obj.pose(:).ang.val] = [X_obj.pose(:).ang.val] + {gns(1:3:end)};
        [X_obj.pose(:).trans.xyz] = [X_obj.pose(:).trans.xyz] + {gns(4:3:end)};
        
        % 2. Add to IMU velocity
        if 1
            nr = length( X_obj.velocity );
        else
            if(PreIntegration_options.bPreInt == 1)
                nr = 3*nFrames;
            else
                nr = 3*(nIMUrate*(nFrames-1)+1);
            end
        end
        [gns] = fn_GenGaussNoise(nr, nc, fXnoisescale);
        [X_obj.velocity(:).xyz] = {gns};        
        
        % 3. Add to g, Au2c, Tu2c, bf, bw
        nr = 3*5;
        [gns] = fn_GenGaussNoise(nr, nc, fXnoisescale);
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
    fprintf('%f ', [X_obj.pose(1).trans.xyz; X_obj.pose(2).trans.xyz; X_obj.pose(3).trans.xyz]);
    fprintf('\nFeature_1: ');
    fprintf('%f ', X_obj.feature(1).xyz');
    fprintf('...]\n');    
