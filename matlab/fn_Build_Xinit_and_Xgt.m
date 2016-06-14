function [X_obj, Xg_obj, Feature3D ] = fn_Build_Xinit_and_Xgt( X_obj, Xg_obj, ...
                        RptFeatureObs, RptFidSet, Feature3D, gtIMUposes, selpids, PBAFeature, ...
                        nPoses, nPts, nIMUdata, nIMUrate, dtIMU, ...
                        ImuTimestamps, imufulldata, inertialDelta, K, dt, SLAM_Params )
    global PreIntegration_options Data_config
    
% Based on Liang's results, pack the state vector x.
        %     if(isunix)
        %         Data_config.DATA_DIR = '/home/youbwang/Documents/Malaga/Liang/ParallaxBA2Shoudong/DataPrepareBA/Whole170R/Result/';
        %     else
        %         Data_config.DATA_DIR = 'E:\uDocs\Research\IMU\Liang\ParallaxBA2Shoudong\DataPrepareBA\Whole170R\Result\';
        %     end
        % Intial values of poses and IMU states
    xg_col = 0;
    xcol = 0;
    
    if( PreIntegration_options.bMalaga == 1 )
        load( [ Data_config.DATA_DIR 'PBAPose.mat']);    
        tv = (PBAPose(1:nPoses, :))';
        
        ABGcam = [tv(3, :);tv(2, :);tv(1, :)];%tv(1:3, :);%
        Tcam = tv(4:6, :);           
        clearvars PBAPoses tv
        % Given Rcam/Tcam, Rimu/Timu can be calculated as follows:
        % Rimu = Ru2c' * Rc1u; Timu = Tc1u - Rimu '* Tu2c;
        % Rc1u = Rcam * Ru2c; Tc1u = Tu2c + Ru2c'*Tcam
        for pid=1:(nPoses) % correspond to pose 1...n
            Rcam = fn_RFromABG( ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid) );
            Rimu = SLAM_Params.Ru2c' * Rcam * SLAM_Params.Ru2c;
            [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fn_ABGFromR( Rimu );
            Timu(:, pid) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, pid) - Rimu' * SLAM_Params.Tu2c;
        end   
        tv = [ABGimu(:, 2:end); Timu(:, 2:end)];
        
        if(PreIntegration_options.bPreInt == 1)
           %xg(1:((nPoses-1)*6)) = tv(:);
           for i = 1 : nPoses - 1
                Xg_obj.pose(i).ang.val = tv( 1:3, i ); %tv( (i-1)*6 + 1 : (i-1)*6 + 3 );
                Xg_obj.pose(i).ang.col = (1:3) + xg_col;   xg_col = xg_col + 3;
                Xg_obj.pose(i).trans.xyz = tv( 4:6, i ); %tv( (i-1)*6 + 4 : (i-1)*6 + 6 );
                Xg_obj.pose(i).trans.col = (1:3) + xg_col;   xg_col = xg_col + 3;
            end

        else
           [Xg_obj, xg_col] = fn_LinearInterpPoses(nPoses, ABGimu, Timu, ImuTimestamps, Xg_obj, xg_col );
        end
        
    elseif( PreIntegration_options.bDinuka == 1 )
        tv = (gtIMUposes(selpids(1:(nPoses)), 2:7))';
        ABGimu = tv(1:3, :);
        Timu = tv(4:6, :);
        Tcam = zeros(3, nPoses);
        for(pid=1:nPoses)
            % Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
            Rimu = fn_RFromABG( ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid) );
            Tcam(:, pid) = SLAM_Params.Ru2c * (Timu(:, pid) - SLAM_Params.Tu2c + Rimu' * SLAM_Params.Tu2c);
        end
        if (PreIntegration_options.bPreInt == 1)
            [ Xg_obj, xg_col ] = fn_CalcLocalRelativePoses(Xg_obj, xg_col, nPoses, tv);
        else
            pall = ( gtIMUposes(selpids(1):(nIMUdata+selpids(1)), 2:7) )';
            [ Xg_obj, xg_col ] = fn_CalcLocalRelativePoses( Xg_obj, xg_col, nIMUdata+1, pall);
        end
    end    
    fscaleGT = zeros(nPoses, 1);
    %     if(bDinuka == 1)
    % %         t0 = tv(4:6, 1);
    % %         for(pid=2:nPoses)        
    % %            Timu = tv(4:6, pid) - tv(4:6, pid-1); 
    % %            fscaleGT(pid) = norm(Timu);
    % %         end
    %     elseif(PreIntegration_options.bMalaga == 1)
    % %         tv = [zeros(6,1), tv];
    % %         for(pid=1:(nPoses-1))        
    % %            Timu = tv(4:6, pid) - tv(4:6, pid-1); 
    % %            fscaleGT(pid) = norm(Timu);
    % %         end        
    %     end
    for(pid=2:nPoses)        
       Timu = Tcam(:, pid) - Tcam(:, pid-1); 
       fscaleGT(pid) = norm(Timu);
    end
    
    if(PreIntegration_options.bInitPnF5VoU == 1)
        if(PreIntegration_options.bIMUodo == 1)
            %% Obtain initial poses from IMU data
            [Rcam, Acam, Tcam, Feature3D] = fn_GetPosesFromIMUdata( nPoses, nPts, dtIMU, inertialDelta, ...
                        K, RptFeatureObs, SLAM_Params );
        else
            %% obtain relative poses from visual odometry
            [Rcam, Acam, Tcam, Feature3D] = fn_GetPosesFromMatchedFeatures(nPoses, nPts, ...
                        K, fscaleGT, bSimData, RptFeatureObs, kfids);  
        end
        ABGimu = zeros(3, nPoses);
        Timu = zeros(3, nPoses); % ABGimu = zeros(3, nPoses);            
        for(pid=1:(nPoses))% correspond to pose 1...n
            %	Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
            Rimu = SLAM_Params.Ru2c' * Rcam(:,:,pid) * SLAM_Params.Ru2c;
            [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fn_ABGFromR( Rimu );
            Timu(:, pid) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, pid) - Rimu' * SLAM_Params.Tu2c;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %         tv = [ABGimu; Timu];
        %         x(idstart:idend) = tv(:);
        %     else
        %     %     load('pbaPose10.mat');
        %     %     tv = (Pose(2:nPoses, :))';
        %         camposes = tv(:);
        %         if(bDinuka == 1)
        %             ABGimu = tv(1:3, 2:nPoses);%
        %             Timu = tv(4:6, 2:nPoses) - repmat(tv(4:6,1),1,nPoses-1);
        %             clearvars tv
        %         elseif(bMalaga == 1)
        % %             ABGcam = [tv(3, :);tv(2, :);tv(1, :)];%tv(1:3, :);%
        % %             Tcam = tv(4:6, :);           
        % %             clearvars PBAPoses tv
        % %             % Given Rcam/Tcam, Rimu/Timu can be calculated as follows:
        % %             % Rimu = Ru2c' * Rc1u; Timu = Tc1u - Rimu '* Tu2c;
        % %             % Rc1u = Rcam * Ru2c; Tc1u = Tu2c + Ru2c'*Tcam
        % %             for(pid=1:(nPoses-1))% correspond to pose 2...n
        % %                 Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
        % %                 Rimu = Ru2c'*Rcam*Ru2c;
        % %                 [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
        % %                 Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
        % %             end
        %         end
    end
    idstart = 1; 
    if( ( PreIntegration_options.bUVonly == 1 ) || ( PreIntegration_options.bPreInt == 1 ))
        %idend = 6*(nPoses-1);
        if ( PreIntegration_options.bInitPnF5VoU == 1 )
            tv = [ABGimu(:, 2:end); Timu(:, 2:end)];% tv = [ABGimu; Timu];        
            %x(idstart:idend) = tv(:);
            for i = 1 : ( nPoses - 1 )
                X_obj.pose(i).ang.val = tv( 1:3, i );
                X_obj.pose(i).ang.col = (1:3) + xcol;       xcol = xcol + 3;
                X_obj.pose(i).trans.xyz = tv( 4:6, i );
                X_obj.pose(i).trans.col = (1:3) + xcol;     xcol = xcol + 3;
            end
        end
    else% For non-pre-int method, we need interpolation.
        %idend = 6*nIMUdata;
        if( PreIntegration_options.bInitPnF5VoU == 1 )
            % Interpolation for the other poses, assuming uniform motion
            [X_obj, xcol] = fn_LinearInterpPoses( nPoses, ABGimu, Timu, ImuTimestamps, X_obj, xcol );
        end
    end

    
    % Intial values of features at the initial IMU pose
    % Given Pf1c, Pf1u can be calculated as:
    % Pf1u = Ru2c'*Pf1c + Tu2c
    %idstart = idend + 1; 
    %idend = idend + 3*nPts;
    %     load([Data_config.DATA_DIR 'PBAFeature.mat']);
    if( PreIntegration_options.bInitPnF5VoU == 1 )
        actualNumFeatures = size( RptFidSet );
        for fidx=1:actualNumFeatures
            fid = RptFidSet(fidx);
            tv = Feature3D(fid).triangs(1).p3D ; 
            Pf1u = SLAM_Params.Ru2c' * tv + SLAM_Params.Tu2c;

            X_obj.feature(fidx).xyz = Pf1u;
            X_obj.feature(fidx).col = (1:3) + xcol;     xcol = xcol + 3;
        end
        X_obj.feature(actualNumFeatures+1:end) = [];%delete unnecessary features             
    end
    %   Fill in ground truth of features
    if( PreIntegration_options.bMalaga == 1 )
        tv = PBAFeature( RptFidSet, : )'; %% Global ids %only pickup repeated features
        Pf1u = SLAM_Params.Ru2c' * tv + repmat(SLAM_Params.Tu2c, 1, nPts);
        
    elseif( PreIntegration_options.bDinuka == 1 )
        
        load([Data_config.DATA_DIR 'feature_pos.mat']);
        tv = feature_pos( RptFidSet, : )';
        abg10 = ( gtIMUposes(selpids(1), 2:4) )'; % Rotation of the IMU pose corresponding to the first key frame
        R10 = fn_RFromABG( abg10(1, 1), abg10(2, 1), abg10(3, 1) );
        Pf1u = R10' * ( tv - repmat( (gtIMUposes(selpids(1),5:7))', 1, size(RptFidSet,1) ) );
        clearvars gtIMUposes            
        
    end
    
    numFeatures = size(Pf1u, 2);
    for  i = 1:numFeatures%size(xg.feature, 1)
        Xg_obj.feature(i).xyz = Pf1u(:, i);
        Xg_obj.feature(i).col = (1:3) + xg_col; xg_col = xg_col + 3;
    end
    Xg_obj.feature(numFeatures+1:end) = [];
        
    %     x(idstart:idend) = Pf1u(:);
    %% Initial values of velocity
    %     clearvars PBAFeature tv
    if( PreIntegration_options.bUVonly == 0 )
        if( PreIntegration_options.bInitPnF5VoU == 1 )
            [ X_obj, xcol ] = fn_CalcVFromKposes( nIMUdata, ImuTimestamps, nIMUrate, dtIMU, nPoses, imufulldata, inertialDelta, SLAM_Params, X_obj, xcol);   
        elseif( PreIntegration_options.bPreInt == 1 )
           %% idend = idend + 3*nPoses;
        else
           %% idend = idend + 3*(nIMUdata+1); 
        end
        %         if(bPreInt == 1)            
        %             idstart = idend + 1;
        %             idend = idend + 3; 
        %             % The velocity of the first pose.
        %             x(idstart:idend, 1) = (x(4:6,1)-0.5*dtIMU(2)*dtIMU(2)*g0-dp(:,2))/(dtIMU(2));               
        %             pidloopmax = nPoses - 1;
        %             for(pid=2:pidloopmax)
        %               idstart = idend + 1;
        %               idend = idend + 3;
        %               Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
        %               x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dtIMU(pid+1)...
        %                   *dtIMU(pid+1)*g0-Ri'*dp(:,(pid+1)))/(dtIMU(pid+1));%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
        %             end
        %             % The velocity of the last pose.
        %             idstart = idend + 1;
        %             idend = idend + 3;
        %             Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
        %             x(idstart:idend) = x((idstart-3):(idend-3), 1)+dtIMU(nPoses)*g0+Ri'*dv(:,nPoses);             
        % %             idp1s = (nPoses-1)*6+nPts*3+1;
        % %             x((idp1s):(idp1s+2)) = x((idp1s+3):(idp1s+2+3));            
        %         else
        %             %idend = idend + 3*nIMUdata;%(nPoses-1)*nIMUrate
        %             idstart = idend + 1;
        %             idend = idend + 3; 
        %             % The velocity of the first pose.IMUparking6L
        %             if(bDinuka == 1)
        %                 imufulldata = [imufulldata(:,1), imufulldata(:,5:7), imufulldata(:,2:4)];% ts, fb, wb
        %             end
        %             x(idstart:idend, 1) = (x(4:6,1)-0.5*dt*dt*g0-0.5*dt*dt*((imufulldata(ImuTimestamps(1), 2:4))'-bf0))/dt; 
        %             pidloopmax = nIMUdata;
        %             for(pid=2:pidloopmax)
        %               idstart = idend + 1;
        %               idend = idend + 3;
        %               Ri = fnR5ABG(x(6*(pid-2)+1),x(6*(pid-2)+2),x(6*(pid-2)+3));
        %               x(idstart:idend, 1) = (x((6*(pid-1)+4):(6*(pid-1)+6), 1) - x((6*(pid-2)+4):(6*(pid-2)+6), 1)-0.5*dt...
        %                   *dt*g0-Ri'*0.5*dt*dt*((imufulldata(ImuTimestamps(1)+pid-1, 2:4))'-bf0))/dt;%(x(((pid-1)+4):((pid-1)+6)) - x(((pid-2)+4):((pid-2)+6)))/dtIMU(pid);              
        %             end
        %             % The velocity of the last pose.
        %             idstart = idend + 1;
        %             idend = idend + 3;
        %             Ri = fnR5ABG(x(6*(nPoses-2)+1),x(6*(nPoses-2)+2),x(6*(nPoses-2)+3));
        %             x(idstart:idend) = x((idstart-3):(idend-3), 1)+dt*g0+Ri'*dt*((imufulldata(ImuTimestamps(1)+nIMUdata, 2:4))'-bf0);            
        %         end        
        %         x(idstart:idend) = 0; % A better way is to calculate an average speed based on two consecutive keyframes
        %% Intial values of g 
        %idstart = idend + 1; idend = idend + 3;
        if ( PreIntegration_options.bInitPnF5VoU == 1 )
            %x(idstart:idend) = g0;%[0,0,-9.8]';
            X_obj.g.val = SLAM_Params.g0;
            X_obj.g.col = (1:3) + xcol;     xcol = xcol + 3;
        end
        %xg(idstart:idend) = g_true;
        Xg_obj.g.val = SLAM_Params.g_true;
        Xg_obj.g.col = (1:3) + xg_col;     xg_col = xg_col + 3;

    end
    %% Au2c, Tu2c
    %idstart = idend + 1; idend = idend + 6;
    if( PreIntegration_options.bInitPnF5VoU == 1 )
        %x(idstart:idend) = [Au2c;Tu2c];
        X_obj.Au2c.val = SLAM_Params.Au2c;
        X_obj.Au2c.col = (1:3) + xcol;  xcol = xcol + 3;
        X_obj.Tu2c.val = SLAM_Params.Tu2c;
        X_obj.Tu2c.col = (1:3) + xcol;  xcol = xcol + 3;
    end
    %xg(idstart:idend) = [Au2c;Tu2c];
    Xg_obj.Au2c.val = SLAM_Params.Au2c;
    Xg_obj.Au2c.col = (1:3) + xg_col;  xg_col = xg_col + 3;
    Xg_obj.Tu2c.val = SLAM_Params.Tu2c;
    Xg_obj.Tu2c.col = (1:3) + xg_col;  xg_col = xg_col + 3;
    
    if( PreIntegration_options.bUVonly == 0)
        %% bf, bw
        if( PreIntegration_options.bVarBias == 0)
            %idstart = idend + 1; idend = idend + 6;
            if( PreIntegration_options.bInitPnF5VoU == 1)
                %x(idstart:idend) = [bf0;bw0];%zeros(6,1);  
                X_obj.Bf.val = SLAM_Params.bf0;
                X_obj.Bf.col = (1:3) + xcol;    xcol = xcol + 3;
                X_obj.Bw.val = SLAM_Params.bw0;
                X_obj.Bw.col = (1:3) + xcol;    xcol = xcol + 3;
            end
            %xg(idstart:idend) = [bf_true;bw_true];
            Xg_obj.Bf.val = SLAM_Params.bf_true;
            Xg_obj.Bf.col = (1:3) + xg_col;    xg_col = xg_col + 3;
            Xg_obj.Bw.val = SLAM_Params.bw_true;
            Xg_obj.Bw.col = (1:3) + xg_col;    xg_col = xg_col + 3;
            
        else
            %idstart = idend + 1; idend = idend + 6*(nPoses-1);
            if(bInitPnF5VoU == 1)
                %x(idstart:idend) = repmat([bf0;bw0],nPoses-1, 1);%zeros(6,1); 
                for pid = 2 : nPoses
                    X_obj.Bf(pid-1).val = SLAM_Params.bf_0; %zeros(3,1);
                    X_obj.Bf(pid-1).col = (1:3) + xcol;     xcol = xcol + 3;
                    X_obj.Bw(pid-1).val = SLAM_Params.bw_0; %zeros(3,1);
                    X_obj.Bw(pid-1).col = (1:3) + xcol;     xcol = xcol + 3;
                end
            end
            %xg(idstart:idend) = repmat([bf_true;bw_true],nPoses-1, 1);
            for pid = 2 : nPoses
                Xg_obj.Bf(pid-1).val = SLAM_Params.bf_true;
                Xg_obj.Bf(pid-1).col = (1:3) + xg_col;     xg_col = xg_col + 3;
                Xg_obj.Bw(pid-1).val = SLAM_Params.bw_true;
                Xg_obj.Bw(pid-1).col = (1:3) + xg_col;     xg_col = xg_col + 3;
            end
        end
    end         
    
    %xg = xg(1:idend);
    if(PreIntegration_options.bUVonly == 0)
        if ( PreIntegration_options.bDinuka == 1)
            load( Data_config.gtVelfulldir );
            if(PreIntegration_options.bPreInt == 1)
                %idstart = (nPoses-1)*6+3*nPts+1;
                %idend = (nPoses-1)*6+3*nPts+3*nPoses;
                tv = (true_vel(ImuTimestamps, 2:end))';
            else
                %idstart = nIMUdata*6+3*nPts+1;
                %idend = nIMUdata*6+3*nPts+3*(nIMUdata+1);
                tv = (true_vel(ImuTimestamps(1):ImuTimestamps(nPoses), 2:end))';
            end        
            %xg(idstart:idend) = tv(:);
            for pid = 1 : nPoses
                Xg_obj.velocity(pid).xyz = (true_vel(ImuTimestamps(pid), 2:end))';
                Xg_obj.velocity(pid).col = (1:3) + xg_col;  xg_col = xg_col + 3;
            end
        else
            if( PreIntegration_options.bPreInt == 1 )
                %idend = (nPoses-1)*6+3*nPts;
            else
                %idend = nIMUdata*6+3*nPts;
            end
            %[xg] = fnCalV5Kposes(bPreInt, xg, nPoses, dtIMU, idend, dp, dv, g0, bf0, imufulldata);
            [ Xg_obj, xg_col ] = fn_CalcVFromKposes( ...
                                        nIMUdata, ImuTimestamps, nIMUrate, dtIMU, nPoses, ...
                                        imufulldata, inertialDelta, SLAM_Params, Xg_obj, xg_col );
        end 
    end

    if( PreIntegration_options.bInitPnF5VoU == 0 )
        %x = xg;
        X_obj = Xg_obj;
        if( PreIntegration_options.bAddInitialNoise == 1 )
            noise_all = 1e-2*(rand(xcol, 1) - 0.5);
            nid = 0;
            for p = 1:length(X_obj.pose)
                X_obj.pose(p).ang.val = noise_all(nid+1:nid+3);     nid = nid + 3;
                X_obj.pose(p).trans.xyz = noise_all(nid+1:nid+3);     nid = nid + 3;
            end
            for f = 1:length(X_obj.feature)
                X_obj.feature(f).xyz = noise_all(nid+1:nid+3);     nid = nid + 3;
            end
            for p = 1:length(X_obj.velocity)
                X_obj.velocity(p).xyz = noise_all(nid+1:nid+3);     nid = nid + 3;
            end
            X_obj.g.val = noise_all(nid+1:nid+3);     nid = nid + 3;
            X_obj.Au2c.val = noise_all(nid+1:nid+3);     nid = nid + 3;
            X_obj.Tu2c.val = noise_all(nid+1:nid+3);     nid = nid + 3;
            for p = 1:length(X_obj.Bf)
                X_obj.Bf(p).val = noise_all(nid+1:nid+3);     nid = nid + 3;
                X_obj.Bw(p).val = noise_all(nid+1:nid+3);     nid = nid + 3;
            end            
        end
    else
        %x = x(1:(size(xg,1)));
    end
