function [xg, fscaleGT] = fnGetXgroundtruth_general(xg, datadir, nPoseNew, ...
                ImuTimestamps, gtIMUposes, selpids, nPts, PBAFeature, ...
                RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, ...
                dp, dv, gtVelfulldir, SLAM_Params)

        global InertialDelta_options
            
        %% Poses
        if(InertialDelta_options.bMalaga == 1)
            load([datadir 'PBAPose.mat']);    
            tv = (PBAPose(1:nPoseNew, :))';
            ABGcam = [tv(3, :);tv(2, :);tv(1, :)];%tv(1:3, :);%
            Tcam = tv(4:6, :);           
            clearvars PBAPoses tv
            ABGimu = zeros(3, nPoseNew);
            Timu = zeros(3, nPoseNew);
            % Given Rcam/Tcam, Rimu/Timu can be calculated as follows:
            % Rimu = Ru2c' * Rc1u; Timu = Tc1u - Rimu '* Tu2c;
            % Rc1u = Rcam * Ru2c; Tc1u = Tu2c + Ru2c'*Tcam
            for(pid=1:(nPoseNew))% correspond to pose 1...n
                Rcam = fnRFromABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
                Rimu = SLAM_Params.Ru2c' * Rcam * SLAM_Params.Ru2c;
                [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABGFromR(Rimu);
                Timu(:, pid) = SLAM_Params.Tu2c + SLAM_Params.Ru2c' * Tcam(:, pid) - Rimu' * SLAM_Params.Tu2c;
            end   
            tv = [ABGimu(:, 2:end); Timu(:, 2:end)];
           [xg] = fnLinearInterpPoses(nPoseNew, ABGimu, Timu, ImuTimestamps, xg);
           
        elseif(InertialDelta_options.bDinuka == 1)
            tv = (gtIMUposes(selpids(1:(nPoseNew)), 2:7))';
            ABGimu = tv(1:3, :);
            Timu = tv(4:6, :);
            Tcam = zeros(3, nPoseNew);
            for(pid=1:nPoseNew)
                % Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
                Rimu = fnRFromABG(ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid));
                Tcam(:, pid) = SLAM_Params.Ru2c * (Timu(:, pid) - SLAM_Params.Tu2c + Rimu' * SLAM_Params.Tu2c);
            end
            
            pall = (gtIMUposes(selpids(1):(nIMUdata+selpids(1)), 2:7))';
            [xg] = fnCalcLocalRelativePoses(xg, nIMUdata+1, pall);
        end  
        
    %%   Fill in ground truth of features
        idend = 6*nIMUdata; 
        idstart = idend + 1; 
        idend = idend + 3*nPts;    
        if(InertialDelta_options.bMalaga == 1)
            tv = PBAFeature(RptFidSet, :)'; %% Global ids %only pickup repeated features
            Pf1u = SLAM_Params.Ru2c' * tv + repmat(SLAM_Params.Tu2c, 1, nPts);
        elseif(InertialDelta_options.bDinuka == 1)
            load([datadir 'feature_pos.mat']);
            tv = feature_pos(RptFidSet, :)';
            abg10 = (gtIMUposes(selpids(1), 2:4))'; % Rotation of the IMU pose corresponding to the first key frame
            R10 = fnRFromABG(abg10(1, 1), abg10(2, 1), abg10(3, 1));
            Pf1u = R10'*(tv - repmat((gtIMUposes(selpids(1),5:7))', 1,size(RptFidSet,1)));
%             clearvars gtIMUposes            
        end        
        %xg(idstart:idend) = Pf1u(:);        
        %size(xg.feature, 1)
        %size(Pf1u )
        numFeatures = size(Pf1u, 2);
        for  i = 1:numFeatures%size(xg.feature, 1)
            xg.feature(i).xyz = Pf1u(:, i);
        end
        xg.feature(numFeatures+1:end) = [];
        
        %% Velocity
        if(InertialDelta_options.bUVonly == 0)
            if(InertialDelta_options.bDinuka == 1)
                load(gtVelfulldir);
                idstart = idend + 1;%nIMUdata*6+3*nPts
                idend = idend +3*(nIMUdata+1);% nIMUdata*6+3*nPts
                tv = (true_vel(ImuTimestamps(1):ImuTimestamps(nPoseNew), 2:end))';
                %xg(idstart:idend) = tv(:);
                for i = 1:size(xg.velocity, 1)
                    xg.velocity(i).xyz = tv( (i-1)*3 + 1 : (i-1)*3 + 3)';
                end
            else
                [xg,idend] = fnCalVFromKposes(nIMUdata, ImuTimestamps, ...
                    nIMUrate, xg, nPoseNew, dtIMU, idend, ...
                    dp, dv, g_true, bf_true, imufulldata);
            end 
        end
        
        if(InertialDelta_options.bUVonly == 0)
            %% g
            %idstart = idend + 1; idend = idend + 3;
            %xg(idstart:idend) = g_true;
            xg.g.val = SLAM_Params.g_true;
        end
         %% Au2c, Tu2c
        %idstart = idend + 1; idend = idend + 6;
        %xg(idstart:idend) = [Au2c;Tu2c];
        xg.Au2c.val = SLAM_Params.Au2c_true;
        xg.Tu2c.val = SLAM_Params.Tu2c_true;
        if(InertialDelta_options.bUVonly == 0)
            %% bf, bw
            if(InertialDelta_options.bVarBias == 0)
                %idstart = idend + 1; idend = idend + 6;
                %xg(idstart:idend) = [bf_true;bw_true];
                xg.Bf.val = SLAM_Params.bf_true;
                xg.Bw.val = SLAM_Params.bw_true;
            else
                idstart = idend + 1; idend = idend + 6*(nPoseNew-1);
                xg(idstart:idend) = repmat([bf_true;bw_true],nPoseNew-1, 1);
            end
        end
        %xg = xg(1:idend);
        
        
        
        
        
        % Calculate translational scales
        fscaleGT = zeros(nPoseNew, 1);
        for(pid=2:nPoseNew)        
           Timu = Tcam(:, pid) - Tcam(:, pid-1); 
           fscaleGT(pid) = norm(Timu);
        end
        
        