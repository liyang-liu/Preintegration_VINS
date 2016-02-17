function [xg, fscaleGT] = fnGetXgroundtruth_general(xg, datadir, nPoseNew, ...
                ImuTimestamps, gtIMUposes, selpids, nPts, PBAFeature, ...
                RptFidSet, dtIMU, nIMUrate, nIMUdata, imufulldata, ...
                dp, dv, Au2c, Ru2c, Tu2c, ...
                gtVelfulldir, g_true, bf_true, bw_true)

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
                Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
                Rimu = Ru2c'*Rcam*Ru2c;
                [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
                Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
            end   
            tv = [ABGimu(:, 2:end); Timu(:, 2:end)];
            if(bPreInt == 1)
               xg(1:((nPoseNew-1)*6)) = tv(:);
            else
               [xg] = fnLinearInterpPoses(nPoseNew, ABGimu, Timu, ImuTimestamps, xg);
            end
        elseif(InertialDelta_options.bDinuka == 1)
            tv = (gtIMUposes(selpids(1:(nPoseNew)), 2:7))';
            ABGimu = tv(1:3, :);
            Timu = tv(4:6, :);
            Tcam = zeros(3, nPoseNew);
            for(pid=1:nPoseNew)
                % Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
                Rimu = fnR5ABG(ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid));
                Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + Rimu'*Tu2c);
            end
            if(InertialDelta_options.bPreInt == 1)
                [xg] = fnCal9RelativePoses(xg, nPoseNew, tv);
            else
                pall = (gtIMUposes(selpids(1):(nIMUdata+selpids(1)), 2:7))';
                [xg] = fnCal9RelativePoses(xg, nIMUdata+1, pall);
            end
        end  
        
    %%   Fill in ground truth of features
        if(InertialDelta_options.bPreInt == 1)
           idend = 6*(nPoseNew-1); 
        else
           idend = 6*nIMUdata; 
        end
        idstart = idend + 1; 
        idend = idend + 3*nPts;    
        if(InertialDelta_options.bMalaga == 1)
            tv = PBAFeature(RptFidSet, :)'; %% Global ids %only pickup repeated features
            Pf1u = Ru2c'*tv + repmat(Tu2c, 1, nPts);
        elseif(InertialDelta_options.bDinuka == 1)
            load([datadir 'feature_pos.mat']);
            tv = feature_pos(RptFidSet, :)';
            abg10 = (gtIMUposes(selpids(1), 2:4))'; % Rotation of the IMU pose corresponding to the first key frame
            R10 = fnR5ABG(abg10(1, 1), abg10(2, 1), abg10(3, 1));
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
                if(InertialDelta_options.bPreInt == 1)
                    idstart = idend + 1;%(nPoseNew-1)*6+3*nPts
                    idend = idend + 3*nPoseNew;%(nPoseNew-1)*6+3*nPts
                    tv = (true_vel(ImuTimestamps(1:nPoseNew), 2:end))';
                else
                    idstart = idend + 1;%nIMUdata*6+3*nPts
                    idend = idend +3*(nIMUdata+1);% nIMUdata*6+3*nPts
                    tv = (true_vel(ImuTimestamps(1):ImuTimestamps(nPoseNew), 2:end))';
                end        
                %xg(idstart:idend) = tv(:);
                for i = 1:size(xg.velocity, 1)
                    xg.velocity(i).xyz = tv( (i-1)*3 + 1 : (i-1)*3 + 3)';
                end
            else
                [xg,idend] = fnCalV5Kposes(nIMUdata, ImuTimestamps, ...
                    nIMUrate, xg, nPoseNew, dtIMU, idend, ...
                    dp, dv, g_true, bf_true, imufulldata);
            end 
        end
        
        if(InertialDelta_options.bUVonly == 0)
            %% g
            %idstart = idend + 1; idend = idend + 3;
            %xg(idstart:idend) = g_true;
            xg.g = g_true;
        end
         %% Au2c, Tu2c
        %idstart = idend + 1; idend = idend + 6;
        %xg(idstart:idend) = [Au2c;Tu2c];
        xg.Au2c = Au2c;
        xg.Tu2c = Tu2c;
        if(InertialDelta_options.bUVonly == 0)
            %% bf, bw
            if(InertialDelta_options.bVarBias == 0)
                %idstart = idend + 1; idend = idend + 6;
                %xg(idstart:idend) = [bf_true;bw_true];
                xg.Bf = bf_true;
                xg.Bw = bw_true;
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
        
        