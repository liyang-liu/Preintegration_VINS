function [x, RptFidSet, RptFeatureObs, nPts] = fnCompX5odometry( ...
            nPoseOld, nPoseNew, nPoses, nPts, x_old, bIMUodo, ...
            ImuTimestamps, nIMUdata, nIMUdata_old, Feature3D, RptFidSet, ...
            RptFidSet_old, dtIMU, g0, dp, dv, dphi, K, bMalaga, RptFeatureObs, ...
            Tu2c, Au2c, Ru2c, fscaleGT, bSimData, kfids, nIMUrate, bDinuka, ...
            bPreInt, bVarBias, x, bf0, bw0, imufulldata, bUVonly)
   
        nPts_old = size(RptFidSet_old,1);
        if(nPoseOld == 1)
            R0imu = eye(3);
            T0imu = zeros(3,1);
            v0imu = zeros(3,1);
        elseif(nPoseOld > 1)
            nIMUdata_old = ImuTimestamps(nPoseOld) - ImuTimestamps(1);
            R0imu = zeros(3,3,nPoseOld);
            R0imu(:,:,1) = eye(3);
            T0imu = zeros(3,nPoseOld);
            v0imu = zeros(3,nPoseOld);             
            if(bPreInt == 1)               
%                 pid_end = nPoseOld;                
                pids = (1:nPoseOld)';
                idx_v = 6*(nPoseOld-1)+3*nPts_old + 3*(pids-1);
            else    
%                 R0imu = zeros(3,3,nIMUdata_old+1);                
%                 T0imu = zeros(3,nIMUdata_old+1);
%                 v0imu = zeros(3,nIMUdata_old+1);                
%                 pid_end = nIMUdata_old+1;                
                pids = ImuTimestamps(1:nPoseOld) - ImuTimestamps(1)+1;
                idx_v = 6*nIMUdata_old+3*nPts_old + 3*(pids-1);
            end
%             R0imu(:,:,1) = eye(3);            
            for(id=1:size(pids,1))%2:pid_end)
                if(id > 1)
                    pid = pids(id);
                    idx = 6*(pid-2);
                    R0imu(:, :, id) = fnR5ABG(x_old(idx+1), x_old(idx+2), x_old(idx+3));
                    T0imu(:, id) = x_old((idx+4):(idx+6)); 
                end
                if(bUVonly == 0)
                    v0imu(:, id) = x_old((idx_v(id)+1):(idx_v(id)+3)); 
                end
            end
        else
            fprintf('\n nPoseOld == %d? Unexpected!\n', nPoseOld);
            return;
        end    
        % Intial values of poses and IMU states
             if(bIMUodo == 1)
                %% Obtain initial poses from IMU data
%                 if(nPoseOld == 1)
%                     
%                 else
%                     if(bPreInt == 1)
%                         idend = 6*(nPoseOld-1)+3*nPts;
%                     else                        
%                         idend = 6*nIMUdata_old+3*nPts;
%                     end
%                     idstart = idend + 1;
%                     idend = idend + 3;
%                     v0imu = x_old(idstart:idend);                    
%                 end                
%                 [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5IMUdata(dtIMU, g0, dp, dv, dphi,nPoseNew, nPts, ...
%                         K, bSimData, bMalaga, RptFeatureObs, Tu2c, Ru2c);               
                [Rcam, ~, Tcam, vimu, Feature3D, RptFidSet, RptFeatureObs] = fnGetPoses5IMUdata_Inc(nPoseOld, ...
                    nPoseNew, nPoses, R0imu, T0imu, v0imu, dtIMU, g0, dp, dv, dphi, ...
                    K, Feature3D, bMalaga, RptFidSet, RptFeatureObs, Tu2c, Ru2c);
                nPts = size(RptFidSet,1);
            else
                %% obtain relative poses from visual odometry
                [Rcam, ~, Tcam, Feature3D] = fnGetPoses5MatchedFeatures(nPoseNew, nPts, ...
                            K, fscaleGT, bSimData, RptFeatureObs, kfids);  
            end
            ABGimu = zeros(3, nPoseNew);%nPoses+1);
            Timu = zeros(3, nPoseNew);%nPoses+1); % ABGimu = zeros(3, nPoses);            
            for(pid=(nPoseOld+1):nPoseNew)%(nPoses+1))% correspond to pose 1...n
        %         Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
                Rimu = Ru2c'*Rcam(:,:,pid)*Ru2c;
                [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
                Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;
            end

%         idstart = 1; 
%         if((bUVonly == 1) || (bPreInt == 1))
%             idend = 6*(nPoseNew-1);
%             tv = [ABGimu(:, 2:end); Timu(:, 2:end)];% tv = [ABGimu; Timu];        
%             x(idstart:idend) = tv(:);
%             if(nPoseOld > 1)
%                idend_old = 6*(nPoseOld-1); 
%                x(idstart:idend_old) = x_old(idstart:idend_old);
%             end
%         else% For non-pre-int method, we need interpolation.
%             idend = 6*nIMUdata;% only poses at keyframes are available
%             % Interpolation for the other poses, assuming uniform motion
%             [x] = fnLinearInterpPoses(nPoseNew, ABGimu, Timu, ImuTimestamps,x);
%             if(nPoseOld > 1)
%                idend_old = 6*nIMUdata_old; 
%                x(idstart:idend_old) = x_old(idstart:idend_old);
%             end            
%         end
    %% Combine old and new poses into x        
        idend = 0; 
        if(bPreInt == 1)
            if(nPoseOld > 1)
                idstart = idend + 1;                
                idend = idend + 6*(nPoseOld-1); 
                x(idstart:idend) = x_old(idstart:idend); 
            end
            idstart  = idend + 1; 
            idend = 6*(nPoseNew - 1);
            tv = [ABGimu(:,(nPoseOld+1):end); Timu(:,(nPoseOld+1):end)];%
            x(idstart:idend) = tv(:);
        else
            if(nPoseOld > 1)
                idstart = idend + 1; 
                idend = idend + 6*nIMUdata_old; 
                x(idstart:idend) = x_old(idstart:idend);
                idend = idend - 6;
                idstart = idend + 1;
                idend = idend + 6;
                ABGimu0 = x_old(idstart:(idstart+2));
%                 Timu0 = x_old((idstart+3):idend);
%                 vidend = 6*nIMUdata_old + 3*nPts_old + 3*(nIMUdata_old);
%                 vidstart = vidend + 1;
%                 vidend = vidend + 3;
%                 vimu0 = x_old(vidstart:vidend);   
            else
%                 vimu0 = zeros(3,1);
                ABGimu0 = zeros(3,1);
%                 Timu0 = zeros(3,1);
            end
            idstart  = idend + 1; 
            idend = 6*nIMUdata; 
            [upos, uvel] = fnIMUnonPreIntPoses_Inc(nPoseNew, nPoseOld, ABGimu0, ...
                T0imu(:,nPoseOld), v0imu(:, nPoseOld), imufulldata, ... % Timu0, vimu0
                ImuTimestamps, nIMUrate, bf0, bw0, g0);
            x(idstart:idend) = upos;
        end 

        % Intial values of features at the initial IMU pose
        % Given Pf1c, Pf1u can be calculated as:
        % Pf1u = Ru2c'*Pf1c + Tu2c
        idstart = idend + 1; 
        idstart_new = idstart;
        idend = idend + 3*nPts;
        tv =  (Feature3D(RptFidSet, 5:7))';% select the first group      
        Pf1u = Ru2c'*tv + repmat(Tu2c, 1, nPts);                
        x(idstart:idend) = Pf1u(:);
        if(nPoseOld > 1)
            [~, idrptf, idrptf_old] = intersect(RptFidSet, RptFidSet_old);
            if(bPreInt == 1)
                idstart_old = 6*(nPoseOld-1)+1;
            else                        
                idstart_old = 6*nIMUdata_old+1;
            end    
            for(fid=1:size(idrptf,1))
                x((idstart_new+(idrptf(fid)-1)*3):(idstart_new+(idrptf(fid)-1)*3+2)) = x_old(...
                    (idstart_old+(idrptf_old(fid)-1)*3):(idstart_old+(idrptf_old(fid)-1)*3+2));
            end
        end

        %% Initial values of velocity
        if(bUVonly == 0)
%             [x,idend] = fnCalV5Kposes(nIMUdata, ImuTimestamps, nIMUrate, ...
%                 bDinuka, bPreInt, x, nPoseNew, dtIMU, idend, dp, dv, g0, bf0, ...
%                 imufulldata);
            if(nPoseOld > 1)
                if(bPreInt == 1)
                    idstart = idend + 1;
                    idend = idend + 3*nPoseOld;
                    idend_old = 6*(nPoseOld-1) + 3*size(RptFidSet_old,1);
                    idstart_old = idend_old + 1;
                    idend_old = idend_old + 3*nPoseOld;
                    x(idstart:idend) = x_old(idstart_old:idend_old);
                else
                    idstart = idend + 1;
                    idend = idend + 3*(nIMUdata_old+1);
                    idend_old = 6*nIMUdata_old + 3*size(RptFidSet_old,1);
                    idstart_old = idend_old + 1;
                    idend_old = idend_old + 3*(nIMUdata_old+1);
                    x(idstart:idend) = x_old(idstart_old:idend_old);
                end
            end
            [tv,~] = fnCalV5Kposes_Inc(nPoseNew, nPoseOld, ...
                nPoses, nPts, nIMUdata, ImuTimestamps, nIMUrate, bDinuka, ...
                bPreInt, x, dtIMU, dp, dv, g0, bf0, imufulldata);
            if(bPreInt == 1)
%                 if(nPoseOld > 1)
%                     idstart = idend + 1;
%                     idend = idend + 3*nPoseOld;
%                     idend_old = 6*(nPoseOld-1) + 3*size(RptFidSet_old,1);
%                     idstart_old = idend_old + 1;
%                     idend_old = idend_old + 3*nPoseOld;
%                     x(idstart:idend) = x_old(idstart_old:idend_old);
%                 end
                idstart = idend + 1;
                idend = idend + 3*nPoses;
                if(nPoseOld == 1)
                    idend = idend + 3;
                end
                x(idstart:idend) = tv;
            else
%                 if(nPoseOld > 1)
%                     idstart = idend + 1;
%                     idend = idend + 3*(nIMUdata_old+1);
%                     idend_old = 6*nIMUdata_old + 3*size(RptFidSet_old,1);
%                     idstart_old = idend_old + 1;
%                     idend_old = idend_old + 3*(nIMUdata_old+1);
%                     x(idstart:idend) = x_old(idstart_old:idend_old);
%                 end
                idstart = idend + 1;
                if(nPoseOld == 1)
                    idend = idend + 3*(nIMUdata+1);
                else
                    idend = idend + 3*(nIMUdata - nIMUdata_old);
                end
                x(idstart:idend) = uvel;            
            end
            
            % Intial values of g 
            idstart = idend + 1; idend = idend + 3;
            x(idstart:idend) = g0;%[0,0,-9.8]';           
        end
        %% Au2c, Tu2c
        idstart = idend + 1; idend = idend + 6;
        x(idstart:idend) = [Au2c;Tu2c];

        if(bUVonly == 0)
            %% bf, bw
            if(bVarBias == 0)
                idstart = idend + 1; idend = idend + 6;
                x(idstart:idend) = [bf0;bw0];%zeros(6,1);  
            else
                idstart = idend + 1; idend = idend + 6*(nPoseNew-1);                
                x(idstart:idend) = repmat([bf0;bw0],nPoseNew-1, 1);%zeros(6,1); 
            end
        end 
        x = x(1:idend);