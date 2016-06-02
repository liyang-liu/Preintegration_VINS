function [R0imu, T0imu, v0imu] = fnCaluRTV0(bUVonly, bPreInt, nPoseOld, ImuTimestamps, ...
                x_old, RptFidSet_old)
        
        if(nPoseOld == 1)
            R0imu = eye(3);
            T0imu = zeros(3,1);
            v0imu = zeros(3,1);
        elseif(nPoseOld > 1)
            nPts_old = size(RptFidSet_old,1);
            nIMUdata_old = ImuTimestamps(nPoseOld) - ImuTimestamps(1);
            if(bPreInt == 1)
                R0imu = zeros(3,3,nPoseOld);
                R0imu(:,:,1) = eye(3);
                T0imu = zeros(3,nPoseOld);
                v0imu = zeros(3,nPoseOld);                
                pid_end = nPoseOld;
                idx_v = 6*(nPoseOld-1)+3*nPts_old;
            else    
                R0imu = zeros(3,3,nIMUdata_old+1);                
                T0imu = zeros(3,nIMUdata_old+1);
                v0imu = zeros(3,nIMUdata_old+1);                
                pid_end = nIMUdata_old+1;
                idx_v = 6*nIMUdata_old+3*nPts_old;
            end
            R0imu(:,:,1) = eye(3);            
            for(pid=2:pid_end)
                idx = 6*(pid-2);
                R0imu(:, :, pid) = fnR5ABG(x_old(idx+1), x_old(idx+2), x_old(idx+3));
                T0imu(:, pid) = x_old((idx+4):(idx+6)); 
                if(bUVonly == 0)
                    v0imu(:, pid) = x_old((idx_v+1):(idx_v+3)); 
                    idx_v = idx_v + 3;
                end
            end
        end