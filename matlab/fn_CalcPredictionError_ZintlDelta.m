function e = fnCalcPredictionError_ZintlDelta(X_obj, Zobs, nPoses, nPts, bf0, bw0, ...
                                    dtIMU, J, nIMUrate, ImuTimestamps )%g, 
    global PreIntegration_options
    
    e = Zobs;    
        
    nIMUdata = ImuTimestamps(nPoses)-ImuTimestamps(1);

    if(PreIntegration_options.bVarBias == 0)
        bf = X_obj.Bf.val;
        dbf = bf - bf0;
        bw = X_obj.Bw.val;
        dbw = bw - bw0;
    end

    g = X_obj.g.val;
    
    Ru1 = eye(3); 
    Tu1 = zeros(3,1);
    
    % Reprojection at each pose
    
    if(PreIntegration_options.bPreInt == 1)
        for pid=2:nPoses 

            if(PreIntegration_options.bVarBias == 1)                    
                idx = ((nPoses-1)*6+nPts*3+3*nPoses+10+(pid-2)*6);
                bf = X_obj(idx:(idx+2),1);
                dbf = bf - bf0;
                bw = X_obj((idx+3):(idx+5),1);
                dbw = bw - bw0;
            end 

            Au = X_obj.pose(pid-1).ang.val;
            alpha = Au(1); beta = Au(2); gamma = Au(3);
            Ru2 = fn_RFromABG(alpha, beta, gamma);
            Tu2 = X_obj.pose(pid-1).trans.val;
            v1 = X_obj.velocity(pid-1).xyz;
            v2 = X_obj.velocity(pid).xyz;
            dt = dtIMU(pid);

            [dp,dv,dphi] = fn_PredictIntlDelta(Tu1,Tu2,Ru1,Ru2,v1,v2,g,dbf,dbw,dt,J{pid});    

            e.intlDelta(pid-1).deltaP.val   = dp - Zobs.intlDelta(pid-1).deltaP.val;
            e.intlDelta(pid-1).deltaV.val   = dv - Zobs.intlDelta(pid-1).deltaV.val;
            e.intlDelta(pid-1).deltaPhi.val = dphi - Zobs.intlDelta(pid-1).deltaPhi.val;                            
            Tu1 = Tu2; Ru1 = Ru2;
        end % for pid
        
    else 
        %% Non Preintegration
        dt = 1.0/nIMUrate;
        cid = 1;
        for pid = 1 : nIMUdata %((nPoses-1)*nIMUrate)
            if( PreIntegration_options.bVarBias == 1 )
                if( pid >= ( ImuTimestamps(cid+1) - ImuTimestamps(1) + 1 ) ) 
                    cid = cid + 1;                    
                end         
                if 1
                else
                    idx = nIMUdata*6+nPts*3+3*(nIMUdata+1)+10 + 6*(cid-1);
                    bf = x(idx:(idx+2),1);
                    %   dbf = bf - bf0;
                    bw = x((idx+3):(idx+5),1);
                    %	dbw = bw - bw0;
                end
            end            
            vi = X_obj.velocity( pid ).xyz;
            vi1 = X_obj.velocity(pid + 1).xyz;
            if(pid > 1)
                angVec = X_obj.pose(pid-1).ang.val;
                alpha = angVec(1); beta = angVec(2); gamma = angVec(3);
                Ti = X_obj.pose(pid-1).trans.val;
            else
                alpha = 0; beta = 0; gamma = 0;
                Ti = zeros(3,1);
            end  
            phii = [alpha;beta;gamma];
            Ri = fn_RFromABG( alpha, beta, gamma);
            % ai
            ai = Ri * ( (vi1 - vi ) / dt - g ) + bf;
            % wi
            Ei = Jac_ko(phii);
            %idx = idx+6;
            angVec = X_obj.pose(pid).ang.val;
            alpha = angVec(1); beta = angVec(2); gamma = angVec(3);
            phii1 = [alpha; beta; gamma];
            Ti1 = X_obj.pose(pid).trans.val;
            wi = Ei * (phii1 - phii) / dt + bw;
            % 0 = Ti1-Ti-vi*dt;
            bzero = Ti1 - Ti - vi*dt;
            e.imu(pid).w.val = wi - Zobs.imu(pid).w.val;
            e.imu(pid).acc.val = ai - Zobs.imu(pid).acc.val;
            e.imu(pid).deltaT.val = bzero - Zobs.imu(pid).deltaT.val;
        end
    end

    %% After IMU observations
    
    if( PreIntegration_options.bAddZg == 1 )
        %% g
        e.g.val = X_obj.g.val - Zobs.g.val;
    end

    if( PreIntegration_options.bAddZau2c == 1 )
        %% Au2c
        e.Au2c.val = X_obj.Au2c.val - Zobs.Au2c.val;
    end
        
    if(PreIntegration_options.bAddZtu2c == 1)
        %% Tu2c
        e.Tu2c.val = X_obj.Tu2c.val - Zobs.Tu2c.val;
    end  
        
    if(PreIntegration_options.bVarBias == 0)
         
        if(PreIntegration_options.bAddZbf == 1)
            %% bf
            e.Bf.val = X_obj.Bf.val - Zobs.Bf.val;
        end
        
        if(PreIntegration_options.bAddZbw == 1)
            %% bw
            e.Bw.val = X_obj.Bw.val - Zobs.Bw.val;
        end
        
    else

        for(pid=2:(nPoses-1))            
            bfi = X_obj.Bf.iter(pid-1).val;
            bwi = X_obj.Bw.iter(pid-1).val;
            bfi1 = X_obj.Bf.iter(pid).val;
            bwi1 = X_obj.Bw.iter(pid).val;
            e.Bf.iter(pid-1).val = (bfi-bfi1) - Zobs.Bf.iter(pid-1);
            e.Bw.iter(pid-1).val = bwi-bwi1 - Zobs.Bw.iter(pid-1);
        end
    end
