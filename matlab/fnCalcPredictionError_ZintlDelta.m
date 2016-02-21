function e = fnCalcPredictionError_ZintlDelta(x, Zobs, nPoses, nPts, bf0, bw0, ...
                                    dtIMU, J, nIMUrate, ImuTimestamps )%g, 
    global InertialDelta_options
    
    e = Zobs;    
        
    nIMUdata = ImuTimestamps(nPoses)-ImuTimestamps(1);

    if(InertialDelta_options.bVarBias == 0)
        bf = x.Bf.val;
        dbf = bf - bf0;
        bw = x.Bw.val;
        dbw = bw - bw0;
    end

    g = x.g.val;
    
    Ru1 = eye(3); 
    Tu1 = zeros(3,1);
    
    % Reprojection at each pose
    
    for pid=2:nPoses 

        if(InertialDelta_options.bVarBias == 1)                    
            idx = ((nPoses-1)*6+nPts*3+3*nPoses+10+(pid-2)*6);
            bf = x(idx:(idx+2),1);
            dbf = bf - bf0;
            bw = x((idx+3):(idx+5),1);
            dbw = bw - bw0;
        end 

        Au = x.pose(pid-1).ang.val;
        alpha = Au(1); beta = Au(2); gamma = Au(3);
        Ru2 = fnRFromABG(alpha, beta, gamma);
        Tu2 = x.pose(pid-1).trans.val;
        v1 = x.velocity(pid-1).xyz;
        v2 = x.velocity(pid).xyz;
        dt = dtIMU(pid);
        
        [dp,dv,dphi] = fnPredictIntlDelta(Tu1,Tu2,Ru1,Ru2,v1,v2,g,dbf,dbw,dt,J{pid});    
        
        e.intlDelta(pid-1).deltaP.val   = dp - Zobs.intlDelta(pid-1).deltaP.val;
        e.intlDelta(pid-1).deltaV.val   = dv - Zobs.intlDelta(pid-1).deltaV.val;
        e.intlDelta(pid-1).deltaPhi.val = dphi - Zobs.intlDelta(pid-1).deltaPhi.val;                            
        Tu1 = Tu2; Ru1 = Ru2;
    end % for pid


    %% After IMU observations
    id1x = (nPoses-1)*9+1;
    tid = (nPoses-1)*6+nPts*3+3*nPoses+1;
    
    if(InertialDelta_options.bAddZg == 1)
        %% g
        e.g.val = x.g.val - Zobs.g.val;
    end

    if(InertialDelta_options.bAddZau2c == 1)
        %% Au2c
        e.Au2c.val = x.Au2c.val - Zobs.Au2c.val;
    end
        
    if(InertialDelta_options.bAddZtu2c == 1)
        %% Tu2c
        e.Tu2c.val = x.Tu2c.val - Zobs.Tu2c.val;
    end  
        
    if(InertialDelta_options.bVarBias == 0)
         
        if(InertialDelta_options.bAddZbf == 1)
            %% bf
            e.Bf.val = x.Bf.val - Zobs.Bf.val;
        end
        
        if(InertialDelta_options.bAddZbw == 1)
            %% bw
            e.Bw.val = x.Bw.val - Zobs.Bw.val;
        end
        
    else

        idx_bf0 = tid + 9;
        for(pid=2:(nPoses-1))
            bfi = x((idx_bf0+(pid-2)*6):(idx_bf0+(pid-2)*6)+2);
            bwi = x((idx_bf0+3+(pid-2)*6):(idx_bf0+(pid-2)*6+5));
            bfi1 = x((idx_bf0+6+(pid-2)*6):(idx_bf0+(pid-2)*6+8));
            bwi1 = x((idx_bf0+9+(pid-2)*6):((idx_bf0+(pid-2)*6+11)));
            e(id1x:(id1x+5)) = [bfi-bfi1;bwi-bwi1] - Zobs(id1x:(id1x+5));
            id1x = id1x + 6;
        end
    end
