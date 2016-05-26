function e = fnCalcPredictionError_ZintlDelta(x, Zobs, nPoses, nPts, bf0, bw0, ...
                                    dtIMU, J, nIMUrate, ImuTimestamps )%g, 
    global PreIntegration_options
    
    %e = zeros(size(Zobs));
    e = Zobs;
    if(PreIntegration_options.bUVonly == 0)
        
        nIMUdata = ImuTimestamps(nPoses)-ImuTimestamps(1);
        
        if(PreIntegration_options.bVarBias == 0)
            
            if(PreIntegration_options.bPreInt == 1)
               idx = ((nPoses-1)*6+nPts*3+3*nPoses+10);
            else
               idx = (nIMUdata*6+nPts*3+3*(nIMUdata+1)+10); 
            end
            
            %bf = x(idx:(idx+2),1);
            bf = x.Bf.val;
            dbf = bf - bf0;
            %bw = x((idx+3):(idx+5),1);
            bw = x.Bw.val;
            dbw = bw - bw0;
        end
        
        if(PreIntegration_options.bPreInt == 1)
            idx = (nPoses-1)*6+nPts*3+3*nPoses+1;
        else
            idx = (nIMUdata*6+nPts*3+3*(nIMUdata+1)+1);
        end
        
        %g = x(idx:(idx+2),1);
        g = x.g.val;
        
    end
    
    Ru1 = eye(3); 
    Tu1 = zeros(3,1);
    
    % Reprojection at each pose
    
    if(PreIntegration_options.bUVonly == 0)
        if(PreIntegration_options.bPreInt == 1)
            
            for pid=2:nPoses 
                
                if(PreIntegration_options.bVarBias == 1)                    
                    idx = ((nPoses-1)*6+nPts*3+3*nPoses+10+(pid-2)*6);
                    bf = x(idx:(idx+2),1);
                    dbf = bf - bf0;
                    bw = x((idx+3):(idx+5),1);
                    dbw = bw - bw0;
                end 
                
                idx = ((pid-2)*6+1);%(nPoses-1)*6+nPts*3+3*nPoses + 9
                %alpha = x(idx);beta = x(idx + 1); gamma = x(idx + 2);
                Au = x.pose(pid-1).ang.val;
                alpha = Au(1); beta = Au(2); gamma = Au(3);
                Ru2 = fnRFromABG(alpha, beta, gamma);
                %Tu2 = x((idx+3):(idx+5));
                Tu2 = x.pose(pid-1).trans.xyz;
                %idx = ((nPoses-1)*6+nPts*3+(pid-2)*3+1);
                %v1 = x(idx:(idx+2)); v2 = x((idx+3):(idx+5));
                v1 = x.velocity(pid-1).xyz;
                v2 = x.velocity(pid).xyz;
                dt = dtIMU(pid);
                [dp,dv,dphi] = fnPredictIntlDelta(Tu1,Tu2,Ru1,Ru2,v1,v2,g,dbf,dbw,dt,J{pid});    
                %idx = (pid-2)*9+1;
                %e(idx:(idx+8)) = [dp;dv;dphi] - Zobs(idx:(idx+8));
                e.intlDelta(pid-1).deltaP.val   = dp - Zobs.intlDelta(pid-1).deltaP.val;
                e.intlDelta(pid-1).deltaV.val   = dv - Zobs.intlDelta(pid-1).deltaV.val;
                e.intlDelta(pid-1).deltaPhi.val = dphi - Zobs.intlDelta(pid-1).deltaPhi.val;                            
                Tu1 = Tu2; Ru1 = Ru2;
            end % for pid
            
        else % not pre integration
            
            dt = 1.0/nIMUrate;
            cid = 1;
            
            for pid=1:nIMUdata%((nPoses-1)*nIMUrate)
                
                if(PreIntegration_options.bVarBias == 1)
                    
                    if(pid >= (ImuTimestamps(cid+1)-ImuTimestamps(1)+1)) 
                        cid = cid + 1;                    
                    end         
                    idx = nIMUdata*6+nPts*3+3*(nIMUdata+1)+10 + 6*(cid-1);
                    bf = x(idx:(idx+2),1);
    
                    bw = x((idx+3):(idx+5),1);
    
                end % if
                
                idx = (nIMUdata*6+3*nPts+(pid-1)*3); 
                vi = x((idx+1):(idx+3));
                vi1 = x((idx+4):(idx+6));
                
                if(pid > 1)
                    idx = ((pid-2)*6);%nIMUrate*
                    alpha = x(idx+1); beta = x(idx+2); gamma = x(idx+3);
                    Ti = x((idx+4):(idx+6));
                else
                    alpha = 0; beta = 0; gamma = 0;
                    idx = -6;
                    Ti = zeros(3,1);
                end  
                
                phii = [alpha;beta;gamma];
                Ri = fnRFromABG(alpha,beta,gamma);
                % ai
                ai = Ri*((vi1-vi)/dt-g)+bf;
                % wi
                Ei = Jac_ko(phii);
                idx = idx+6;
                alpha = x(idx+1); beta = x(idx+2); gamma = x(idx+3);
                phii1 = [alpha;beta;gamma];
                Ti1 = x((idx+4):(idx+6));
                wi = Ei*(phii1-phii)/dt+bw;
                
                % 0 = Ti1-Ti-vi*dt;
                bzero = Ti1-Ti-vi*dt;
                idx = (pid-1)*9+1;
                if(idx > 20700)
                    tt = 1;
                end
                
                e(idx:(idx+8)) = [wi;ai;bzero] - Zobs(idx:(idx+8));
                
            end % for pid
        end % else, not pre-integration

        %% After IMU observations
        if((PreIntegration_options.bPreInt == 1))% && ((PreIntegration_options.bAddZg == 1) || (PreIntegration_options.bAddZtu2c == 1)  || (PreIntegration_options.bAddZau2c == 1)|| (PreIntegration_options.bAddZantu2c == 1) || (PreIntegration_options.bAddZbf == 1)))
            id1x = (nPoses-1)*9+1;
            tid = (nPoses-1)*6+nPts*3+3*nPoses+1;
        else
            id1x = nIMUdata*3*3+1;%(nPoses-1)*nIMUrate
            tid = nIMUdata*6+nPts*3+3*(nIMUdata+1)+1;
        end
        
    else % if UVOnly
        
        id1x = 1;
        tid = (nPoses-1)*6+nPts*3+1-3;% -3 for agreement.
        
    end
    
    if(PreIntegration_options.bUVonly == 0)
        if(PreIntegration_options.bAddZg == 1)
            %% g
            %idx_g = tid;
            %g = x(idx_g:(idx_g+2));
            %e(id1x:(id1x+2)) = g - Zobs(id1x:(id1x+2));
            %id1x = id1x + 3;
            e.g.val = x.g.val - Zobs.g.val;
        end
    end
    
    if(PreIntegration_options.bAddZau2c == 1)
        %% Au2c
        %idx_Au2c = tid+3;
        %Au2c = x(idx_Au2c:(idx_Au2c+2));
        %e(id1x:(id1x+2)) = Au2c - Zobs(id1x:(id1x+2)); 
        %id1x = id1x + 3;
        e.Au2c.val = x.Au2c.val - Zobs.Au2c.val;
    end
        
    if(PreIntegration_options.bAddZtu2c == 1)
        %% Tu2c
        %idx_Tu2c = tid+6;
        %Tu2c = x(idx_Tu2c:(idx_Tu2c+2));
        %e(id1x:(id1x+2)) = Tu2c - Zobs(id1x:(id1x+2));
        %id1x = id1x + 3;
        e.Tu2c.val = x.Tu2c.val - Zobs.Tu2c.val;
    end  
        
     if(PreIntegration_options.bUVonly == 1)
        %% A2, T2
            idx_T2 = 6;%4;%for Malaga 1;%
            T2 = x(idx_T2);%:(idx_T2+5));
            e(id1x) = T2 - Zobs(id1x);%:(id1x+5):(id1x+5));
    
     elseif(PreIntegration_options.bVarBias == 0)
         
        if(PreIntegration_options.bAddZbf == 1)
        %% bf
            %idx_bf = tid+9;
            %bf = x(idx_bf:(idx_bf+2));
            %e(id1x:(id1x+2)) = bf - Zobs(id1x:(id1x+2)); 
            %id1x = id1x + 3;
            e.Bf.val = x.Bf.val - Zobs.Bf.val;
        end
        
        if(PreIntegration_options.bAddZbw == 1)
        %% bw
            %idx_bw = tid+12;
            %bw = x(idx_bw:(idx_bw+2));
            %e(id1x:(id1x+2)) = bw - Zobs(id1x:(id1x+2));        
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
    