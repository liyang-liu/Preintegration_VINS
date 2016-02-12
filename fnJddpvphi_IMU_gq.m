function J = fnJddpvphi_IMU_gq(idRow, idCol, nJacs, nUV, dtIMU, Jd, nPoses, nPts, x )%g, 
    
    global InertialDelta_options
    
    %% Find Jacobian for dp, dv and dphi
    %% dp = Ru1 * (Tu2-Tu1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
    %% dv = Ru1 * (v2-v1-g*dt) - ddvdbf*dbf - ddvdbw*dbw;
    %% [a,b,g] = fnABG5R(Ru2*(Ru1)');
    %% dphi = [a;b;g] - ddphidbw*dbw;
    % K: camera model
    % p3d0: 3D points at the first camera pose
    % x: current estimate of the states (alpha, beta, gamma, T)

    tmpv = zeros(1, nJacs);
    tidend = 0;

    if(InertialDelta_options.bUVonly == 1)
        
        idx_au2c = (nPoses-1)*6+nPts*3+1;
        %     idxr = nUV + 1;%size(J) - 6 + 1;
    
    else
        
        idx_au2c = (nPoses-1)*6+nPts*3+3*nPoses+4;
        idx_g = (nPoses-1)*6+nPts*3+3*nPoses+1;
        g = x(idx_g:(idx_g+2));
        idx_bf = (nPoses-1)*6+nPts*3+3*nPoses+10;
        idx_bw = (nPoses-1)*6+nPts*3+3*nPoses+13;

        R1 = eye(3); T1 = zeros(3,1);
        % a1 = 0; b1 = 0; g1 = 0;
        drda1 = zeros(3); drdb1 = zeros(3); drdg1 = zeros(3);
        idx_v1 = (nPoses-1)*6+nPts*3+1;
        v1 = x(idx_v1:(idx_v1+2));
        
        for pid = 2:nPoses
            
            dt = dtIMU(pid);
            idx_a2 = (pid-2)*6+1;
            a2 = x(idx_a2); b2 = x(idx_a2+1); g2 = x(idx_a2+2);
            R2 = fnR5ABG(a2,b2,g2);
            T2 = x((idx_a2+3):(idx_a2+5),1);
            idx_v2 = (nPoses-1)*6+nPts*3+(pid-1)*3+1;
            v2 = x(idx_v2:(idx_v2+2));
            jddpdbf = Jd{pid}(1:3, 10:12); jddpdbw = Jd{pid}(1:3, 13:15);
            jddvdbf = Jd{pid}(4:6, 10:12); jddvdbw = Jd{pid}(4:6, 13:15);
            jddphidbw = Jd{pid}(7:9, 13:15);              
            idxr = nUV + (pid-2)*9 + 1;% 2*nPts*nPoses   
            
            % 1. dp = R1 * (T2-T1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
            ddpdt2 = R1;
            ddpdv1 = -R1 * dt;
            ddpdgr = -0.5 * R1 * dt * dt;
            ddpdbf = -jddpdbf;
            ddpdbw = -jddpdbw;
            
            if(pid > 2)
        
                ddpda = drda1 * (T2-T1-v1*dt-0.5*g*dt*dt);%3x1
                ddpdb = drdb1 * (T2-T1-v1*dt-0.5*g*dt*dt);
                ddpdg = drdg1 * (T2-T1-v1*dt-0.5*g*dt*dt);
                ddpdabg1 = [ddpda,ddpdb,ddpdg];% 3x3
                ddpdt1 = -R1;        
                
                tidstart = tidend + 1; tidend = tidend + 3*3*2;
                tmpv(tidstart:tidend) = [(ddpdabg1(:))', (ddpdt1(:))'];
            end 
            
            tidstart = tidend + 1; tidend = tidend + 3*3*5;
            tmpv(tidstart:tidend) = [(ddpdt2(:))',(ddpdv1(:))',(ddpdgr(:))',(ddpdbf(:))',(ddpdbw(:))'];

            % 2. dv = R1 * (v2-v1-g*dt) - jddvdbf*(bf-bf0) - jddvdbw*(bw-bw0);    
            ddvdv1 = -R1;
            ddvdv2 = R1;
            ddvdg = -R1 * dt;
            ddvdbf = -jddvdbf;
            ddvdbw = -jddvdbw;
            
            %% Fill in ddv
            idxr = idxr + 3; 
            if(pid > 2)
                % dabg1
                ddvdabg1 = [drda1 * (v2-v1-g*dt), drdb1 * (v2-v1-g*dt), drdg1 * (v2-v1-g*dt)];
                tidstart = tidend + 1; tidend = tidend + 3*3*1;
                tmpv(tidstart:tidend) = (ddvdabg1(:))';
            end
            
            tidstart = tidend + 1; tidend = tidend + 3*3*5;
            tmpv(tidstart:tidend) = [(ddvdv1(:))', (ddvdv2(:))', (ddvdg(:))', (ddvdbf(:))', (ddvdbw(:))'];
            
            %% 3. [a,b,g] = fnABG5R(Ru2*(Ru1)'); dphi = [a;b;g] - ddphidbw*dbw;
            idxr = idxr + 3;    
            R12 = R2*(R1)';
            if(pid > 2)		
                ddphida1 = fndABGdABG(R2*(drda1'), R12);
                ddphidb1 = fndABGdABG(R2*(drdb1'), R12);
                ddphidg1 = fndABGdABG(R2*(drdg1'), R12);
                tidstart = tidend + 1; tidend = tidend + 3*3*1;
                tmpv(tidstart:tidend) = [(ddphida1(:))', (ddphidb1(:))', (ddphidg1(:))'];
            end 
            
            [drda2, drdb2, drdg2] = fabg2r_dr(a2, b2, g2);
            ddphida2 = fndABGdABG(drda2*R1', R12);
            ddphidb2 = fndABGdABG(drdb2*R1', R12);
            ddphidg2 = fndABGdABG(drdg2*R1', R12);
            ddphidbw = -jddphidbw;	
            
            tidstart = tidend + 1; tidend = tidend + 3*3*2;
            tmpv(tidstart:tidend) = [(ddphida2(:))', (ddphidb2(:))', (ddphidg2(:))', (ddphidbw(:))'];

        % 	a1 = a2; b1 = b2; g1 = g2;
            drda1 = drda2; drdb1 = drdb2; drdg1 = drdg2;
            T1 = T2;R1 = R2;
            v1 = v2; idx_v1 = idx_v2;
            
        end

        %% After IMU observations

        idxr = nUV + (nPoses-1)*9 + 1;%2*nPts*nPoses
        
        if(InertialDelta_options.bAddZg == 1)
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            tmpv(tidstart:tidend) = (tv(:))';
        end
    end

    if(InertialDelta_options.bAddZau2c == 1)
        %% dAu2c % 
        tidstart = tidend + 1; tidend = tidend + 3*3*1;
        tv = eye(3);
        tmpv(tidstart:tidend) = (tv(:))';   
    end
    
    if(InertialDelta_options.bAddZtu2c == 1)
        %% dTu2c % 
        tidstart = tidend + 1; tidend = tidend + 3*3*1;
        tv = eye(3);
        tmpv(tidstart:tidend) = (tv(:))';     
    end

    if(InertialDelta_options.bUVonly == 1)
        %% dT2 % 
        tidstart = tidend + 1; tidend = tidend + 1;
        tmpv(tidstart:tidend) = 1;    
        
    elseif(InertialDelta_options.bVarBias == 0)
        
        if(InertialDelta_options.bAddZbf == 1)            
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            tmpv(tidstart:tidend) = (tv(:))';  
        end
        
        if(InertialDelta_options.bAddZbw == 1)
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            tmpv(tidstart:tidend) = (tv(:))'; 
        end        
    else
        
        for(pid = 2:(nPoses-1))
            
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            tmpv(tidstart:tidend) = (tv(:))'; %dbfi
            
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = -eye(3);
            tmpv(tidstart:tidend) = (tv(:))'; %dbfi1
            
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            tmpv(tidstart:tidend) = (tv(:))'; %dbwi
            
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = -eye(3);
            tmpv(tidstart:tidend) = (tv(:))'; %dbwi1       
            
        end
        
    end

    J = sparse(idRow, idCol, tmpv);
