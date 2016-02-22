function J = fnJacobian_dIntlDelta_dX(J, dtIMU, Jd, nPoses, nPts, X, Zobs )%g, 
    %fnJacobian_dIntlDelta_dX(J, idRow, idCol, nJacs, nUV, dtIMU, Jd, nPoses, nPts, x )%g, 
    global InertialDelta_options
    
    %% Find Jacobian for dp, dv and dphi
    %% dp = Ru1 * (Tu2-Tu1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
    %% dv = Ru1 * (v2-v1-g*dt) - ddvdbf*dbf - ddvdbw*dbw;
    %% [a,b,g] = fnABGFromR(Ru2*(Ru1)');
    %% dphi = [a;b;g] - ddphidbw*dbw;
    % K: camera model
    % p3d0: 3D points at the first camera pose
    % X: current estimate of the states (alpha, beta, gamma, T)

    %tmpv = zeros(1, nJacs);
    %tidend = 0;
    tid = 0;
    
    if(InertialDelta_options.bUVonly == 1)
        
        idx_au2c = (nPoses-1)*6+nPts*3+1;
        %     idxr = nUV + 1;%size(J) - 6 + 1;
    
    else
        
        %idx_au2c = (nPoses-1)*6 + nPts*3 + 3*nPoses + 4;
        %idx_g = (nPoses-1)*6 + nPts*3 + 3*nPoses + 1;
        %g = x(idx_g:(idx_g+2));
        %idx_bf = (nPoses-1)*6+nPts*3+3*nPoses+10;
        %idx_bw = (nPoses-1)*6+nPts*3+3*nPoses+13;
        g = X.g.val;
        bf = X.Bf.val;
        bw = X.Bw.val;
        
        R1 = eye(3); T1 = zeros(3,1);
        % a1 = 0; b1 = 0; g1 = 0;
        drda1 = zeros(3); drdb1 = zeros(3); drdg1 = zeros(3);
        %idx_v1 = (nPoses-1)*6 + nPts*3 + 1;
        %v1 = x(idx_v1:(idx_v1+2));
        v1 = X.velocity(1).xyz;
        
        for pid = 2:nPoses
            
            tid = tid + 1;
            
            dt = dtIMU(pid);
            %idx_a2 = (pid-2)*6+1;
            %a2 = x(idx_a2); b2 = x(idx_a2+1); g2 = x(idx_a2+2);
            Au2 = X.pose(pid-1).ang.val;
            a2 = Au2(1); b2 = Au2(2); g2 = Au2(3);
            R2 = fnRFromABG(a2,b2,g2);
            %T2 = x((idx_a2+3):(idx_a2+5),1);
            T2 = X.pose(pid-1).trans.val;
            
            %idx_v2 = (nPoses-1)*6 + nPts*3 + (pid-1)*3 + 1;
            %v2 = x(idx_v2:(idx_v2+2));
            v2 = X.velocity(pid).xyz;
            
            jddpdbf = Jd{pid}(1:3, 10:12); jddpdbw = Jd{pid}(1:3, 13:15);
            jddvdbf = Jd{pid}(4:6, 10:12); jddvdbw = Jd{pid}(4:6, 13:15);
            jddphidbw = Jd{pid}(7:9, 13:15);              
            %idxr = nUV + (pid-2)*9 + 1;% 2*nPts*nPoses   
            
            % 1. dp = R1 * (T2-T1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
            ddpdt2 = R1;
            ddpdv1 = -R1 * dt;
            ddpdgr = -0.5 * R1 * dt * dt;
            ddpdbf = -jddpdbf;
            ddpdbw = -jddpdbw;
            
            row = Zobs.intlDelta(tid).deltaP.row;
            
            if(pid > 2)
        
                ddpda = drda1 * (T2-T1-v1*dt-0.5*g*dt*dt);%3x1
                ddpdb = drdb1 * (T2-T1-v1*dt-0.5*g*dt*dt);
                ddpdg = drdg1 * (T2-T1-v1*dt-0.5*g*dt*dt);
                ddpdabg1 = [ddpda,ddpdb,ddpdg];% 3x3
                ddpdt1 = -R1;        
                
                %tidstart = tidend + 1; tidend = tidend + 3*3*2;
                %tmpv(tidstart:tidend) = [(ddpdabg1(:))', (ddpdt1(:))'];
                
                col = (X.pose(pid-2).ang.col(:))';
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dAbg_1.val = (ddpdabg1(:))';
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dAbg_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dAbg_1.col = [ col; col; col ];
                
                col = (X.pose(pid-2).trans.col(:))';
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_1.val = (ddpdt1(:))';
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_1.col = [ col; col; col ];
                
            else                
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dAbg_1 = [];                
                J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_1 = [];
            end 
            
            %tidstart = tidend + 1; tidend = tidend + 3*3*5;
            %tmpv(tidstart:tidend) = [(ddpdt2(:))',(ddpdv1(:))',(ddpdgr(:))',(ddpdbf(:))',(ddpdbw(:))'];
            col = (X.pose(pid-1).trans.col(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_2.val = (ddpdt2(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_2.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dTrans_2.col = [ col; col; col ];
            
            col = (X.velocity(pid-1).col(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dV_1.val = (ddpdv1(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dV_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dV_1.col = [ col; col; col ];
            
            col = (X.g.col(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dG.val = (ddpdgr(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dG.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dG.col = [ col; col; col ];
            
            
            col = (X.Bf.col(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dBf.val = (ddpdbf(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dBf.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dBf.col = [ col; col; col ];
            
            col = (X.Bw.col(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dBw.val = (ddpdbw(:))';
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dBw.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDp_dX.dDp_dBw.col = [ col; col; col ];
            
            % 2. dv = R1 * (v2-v1-g*dt) - jddvdbf*(bf-bf0) - jddvdbw*(bw-bw0);    
            ddvdv1 = -R1;
            ddvdv2 = R1;
            ddvdg = -R1 * dt;
            ddvdbf = -jddvdbf;
            ddvdbw = -jddvdbw;
            
            row = Zobs.intlDelta(tid).deltaV.row;
            
            %% Fill in ddv
            %idxr = idxr + 3; 
            if(pid > 2)
                % dabg1
                ddvdabg1 = [drda1 * (v2-v1-g*dt), drdb1 * (v2-v1-g*dt), drdg1 * (v2-v1-g*dt)];
                %tidstart = tidend + 1; tidend = tidend + 3*3*1;
                %tmpv(tidstart:tidend) = (ddvdabg1(:))';
                col = (X.pose(pid-2).ang.col(:))';
                J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1.val = (ddvdabg1(:))';                
                J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
                J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1.col = [ col; col; col ];
                
            else
                J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1 = [];
            end
            
            %tidstart = tidend + 1; tidend = tidend + 3*3*5;
            %tmpv(tidstart:tidend) = [(ddvdv1(:))', (ddvdv2(:))', (ddvdg(:))', (ddvdbf(:))', (ddvdbw(:))'];
            col = (X.velocity(pid-1).col(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dV_1.val = (ddvdv1(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dV_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dV_1.col = [col; col; col];
            
            col = (X.velocity(pid).col(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dV_2.val = (ddvdv2(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dV_2.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dV_2.col = [col; col; col];
            
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dG.val = (ddvdg(:))';
            col = (X.g.col(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dG.row =  [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dG.col = [col; col; col];
            
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dBf.val = (ddvdbf(:))';
            col = (X.Bf.col(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dBf.row =  [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dBf.col = [col; col; col];
            
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dBw.val = (ddvdbw(:))';
            col = (X.Bw.col(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dBw.row =  [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dBw.col = [col; col; col];
            
            %% 3. [a,b,g] = fnABGFromR(Ru2*(Ru1)'); dphi = [a;b;g] - ddphidbw*dbw;
            row = Zobs.intlDelta(tid).deltaPhi.row;
            
            %idxr = idxr + 3;    
            R12 = R2*(R1)';                        
            if(pid > 2)		
                ddphida1 = fndABGdABG(R2*(drda1'), R12);
                ddphidb1 = fndABGdABG(R2*(drdb1'), R12);
                ddphidg1 = fndABGdABG(R2*(drdg1'), R12);
                %tidstart = tidend + 1; tidend = tidend + 3*3*1;
                %tmpv(tidstart:tidend) = [(ddphida1(:))', (ddphidb1(:))', (ddphidg1(:))'];
                col = (X.pose(pid-2).ang.col(:))';
                J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_1.val = ...
                            [(ddphida1(:))', (ddphidb1(:))', (ddphidg1(:))'];
                J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
                J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_1.col = [col; col; col];
            else
                J.dIntlDelta_dX(tid).dPhi_dX.dDphi_dAbg1 = [];
            end 
            
            [drda2, drdb2, drdg2] = fabg2r_dr(a2, b2, g2);
            ddphida2 = fndABGdABG(drda2*R1', R12);
            ddphidb2 = fndABGdABG(drdb2*R1', R12);
            ddphidg2 = fndABGdABG(drdg2*R1', R12);
            ddphidbw = -jddphidbw;	
            
            %tidstart = tidend + 1; tidend = tidend + 3*3*2;
            %tmpv(tidstart:tidend) = [(ddphida2(:))', (ddphidb2(:))', (ddphidg2(:))', (ddphidbw(:))'];
            col = (X.pose(pid-1).ang.col(:))';
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_2.val = ...
                        [(ddphida2(:))', (ddphidb2(:))', (ddphidg2(:))'];
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_2.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_2.col = [col; col; col];

            
            col = (X.Bw.col(:))';
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dBw.val = (ddphidbw(:))';
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dBw.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dBw.col = [col; col; col];
            
        % 	a1 = a2; b1 = b2; g1 = g2;
            drda1 = drda2; drdb1 = drdb2; drdg1 = drdg2;
            T1 = T2;R1 = R2;
            v1 = v2; %idx_v1 = idx_v2;
            
        end

        %% After IMU observations

        %idxr = nUV + (nPoses-1)*9 + 1;%2*nPts*nPoses
        
        if(InertialDelta_options.bAddZg == 1)
            %tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            row = Zobs.g.row;
            col = X.g.col;
            
            %tmpv(tidstart:tidend) = (tv(:))';
            J.dG_dG.val = tv;
            J.dG_dG.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dG_dG.col = [col; col; col];
            
        end
    end

    if(InertialDelta_options.bAddZau2c == 1)
        %% dAu2c % 
        %tidstart = tidend + 1; tidend = tidend + 3*3*1;
        tv = eye(3);
        row = Zobs.Au2c.row;
        col = X.Au2c.col;
        
        J.dAu2c_dX.val = tv;
        J.dAu2c_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dAu2c_dX.col = [col; col; col];
    end
    
    if(InertialDelta_options.bAddZtu2c == 1)
        %% dTu2c % 
        %tidstart = tidend + 1; tidend = tidend + 3*3*1;
        tv = eye(3);
        %tmpv(tidstart:tidend) = (tv(:))';     
        row = Zobs.Tu2c.row;
        col = X.Tu2c.col;
        
        J.dTu2c_dX.val = tv;
        J.dTu2c_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dTu2c_dX.col = [col; col; col];
    end

    if(InertialDelta_options.bUVonly == 1)
        %% dT2 % 
        tidstart = tidend + 1; tidend = tidend + 1;
        tmpv(tidstart:tidend) = 1;    
        
    elseif(InertialDelta_options.bVarBias == 0)
        
        if(InertialDelta_options.bAddZbf == 1)            
            %tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            %tmpv(tidstart:tidend) = (tv(:))';  
            row = Zobs.Bf.row;
            col = X.Bf.col;
            
            J.dBf_dX.val = tv;
            J.dBf_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dBf_dX.col = [col; col; col];
        end
        
        if(InertialDelta_options.bAddZbw == 1)
            %tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tv = eye(3);
            %tmpv(tidstart:tidend) = (tv(:))'; 
            row = Zobs.Bw.row;
            col = X.Bw.col;
            
            J.dBw_dX.val = tv;
            J.dBw_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dBw_dX.col = [col; col; col];
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

    %J = sparse(idRow, idCol, tmpv);
