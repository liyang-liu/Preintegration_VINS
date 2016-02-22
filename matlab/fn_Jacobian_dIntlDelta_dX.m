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

    tid = 0;
    
        
    g = X.g.val;
    bf = X.Bf.val;
    bw = X.Bw.val;

    R1 = eye(3); T1 = zeros(3,1);
    % a1 = 0; b1 = 0; g1 = 0;
    drda1 = zeros(3); drdb1 = zeros(3); drdg1 = zeros(3);
    
    v1 = X.velocity(1).xyz;

    for pid = 2:nPoses

        tid = tid + 1;

        dt = dtIMU(pid);
        Au2 = X.pose(pid-1).ang.val;
        a2 = Au2(1); b2 = Au2(2); g2 = Au2(3);
        R2 = fn_RFromABG(a2,b2,g2);
        T2 = X.pose(pid-1).trans.val;

        v2 = X.velocity(pid).xyz;

        jddpdbf = Jd{pid}(1:3, 10:12); jddpdbw = Jd{pid}(1:3, 13:15);
        jddvdbf = Jd{pid}(4:6, 10:12); jddvdbw = Jd{pid}(4:6, 13:15);
        jddphidbw = Jd{pid}(7:9, 13:15);              

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
        if(pid > 2)
            ddvdabg1 = [drda1 * (v2-v1-g*dt), drdb1 * (v2-v1-g*dt), drdg1 * (v2-v1-g*dt)];
            col = (X.pose(pid-2).ang.col(:))';
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1.val = (ddvdabg1(:))';                
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1.col = [ col; col; col ];
        else
            J.dIntlDelta_dX(tid).dDv_dX.dDv_dAbg_1 = [];
        end

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

        R12 = R2*(R1)';                        
        if(pid > 2)		
            ddphida1 = fn_dABGdABG(R2*(drda1'), R12);
            ddphidb1 = fn_dABGdABG(R2*(drdb1'), R12);
            ddphidg1 = fn_dABGdABG(R2*(drdg1'), R12);
            
            col = (X.pose(pid-2).ang.col(:))';
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_1.val = ...
                        [(ddphida1(:))', (ddphidb1(:))', (ddphidg1(:))'];
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_1.col = [col; col; col];
        else
            J.dIntlDelta_dX(tid).dPhi_dX.dDphi_dAbg1 = [];
        end 

        [drda2, drdb2, drdg2] = fn_ABG2R_dr(a2, b2, g2);
        ddphida2 = fn_dABGdABG(drda2*R1', R12);
        ddphidb2 = fn_dABGdABG(drdb2*R1', R12);
        ddphidg2 = fn_dABGdABG(drdg2*R1', R12);
        ddphidbw = -jddphidbw;	

        col = (X.pose(pid-1).ang.col(:))';
        J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_2.val = ...
                    [(ddphida2(:))', (ddphidb2(:))', (ddphidg2(:))'];
        J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_2.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dAbg_2.col = [col; col; col];


        col = (X.Bw.col(:))';
        J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dBw.val = (ddphidbw(:))';
        J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dBw.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dIntlDelta_dX(tid).dDphi_dX.dDphi_dBw.col = [col; col; col];

        drda1 = drda2; drdb1 = drdb2; drdg1 = drdg2;
        T1 = T2;R1 = R2;
        v1 = v2;
    end

    %% After IMU observations

    if(InertialDelta_options.bAddZg == 1)
        tv = eye(3);
        row = Zobs.g.row;
        col = X.g.col;

        J.dG_dG.val = tv;
        J.dG_dG.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dG_dG.col = [col; col; col];

    end
    

    if(InertialDelta_options.bAddZau2c == 1)
        %% dAu2c % 
        tv = eye(3);
        row = Zobs.Au2c.row;
        col = X.Au2c.col;
        
        J.dAu2c_dX.val = tv;
        J.dAu2c_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dAu2c_dX.col = [col; col; col];
    end
    
    if(InertialDelta_options.bAddZtu2c == 1)
        %% dTu2c % 
        tv = eye(3);
        row = Zobs.Tu2c.row;
        col = X.Tu2c.col;
        
        J.dTu2c_dX.val = tv;
        J.dTu2c_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dTu2c_dX.col = [col; col; col];
    end

        
    if(InertialDelta_options.bVarBias == 0)
        
        if(InertialDelta_options.bAddZbf == 1)            
            tv = eye(3);
            row = Zobs.Bf.row;
            col = X.Bf.col;
            
            J.dBf_dX.val = tv;
            J.dBf_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dBf_dX.col = [col; col; col];
        end
        
        if(InertialDelta_options.bAddZbw == 1)
            tv = eye(3);
            row = Zobs.Bw.row;
            col = X.Bw.col;
            
            J.dBw_dX.val = tv;
            J.dBw_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dBw_dX.col = [col; col; col];
        end        
        
    else
        
        for(pid = 2:(nPoses-1))
            row = Zobs.Bf.row;
            
            tv = eye(3);
            col = X.Bf.iter(pid-1).col;
            J.dBf_dX.iter(pid-1).dX1.val = tv;
            J.dBf_dX.iter(pid-1).dX1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ];
            J.dBf_dX.iter(pid-1).dX1.col = [col; col; col];
            
            tv = -eye(3);
            col = X.Bf.iter(pid).col;
            J.dBf_dX.iter(pid-1).dX2.val = tv;
            J.dBf_dX.iter(pid-1).dX2.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ];
            J.dBf_dX.iter(pid-1).dX2.col = [ col; col; col ];
        end
            
        for(pid = 2:(nPoses-1))
            row = Zobs.bw.row;            
            tv = eye(3);
            col = X.Bw.iter(pid-1).col;
            J.dBw_dX.iter(pid-1).dX1.val = tv;
            J.dBw_dX.iter(pid-1).dX1.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ];
            J.dBw_dX.iter(pid-1).dX1.col = [col; col; col];
            
            tv = -eye(3);
            col = X.Bw.iter(pid).col;
            J.dBw_dX.iter(pid-1).dX2.val = tv;
            J.dBw_dX.iter(pid-1).dX2.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ];
            J.dBw_dX.iter(pid-1).dX2.col = [ col; col; col ];
        end        
    end

