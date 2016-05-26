function J = fn_Jacobian_dImu_dX(J, dtIMU, Jd, nPoses, nPts, nIMUrate, nIMUdata, X, Zobs )%g, 

    global PreIntegration_options Data_config

%% Find Jacobian for dp, dv and dphi
%% dp = Ru1 * (Tu2-Tu1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
%% dv = Ru1 * (v2-v1-g*dt) - ddvdbf*dbf - ddvdbw*dbw;
%% [a,b,g] = fnABG5R(Ru2*(Ru1)');
%% dphi = [a;b;g] - ddphidbw*dbw;
% K: camera model
% p3d0: 3D points at the first camera pose
% x: current estimate of the states (alpha, beta, gamma, T)
    
    g = X.g.val;
        
    dt = 1.0/nIMUrate;
    
    tid = 0;
    
    for pid = 1 : nIMUdata %(nPoses-1)*nIMUrate
        
        tid = tid + 1;
        
        vi = X.velocity( pid ).xyz;
        vi1 = X.velocity( pid + 1 ).xyz;
        if ( pid > 1 )
            angVec = X.pose(pid - 1).ang.val;
            alpha = angVec(1); beta = angVec(2); gamma = angVec(3);
            Ti = X.pose(pid - 1).trans.xyz;
        else
            alpha = 0; beta = 0; gamma = 0;
            Ti = zeros(3,1);
        end
        [drda, drdb, drdg] = fn_ABG2R_dr(alpha, beta, gamma);
        phii = [alpha; beta; gamma];
        Ri = fn_RFromABG(alpha, beta, gamma);
        
        %% 1. wi = Ei*(phii1-phii)/dt+bw;
        Ei = Jac_ko( phii );
        angVec = X.pose(pid).ang.val;
        alpha = angVec(1); beta = angVec(2); gamma = angVec(3);
        phii1 = [alpha; beta; gamma];
        dEdAl = fn_dEdAlpha( phii );
        dEdBe = fn_dEdBeta( phii );
        dEdGa = fn_dEdGamma( phii );
        dwidEa = dEdAl * ( phii1 - phii );
        dwidEb = dEdBe * ( phii1 - phii );
        dwidEg = dEdGa * ( phii1 - phii );
        dwidE = [dwidEa, dwidEb, dwidEg];
        
        dwidphii = ( dwidE - Ei ) / dt;        
        dwidphii1 = Ei / dt;
        dwidbw = eye( 3 );
        
        row = Zobs.imu(pid).w.row;

        %% 1.1 d_w[ij] / d_A[ij]
        if( pid > 1 )
            col = X.pose(pid - 1).ang.col;
            J.dImu_dX(tid).dW_dX.dW_dPhi.val = dwidphii(:);
            J.dImu_dX(tid).dW_dX.dW_dPhi.row = repmat( row(:), 1, length(col) ); 
            J.dImu_dX(tid).dW_dX.dW_dPhi.col = repmat( (col(:))', length(row), 1 );                 
        else
            J.dImu_dX(tid).dW_dX.dW_dPhi = []; % delete if not used
        end

        %% 1.2 d_w[ij] / d_A[i+1,j]
        col = X.pose(pid).ang.col;
        J.dImu_dX(tid).dW_dX.dW_dPhi_1.val = dwidphii1(:);
        J.dImu_dX(tid).dW_dX.dW_dPhi_1.row = repmat( row(:), 1, length(col) ); %[ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
        J.dImu_dX(tid).dW_dX.dW_dPhi_1.col = repmat( (col(:))', length(row), 1 ); %[ col; col; col ];

        %% 1.3 d_w[ij] / d_bw
        col = X.Bw.col;
        J.dImu_dX(tid).dW_dX.dW_dBw.val = eye(3);
        J.dImu_dX(tid).dW_dX.dW_dBw.row = repmat( row(:), 1, length(col) );
        J.dImu_dX(tid).dW_dX.dW_dBw.col = repmat( (col(:))', length(row), 1 );
            
        
        
        %% 2. ai = Ri*((vi1-vi)/dt-g)-bf;
        daidvi1 = Ri/dt;
        daidvi = -Ri/dt;
        daidg = -Ri;
        daidbf = eye(3);
        % fill in Jacobian        
        row = Zobs.imu(pid).acc.row;
        
        %%2.1 d_a[i] / d_A[i,j]
        if(pid > 1)
            daidal = drda * ( (vi1 - vi ) / dt - g );
            daidbe = drdb * ( (vi1 - vi ) / dt - g );
            daidga = drdg * ( (vi1 - vi ) / dt - g );
            daidabg = [daidal, daidbe, daidga];

            col = X.pose(pid - 1).ang.col;
            J.dImu_dX(tid).dAcc_dX.dAcc_dPhi.val = daidabg(:);
            J.dImu_dX(tid).dAcc_dX.dAcc_dPhi.row = repmat( row(:), 1, length(col) );
            J.dImu_dX(tid).dAcc_dX.dAcc_dPhi.col = repmat( (col(:))', length(row), 1 );                
        else
            J.dImu_dX(tid).dAcc_dX.dAcc_dPhi = [];
        end

        %% 2.1 d_a[ij] / d_v[ij]
        col = X.velocity(pid).col;            
        J.dImu_dX(tid).dAcc_dX.dAcc_dV.val = daidvi(:);
        J.dImu_dX(tid).dAcc_dX.dAcc_dV.row = repmat( row(:), 1, length(col) );
        J.dImu_dX(tid).dAcc_dX.dAcc_dV.col = repmat( (col(:))', length(row), 1 );

        %% 2.2 d_a[ij] / d_v[i+1,j]
        col = X.velocity(pid+1).col;            
        J.dImu_dX(tid).dAcc_dX.dAcc_dV_1.val = daidvi1(:);
        J.dImu_dX(tid).dAcc_dX.dAcc_dV_1.row = repmat( row(:), 1, length(col) );
        J.dImu_dX(tid).dAcc_dX.dAcc_dV_1.col = repmat( (col(:))', length(row), 1 );

        %% 2.3 d_a[ij] / d_g
        col = X.g.col;            
        J.dImu_dX(tid).dAcc_dX.dAcc_dG.val = daidg(:);
        J.dImu_dX(tid).dAcc_dX.dAcc_dG.row = repmat( row(:), 1, length(col) );
        J.dImu_dX(tid).dAcc_dX.dAcc_dG.col = repmat( (col(:))', length(row), 1 );

        %% 2.4 d_a[ij] / d_Bf
        col = X.Bf.col;
        J.dImu_dX(tid).dAcc_dX.dAcc_dBf.val = daidbf(:);
        J.dImu_dX(tid).dAcc_dX.dAcc_dBf.row = repmat( row(:), 1, length(col) );
        J.dImu_dX(tid).dAcc_dX.dAcc_dBf.col = repmat( (col(:))', length(row), 1 );
            
        
        %% 3. bzero = Ti1-Ti-vi*dt;
        Ti1 = X.pose(pid).trans.xyz;
        
        dbzdti1 = eye(3);
        dbzdti = -eye(3);
        dbzdvi = -eye(3)*dt;
        
        row = Zobs.imu(pid).deltaT.row;

        %% 3.1 d_bZ[ij] / d_T[i,j]
        if(pid > 1)
            col = X.pose(pid-1).trans.col;
            J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT.val = dbzdti(:);
            J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT.row = repmat( row(:), 1, length(col) );            
            J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT.col = repmat( (col(:))', length(row), 1 );
        else
            J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT = [];
        end

        %% 3.2 d_bZ[ij] / d_T[i+1,j]
        col = X.pose(pid).trans.col;
        J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT_1.val = dbzdti1(:);
        J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT_1.row = repmat( row(:), 1, length(col) );            
        J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dT_1.col = repmat( (col(:))', length(row), 1 );                

        %% 3.1 d_bZ[ij] / d_v[i,j]
        col = X.velocity(pid).col;
        J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dV.val = dbzdvi(:);
        J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dV.row = repmat( row(:), 1, length(col) );            
        J.dImu_dX(tid).dDeltaT_dX.dDeltaT_dV.col = repmat( (col(:))', length(row), 1 );
        
    end

%idxr = nUV + (nIMUdata)*9;%idr + 4;
%% dg, dAu2c, dTu2c % 
if(PreIntegration_options.bAddZg == 1)
    %% dg
    
    tv = eye(3);
    row = Zobs.g.row;
    col = X.g.col;

    J.dG_dG.val = tv;
    J.dG_dG.row = repmat( row(:), 1, length(col) );
    J.dG_dG.col = repmat( (col(:))', length(row), 1 );
        
end

if(PreIntegration_options.bAddZau2c == 1)
    %% dAu2c % 
    tv = eye(3);
    row = Zobs.Au2c.row;
    col = X.Au2c.col;

    %repmat( row(:), 1, length(col) );
    %repmat( (col(:))', length(row), 1 );
    J.dAu2c_dX.val = tv;
    J.dAu2c_dX.row = repmat( row(:), 1, length(col) );
    J.dAu2c_dX.col = repmat( (col(:))', length(row), 1 );
        
end

if(PreIntegration_options.bAddZtu2c == 1)
    %% dTu2c % 

    tv = eye(3);
    row = Zobs.Tu2c.row;
    col = X.Tu2c.col;

    %repmat( row(:), 1, length(col) );
    %repmat( (col(:))', length(row), 1 );
    J.dTu2c_dX.val = tv;
    J.dTu2c_dX.row = repmat( row(:), 1, length(col) ); 
    J.dTu2c_dX.col = repmat( (col(:))', length(row), 1 );
end

if(PreIntegration_options.bVarBias == 0)
    if(PreIntegration_options.bAddZbf == 1)
        tv = eye(3);
        row = Zobs.Bf.row;
        col = X.Bf.col;

        J.dBf_dX.val = tv;
        J.dBf_dX.row = repmat( row(:), 1, length(col) ); 
        J.dBf_dX.col = repmat( (col(:))', length(row), 1 );
    end
    
    if(PreIntegration_options.bAddZbw == 1)
        %% dbw % 
        %repmat( row(:), 1, length(col) );
        %repmat( (col(:))', length(row), 1 );
        tv = eye(3);
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

