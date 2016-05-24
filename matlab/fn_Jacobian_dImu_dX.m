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
    if 0
        tmpv = zeros(1, nJacs);
        tidend = 0;

        idx_g = 6*(nIMUdata)+nPts*3+3*(nIMUdata+1);
        idx_au2c = idx_g + 3;%nIMUrate*(nPoses-1) ((nPoses-1)*nIMUrate+1)(nPoses-1)*6+nPts*3+3*nPoses+4;
        %g = x((idx_g+1):(idx_g+3));
    end
    
    g = X.g.val;
    
    if 0
        idx_bf = idx_au2c + 6;%;(nPoses-1)*6+nPts*3+3*nPoses+10;
        idx_bw = idx_bf + 3;%(nPoses-1)*6+nPts*3+3*nPoses+13;
    end
    
    dt = 1.0/nIMUrate;
    
    tid = 0;
    
    for pid = 1 : nIMUdata %(nPoses-1)*nIMUrate
        
        tid = tid + 1;
        
        %%idr = (pid-1)*9;%nIMUrate* nUV + 
        
        %idx = (nIMUdata)*6+3*nPts+(pid-1)*3; 
        %idx_vi = idx;
        %vi = x((idx+1):(idx+3));
        vi = X.velocity( pid ).xyz;
        %idx_vi1 = idx_vi + 3;
        %vi1 = x((idx+4):(idx+6));
        vi1 = X.velocity( pid + 1 ).xyz;
        if(pid > 1)
            %idx = (pid-2)*6;
            %idx_abgi = idx;
            %alpha = x(idx+1); beta = x(idx+2); gamma = x(idx+3);
            %Ti = x((idx+4):(idx+6));
            angVec = X.pose(pid - 1).ang.val;
            alpha = angVec(1); beta = angVec(2); gamma = angVec(3);
            Ti = X.pose(pid - 1).trans.val;
            %idx_ti = idx + 3;
            %[drda, drdb, drdg] = fabg2r_dr(alpha, beta, gamma);
        else
            alpha = 0; beta = 0; gamma = 0;
            %idx = -6;
            Ti = zeros(3,1);
            %drda = zeros(3); drdb = zeros(3); drdg = zeros(3);
        end
        [drda, drdb, drdg] = fn_ABG2R_dr(alpha, beta, gamma);
        phii = [alpha; beta; gamma];
        Ri = fn_RFromABG(alpha, beta, gamma);
        
        %% 1. wi = Ei*(phii1-phii)/dt+bw;
        Ei = Jac_ko( phii );
        %idx = idx+6;
        %alpha = x(idx+1); beta = x(idx+2); gamma = x(idx+3);
        angVec = X.pose(pid).ang.val;
        alpha = angVec(1); beta = angVec(2); gamma = angVec(3);
        %idx_abgi1 = idx;
        phii1 = [alpha; beta; gamma];
        % J of wi = Ei * ( phii1 - phii ) / dt + bw;
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
        
        if 1
            row = Zobs.imu(pid).w.row;
            
            %% 1.1 d_w[ij] / d_A[ij]
            if(pid > 1)
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
            
        else
            if(pid > 1)
                %	J((idr+1):(idr+3), (idx_abgi+1):(idx_abgi+3)) = dwidphii;
                tidstart = tidend + 1; tidend = tidend + 3*3;
                tmpv(tidstart:tidend) = dwidphii(:);
            end
            %   J((idr+1):(idr+3), (idx_abgi1+1):(idx_abgi1+3)) = dwidphii1;
            %   J((idr+1):(idr+3), (idx_bw+1):(idx_bw+3)) = dwidbw;
            %   idr = idr + 3;
            tidstart = tidend + 1; tidend = tidend + 3*3*2;
            tmpv(tidstart:tidend) = [ dwidphii1(:); dwidbw(:) ];
        end
        
        
        %% 2. ai = Ri*((vi1-vi)/dt-g)-bf;
        daidvi1 = Ri/dt;
        daidvi = -Ri/dt;
        daidg = -Ri;
        daidbf = eye(3);
        % fill in Jacobian
        if 1
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
            
        else
            
            %% 2.1 d_a[ij] / d_A[ij]
            if(pid > 1)
                daidal = drda*((vi1-vi)/dt-g);
                daidbe = drdb*((vi1-vi)/dt-g);
                daidga = drdg*((vi1-vi)/dt-g);
                daidabg = [daidal,daidbe,daidga];
                %   J((idr+1):(idr+3), (idx_abgi+1):(idx_abgi+3)) = daidabg;
                tidstart = tidend + 1; tidend = tidend + 3*3;
                tmpv(tidstart:tidend) = daidabg(:);
            end
            
            %% 2.1 d_a[ij] / d_v[ij], d_a[ij] / d_v[i+1,j], d_a[ij] / d_g, d_a[ij] / d_Bf
            %   J((idr+1):(idr+3), (idx_vi+1):(idx_vi+3)) = daidvi;
            %   J((idr+1):(idr+3), (idx_vi1+1):(idx_vi1+3)) = daidvi1;
            %   J((idr+1):(idr+3), (idx_g+1):(idx_g+3)) = daidg;
            %   J((idr+1):(idr+3), (idx_bf+1):(idx_bf+3)) = daidbf;
            %   idr = idr + 3;
            tidstart = tidend + 1; tidend = tidend + 3*3*4;
            tmpv(tidstart:tidend) = [daidvi(:);daidvi1(:);daidg(:);daidbf(:)];
            
        end
        
        %% 3. bzero = Ti1-Ti-vi*dt;
        %idx = idx + 3;        
        %Ti1 = x((idx+1):(idx+3));
        Ti1 = X.pose(pid).trans.val;
        
        %idx_ti1 = idx;
        % J of bzero = Ti1-Ti-vi*dt;
        dbzdti1 = eye(3);
        dbzdti = -eye(3);
        dbzdvi = -eye(3)*dt;
        
        if 1
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
            
        else
            %	J((idr+1):(idr+3), (idx_ti1+1):(idx_ti1+3)) = dbzdti1;
            if(pid > 1)
                %	J((idr+1):(idr+3), (idx_ti+1):(idx_ti+3)) = dbzdti;
                tidstart = tidend + 1; tidend = tidend + 3*3;
                tmpv(tidstart:tidend) = dbzdti(:);
            end
            %   J((idr+1):(idr+3), (idx_vi+1):(idx_vi+3)) = dbzdvi;
            tidstart = tidend + 1; tidend = tidend + 3*3*2;
            tmpv(tidstart:tidend) = [dbzdti1(:);dbzdvi(:)];        
        end
        
    end

%idxr = nUV + (nIMUdata)*9;%idr + 4;
%% dg, dAu2c, dTu2c % 
if(PreIntegration_options.bAddZg == 1)
    %% dg
    
    if 1
        tv = eye(3);
        row = Zobs.g.row;
        col = X.g.col;

        %repmat( row(:), 1, length(col) );
        %repmat( (col(:))', length(row), 1 );

        J.dG_dG.val = tv;
        J.dG_dG.row = repmat( row(:), 1, length(col) );
        J.dG_dG.col = repmat( (col(:))', length(row), 1 );
        
    else
        %	idxc = idx_g;%idx_au2c;%       
        %   J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
        %   idxr = idxr + 3;
        tidstart = tidend + 1; tidend = tidend + 3*3;
        tv = eye(3);
        tmpv(tidstart:tidend) = tv(:);
    end
end
if(PreIntegration_options.bAddZau2c == 1)
    %% dAu2c % 
    if 1
        tv = eye(3);
        row = Zobs.Au2c.row;
        col = X.Au2c.col;
        
        %repmat( row(:), 1, length(col) );
        %repmat( (col(:))', length(row), 1 );
        J.dAu2c_dX.val = tv;
        J.dAu2c_dX.row = repmat( row(:), 1, length(col) );
        J.dAu2c_dX.col = repmat( (col(:))', length(row), 1 );
        
    else
        %	idxc = idx_au2c;% 
        %   J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
        %   idxr = idxr + 3; 
        tidstart = tidend + 1; tidend = tidend + 3*3;
        tv = eye(3);
        tmpv(tidstart:tidend) = tv(:);     
    end
end

if(PreIntegration_options.bAddZtu2c == 1)
    %% dTu2c % 
    if 1
        tv = eye(3);
        row = Zobs.Tu2c.row;
        col = X.Tu2c.col;
        
        %repmat( row(:), 1, length(col) );
        %repmat( (col(:))', length(row), 1 );
        J.dTu2c_dX.val = tv;
        J.dTu2c_dX.row = repmat( row(:), 1, length(col) ); 
        J.dTu2c_dX.col = repmat( (col(:))', length(row), 1 );
        
    else
        %   idxc = idx_au2c + 3;% 
        %   J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
        %   idxr = idxr + 3; 
        tidstart = tidend + 1; tidend = tidend + 3*3;
        tv = eye(3);
        tmpv(tidstart:tidend) = tv(:);     
    end
end

if(PreIntegration_options.bVarBias == 0)
    if(PreIntegration_options.bAddZbf == 1)
        if 1
            tv = eye(3);
            row = Zobs.Bf.row;
            col = X.Bf.col;
            
            J.dBf_dX.val = tv;
            J.dBf_dX.row = repmat( row(:), 1, length(col) ); 
            J.dBf_dX.col = repmat( (col(:))', length(row), 1 );
        else
            %% dbf % 
            %     idxc = idx_bf;% 
            %     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
            %     idxr = idxr + 3; 
            tidstart = tidend + 1; tidend = tidend + 3*3;
            tv = eye(3);
            tmpv(tidstart:tidend) = tv(:);     
        end
    end
    
    if(PreIntegration_options.bAddZbw == 1)
        %% dbw % 
        if 1
            %repmat( row(:), 1, length(col) );
            %repmat( (col(:))', length(row), 1 );
            tv = eye(3);
            row = Zobs.Bw.row;
            col = X.Bw.col;
            
            J.dBw_dX.val = tv;
            J.dBw_dX.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3); row(3) * ones(1, 3) ]; 
            J.dBw_dX.col = [col; col; col];

        else
            %     idxc = idx_bw;% 
            %     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
            %     idxr = idxr + 3;    
            tidstart = tidend + 1; tidend = tidend + 3*3;
            tv = eye(3);
            tmpv(tidstart:tidend) = tv(:); 
        end
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

if 0
    J = sparse(idRow, idCol, tmpv);
    % 
    % % idxr = idxr +3;
    % % J(idxr:(idxr+2), idxc:(idxc+2)) = dvidvi;
    % % 
    % % %% dbf, dbw
    % % dbfdbf = eye(3);
    % % idxr = idxr + 3;
    % % J(idxr:(idxr+2), idx_bf:(idx_bf+2)) = dbfdbf;
end