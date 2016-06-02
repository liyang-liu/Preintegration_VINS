function J = fnJddpvphi_IMU_gq(idRow, idCol, nJacs, nUV, bUVonly, dtIMU, Jd, nPoses, nPts, x, ...
    bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias)%g, 
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

if(bUVonly == 1)
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
        %     idxr = 3*nPts*nPoses + (pid-2)*9 + 1;
        idxr = nUV + (pid-2)*9 + 1;% 2*nPts*nPoses   
        % 1. dp = R1 * (T2-T1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
        ddpdt2 = R1;
        ddpdv1 = -R1 * dt;
        ddpdgr = -0.5 * R1 * dt * dt;
        ddpdbf = -jddpdbf;
        ddpdbw = -jddpdbw;
        %% fill in Jacobian
        % dT2
%         idx_t2 = (pid - 2)*6 + 4;
%         J(idxr:(idxr+2), idx_t2:(idx_t2+2)) = ddpdt2;
        % v1
%         idx_v11 = (nPoses-1)*6+nPts*3+3*(pid-2) + 1;    
%         J(idxr:(idxr+2),idx_v11:(idx_v11+2)) = ddpdv1;
        % g
    %     idx_g = (nPoses-1)*6+nPts*3+3*nPoses+1;
%         J(idxr:(idxr+2), idx_g:(idx_g+2)) = ddpdg;    
        % bf & bw
    %     idx_bf = (nPoses-1)*6+nPts*3+3*nPoses+10;
%         J(idxr:(idxr+2), idx_bf:(idx_bf+5)) = [ddpdbf,ddpdbw];      
        if(pid > 2)
    %         [drda1, drdb1, drdg1] = fabg2r_dr(a1, b1, g1);
            ddpda = drda1 * (T2-T1-v1*dt-0.5*g*dt*dt);%3x1
            ddpdb = drdb1 * (T2-T1-v1*dt-0.5*g*dt*dt);
            ddpdg = drdg1 * (T2-T1-v1*dt-0.5*g*dt*dt);
            ddpdabg1 = [ddpda,ddpdb,ddpdg];% 3x3
            ddpdt1 = -R1;        
            % dabgT1
%             idx_a1 = (pid-3)*6 + 1;
%             J(idxr:(idxr+2), idx_a1:(idx_a1+5)) = [ddpdabg1, ddpdt1];
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
    %         idxc = (pid-3)*6 + 1;
%             J(idxr:(idxr+2), idx_a1:(idx_a1+2)) = [ddvdabg1];
            tidstart = tidend + 1; tidend = tidend + 3*3*1;
            tmpv(tidstart:tidend) = (ddvdabg1(:))';
        end
        % v1 & v2
%         J(idxr:(idxr+2),idx_v1:(idx_v1+5)) = [ddvdv1, ddvdv2];
        % g
%         J(idxr:(idxr+2), idx_g:(idx_g+2)) = ddvdg; 
        % dbf & dbw
%         J(idxr:(idxr+2),idx_bf:(idx_bf+5)) = [ddvdbf, ddvdbw];
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
%             J(idxr:(idxr+2),idx_a1:(idx_a1+2)) =  [ddphida1, ddphidb1, ddphidg1];
        end 
        [drda2, drdb2, drdg2] = fabg2r_dr(a2, b2, g2);
        ddphida2 = fndABGdABG(drda2*R1', R12);
        ddphidb2 = fndABGdABG(drdb2*R1', R12);
        ddphidg2 = fndABGdABG(drdg2*R1', R12);
%         J(idxr:(idxr+2),idx_a2:(idx_a2+2)) =  [ddphida2, ddphidb2, ddphidg2];
        % dbw
        ddphidbw = -jddphidbw;	
%         J(idxr:(idxr+2),idx_bw:(idx_bw+2)) = ddphidbw;
        tidstart = tidend + 1; tidend = tidend + 3*3*2;
        tmpv(tidstart:tidend) = [(ddphida2(:))', (ddphidb2(:))', (ddphidg2(:))', (ddphidbw(:))'];

    % 	a1 = a2; b1 = b2; g1 = g2;
        drda1 = drda2; drdb1 = drdb2; drdg1 = drdg2;
        T1 = T2;R1 = R2;
        v1 = v2; idx_v1 = idx_v2;
    end

    %% After IMU observations
    % idxr = 3*nPts*nPoses + (nPoses-1)*9 + 1;
    idxr = nUV + (nPoses-1)*9 + 1;%2*nPts*nPoses
    % %% dvi
    % idxc = (nPoses-1)*6+nPts*3+1;
    % for k=1:(0)%nPoses+3
    %     dvidvi = eye(3);       
    %     J(idxr:(idxr+2), idxc:(idxc+2)) = dvidvi;
    %     idxc = idxc + 3;
    %     idxr = idxr + 3;
    % end

    if(bAddZg == 1)
        %% dg
%         idxc = idx_g;%idx_au2c;%       
%         J(idxr:(idxr+2), idxc:(idxc+2)) = eye(3);
%         idxr = idxr + 3;
        tidstart = tidend + 1; tidend = tidend + 3*3*1;
        tv = eye(3);
        tmpv(tidstart:tidend) = (tv(:))';
    end
end

if(bAddZau2c == 1)
    %% dAu2c % 
%     idxc = idx_au2c;% 
%     J(idxr:(idxr+2), idxc:(idxc+2)) = eye(3);
%     idxr = idxr + 3; 
    tidstart = tidend + 1; tidend = tidend + 3*3*1;
    tv = eye(3);
    tmpv(tidstart:tidend) = (tv(:))';   
end
if(bAddZtu2c == 1)
    %% dTu2c % 
%     idxc = idx_au2c + 3;% 
%     J(idxr:(idxr+2), idxc:(idxc+2)) = eye(3);
%     idxr = idxr + 3; 
    tidstart = tidend + 1; tidend = tidend + 3*3*1;
    tv = eye(3);
    tmpv(tidstart:tidend) = (tv(:))';     
end

if(bUVonly == 1)
    %% dT2 % 
%     idxc = 6;%1;%4 
    tidstart = tidend + 1; tidend = tidend + 1;
    tmpv(tidstart:tidend) = 1;    
%     J(idxr, idxc) = 1;%J(idxr:(idxr+5), idxc:(idxc+5)) = eye(6);%3
%     idxr = idxr + 1;%6;%3;    
elseif(bVarBias == 0)
    if(bAddZbf == 1)
        %% dbf % 
%         idxc = idx_bf;% 
%         J(idxr:(idxr+2), idxc:(idxc+2)) = eye(3);
%         idxr = idxr + 3;    
        tidstart = tidend + 1; tidend = tidend + 3*3*1;
        tv = eye(3);
        tmpv(tidstart:tidend) = (tv(:))';  
    end
    if(bAddZbw == 1)
        %% dbw % 
%         idxc = idx_bw;% 
%         J(idxr:(idxr+2), idxc:(idxc+2)) = eye(3);
%         idxr = idxr + 3;    
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

% idxr = idxr +3;
% J(idxr:(idxr+2), idxc:(idxc+2)) = dvidvi;
% 
% %% dbf, dbw
% dbfdbf = eye(3);
% idxr = idxr + 3;
% J(idxr:(idxr+2), idx_bf:(idx_bf+2)) = dbfdbf;