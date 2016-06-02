function J = fnJdaw0_IMU(J, nPoses, nPts, x, nIMUrate, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw)%g, 
%% Find Jacobian for dp, dv and dphi
%% dp = Ru1 * (Tu2-Tu1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
%% dv = Ru1 * (v2-v1-g*dt) - ddvdbf*dbf - ddvdbw*dbw;
%% [a,b,g] = fnABG5R(Ru2*(Ru1)');
%% dphi = [a;b;g] - ddphidbw*dbw;
% K: camera model
% p3d0: 3D points at the first camera pose
% x: current estimate of the states (alpha, beta, gamma, T)

% J = sparse(3*N*nPoses+3*3*(nPoses-1), 3*N+6*(nPoses-1)+3*nPoses+5*3);
    idx_au2c = 6*nIMUrate*(nPoses-1)+nPts*3+3*((nPoses-1)*nIMUrate+1)+3;%(nPoses-1)*6+nPts*3+3*nPoses+4;
    idx_g = 6*nIMUrate*(nPoses-1)+nPts*3+3*((nPoses-1)*nIMUrate+1);%(nPoses-1)*6+nPts*3+3*nPoses+1;
    g = x((idx_g+1):(idx_g+3));
    idx_bf = 6*nIMUrate*(nPoses-1)+nPts*3+3*((nPoses-1)*nIMUrate+1)+9;%;(nPoses-1)*6+nPts*3+3*nPoses+10;
%     bf = x(idx_bf:(idx_bf+2));
    idx_bw = 6*nIMUrate*(nPoses-1)+nPts*3+3*((nPoses-1)*nIMUrate+1)+12;%(nPoses-1)*6+nPts*3+3*nPoses+13;
%     bw = x(idx_bw:(idx_bw+2));

%     R1 = eye(3); T1 = zeros(3,1);
%     % a1 = 0; b1 = 0; g1 = 0;
%     drda1 = zeros(3); drdb1 = zeros(3); drdg1 = zeros(3);
%     idx_v1 = 6*nIMUrate*(nPoses-1)+nPts*3+1;%+3*((nPoses-1)*nIMUrate+1)%(nPoses-1)*6+nPts*3+1;
%     v1 = x(idx_v1:(idx_v1+2));

    dt = 1.0/nIMUrate;
    
    for pid=1:((nPoses-1)*nIMUrate)
        idr = 2*nPts*nPoses + (pid-1)*9;%nIMUrate*
        idx = ((nPoses-1)*nIMUrate*6+3*nPts+(pid-1)*3); 
        idx_vi = idx;
        vi = x((idx+1):(idx+3));
        idx_vi1 = idx_vi + 3;
        vi1 = x((idx+4):(idx+6));
        if(pid > 1)
            idx = (pid-2)*6;
            idx_abgi = idx;
            alpha = x(idx+1); beta = x(idx+2); gamma = x(idx+3);
            Ti = x((idx+4):(idx+6));
            idx_ti = idx + 3;
            %[drda, drdb, drdg] = fabg2r_dr(alpha, beta, gamma);
        else
            alpha = 0; beta = 0; gamma = 0;
            idx = -6;
            Ti = zeros(3,1);
            %drda = zeros(3); drdb = zeros(3); drdg = zeros(3);
        end
        [drda, drdb, drdg] = fabg2r_dr(alpha, beta, gamma);
        phii = [alpha;beta;gamma];
        Ri = fnR5ABG(alpha,beta,gamma);
        %% 1. wi = Ei*(phii1-phii)/dt+bw;
        Ei = Jac_ko(phii);
        idx = idx+6;
        alpha = x(idx+1); beta = x(idx+2); gamma = x(idx+3);
        idx_abgi1 = idx;
        phii1 = [alpha;beta;gamma];
        % J of wi = Ei*(phii1-phii)/dt+bw;
        dEdal = fndEdalpha(phii);
        dEdbe = fndEdbeta(phii);
        dEdga = fndEdgamma(phii);
        dwidEa = dEdal*(phii1-phii);
        dwidEb = dEdbe*(phii1-phii);
        dwidEg = dEdga*(phii1-phii);
        dwidE = [dwidEa, dwidEb,dwidEg];
        dwidphii1 = Ei/dt;
        dwidphii = (dwidE-Ei)/dt;
        dwidbw = eye(3);
        if(pid > 1)
            J((idr+1):(idr+3), (idx_abgi+1):(idx_abgi+3)) = dwidphii;
        end
        J((idr+1):(idr+3), (idx_abgi1+1):(idx_abgi1+3)) = dwidphii1;
        J((idr+1):(idr+3), (idx_bw+1):(idx_bw+3)) = dwidbw;
        idr = idr + 3;
        
        %% 2. ai = Ri*((vi1-vi)/dt-g)-bf;
        daidvi1 = Ri/dt;
        daidvi = -Ri/dt;
        daidg = -Ri;
        daidbf = eye(3);
        % fill in Jacobian
        if(pid > 1)
            daidal = drda*((vi1-vi)/dt-g);
            daidbe = drdb*((vi1-vi)/dt-g);
            daidga = drdg*((vi1-vi)/dt-g);
            daidabg = [daidal,daidbe,daidga];
            J((idr+1):(idr+3), (idx_abgi+1):(idx_abgi+3)) = daidabg;
        end
        J((idr+1):(idr+3), (idx_vi+1):(idx_vi+3)) = daidvi;
        J((idr+1):(idr+3), (idx_vi1+1):(idx_vi1+3)) = daidvi1;
        J((idr+1):(idr+3), (idx_g+1):(idx_g+3)) = daidg;
        J((idr+1):(idr+3), (idx_bf+1):(idx_bf+3)) = daidbf;
        idr = idr + 3;
        
        %% 3. bzero = Ti1-Ti-vi*dt;
        idx = idx + 3;
        Ti1 = x((idx+1):(idx+3));
        idx_ti1 = idx;
        % J of bzero = Ti1-Ti-vi*dt;
        dbzdti1 = eye(3);
        dbzdti = -eye(3);
        dbzdvi = -eye(3)*dt;
        J((idr+1):(idr+3), (idx_ti1+1):(idx_ti1+3)) = dbzdti1;
        if(pid > 1)
            J((idr+1):(idr+3), (idx_ti+1):(idx_ti+3)) = dbzdti;
        end
        J((idr+1):(idr+3), (idx_vi+1):(idx_vi+3)) = dbzdvi;
    end
% end
%%%%%%%%%%%%%%%%
% for pid = 2:nPoses
%     idx_a2 = (pid-2)*6+1;
%     a2 = x(idx_a2); b2 = x(idx_a2+1); g2 = x(idx_a2+2);
%     R2 = fnR5ABG(a2,b2,g2);
%     T2 = x((idx_a2+3):(idx_a2+5),1);
%     idx_v2 = (nPoses-1)*6+nPts*3+(pid-1)*3+1;
%     v2 = x(idx_v2:(idx_v2+2));
%     jddpdbf = Jd{pid}(1:3, 10:12); jddpdbw = Jd{pid}(1:3, 13:15);
%     jddvdbf = Jd{pid}(4:6, 10:12); jddvdbw = Jd{pid}(4:6, 13:15);
%     jddphidbw = Jd{pid}(7:9, 13:15);  
% %     idxr = 3*nPts*nPoses + (pid-2)*9 + 1;
% idxr = 2*nPts*nPoses + (pid-2)*9 + 1;    
%     % 1. dp = R1 * (T2-T1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
%     ddpdt2 = R1;
%     ddpdv1 = -R1 * dt;
%     ddpdg = -0.5 * R1 * dt * dt;
%     ddpdbf = -jddpdbf;
%     ddpdbw = -jddpdbw;
%     %% fill in Jacobian
%     % dT2
%     idx_t2 = (pid - 2)*6 + 4;
%     J(idxr:(idxr+2), idx_t2:(idx_t2+2)) = ddpdt2;
%     % v1
%     idx_v11 = (nPoses-1)*6+nPts*3+3*(pid-2) + 1;    
%     J(idxr:(idxr+2),idx_v11:(idx_v11+2)) = ddpdv1;
%     % g
% %     idx_g = (nPoses-1)*6+nPts*3+3*nPoses+1;
%     J(idxr:(idxr+2), idx_g:(idx_g+2)) = ddpdg;    
%     % bf & bw
% %     idx_bf = (nPoses-1)*6+nPts*3+3*nPoses+10;
%     J(idxr:(idxr+2), idx_bf:(idx_bf+5)) = [ddpdbf,ddpdbw];      
%     if(pid > 2)
% %         [drda1, drdb1, drdg1] = fabg2r_dr(a1, b1, g1);
%         ddpda = drda1 * (T2-T1-v1*dt-0.5*g*dt*dt);%3x1
%         ddpdb = drdb1 * (T2-T1-v1*dt-0.5*g*dt*dt);
%         ddpdg = drdg1 * (T2-T1-v1*dt-0.5*g*dt*dt);
%         ddpdabg1 = [ddpda,ddpdb,ddpdg];% 3x3
%         ddpdt1 = -R1;        
%         % dabgT1
%         idx_a1 = (pid-3)*6 + 1;
%         J(idxr:(idxr+2), idx_a1:(idx_a1+5)) = [ddpdabg1, ddpdt1];        
%     end        
%     % 2. dv = R1 * (v2-v1-g*dt) - jddvdbf*(bf-bf0) - jddvdbw*(bw-bw0);    
%     ddvdv1 = -R1;
%     ddvdv2 = R1;
%     ddvdg = -R1 * dt;
%     ddvdbf = -jddvdbf;
%     ddvdbw = -jddvdbw;
%     %% Fill in ddv
%     idxr = idxr + 3; 
%     if(pid > 2)
%         % dabg1
% 		ddvdabg1 = [drda1 * (v2-v1-g*dt), drdb1 * (v2-v1-g*dt), drdg1 * (v2-v1-g*dt)];
% %         idxc = (pid-3)*6 + 1;
%         J(idxr:(idxr+2), idx_a1:(idx_a1+2)) = [ddvdabg1];
%     end
%     % v1 & v2
% %     idxc = (nPoses-1)*6+nPts*3+3*(pid-2) + 1;    
%     J(idxr:(idxr+2),idx_v1:(idx_v1+5)) = [ddvdv1, ddvdv2];
%     % g
% %     idxc = (nPoses-1)*6+nPts*3+3*nPoses+1;
%     J(idxr:(idxr+2), idx_g:(idx_g+2)) = ddvdg; 
%     % dbf & dbw
% %     idxc = (nPoses-1)*6+nPts*3+3*nPoses+10;
%     J(idxr:(idxr+2),idx_bf:(idx_bf+5)) = [ddvdbf, ddvdbw];
%     %% 3. [a,b,g] = fnABG5R(Ru2*(Ru1)'); dphi = [a;b;g] - ddphidbw*dbw;
%     idxr = idxr + 3;    
%     R12 = R2*(R1)';
% 	if(pid > 2)		
% 		ddphida1 = fndABGdABG(R2*(drda1'), R12);
% 		ddphidb1 = fndABGdABG(R2*(drdb1'), R12);
% 		ddphidg1 = fndABGdABG(R2*(drdg1'), R12);
% %         idxc = (pid-3)*6 + 1;
%         J(idxr:(idxr+2),idx_a1:(idx_a1+2)) =  [ddphida1, ddphidb1, ddphidg1];
%     end 
%     [drda2, drdb2, drdg2] = fabg2r_dr(a2, b2, g2);
%     ddphida2 = fndABGdABG(drda2*R1', R12);
%     ddphidb2 = fndABGdABG(drdb2*R1', R12);
%     ddphidg2 = fndABGdABG(drdg2*R1', R12);
% %     idxc = (pid-2)*6 + 1;
%     J(idxr:(idxr+2),idx_a2:(idx_a2+2)) =  [ddphida2, ddphidb2, ddphidg2]; 	
% 	% dbw
% 	ddphidbw = -jddphidbw;	
%     J(idxr:(idxr+2),idx_bw:(idx_bw+2)) = ddphidbw;
% 
% % 	a1 = a2; b1 = b2; g1 = g2;
%     drda1 = drda2; drdb1 = drdb2; drdg1 = drdg2;
%     T1 = T2;R1 = R2;
%     v1 = v2; idx_v1 = idx_v2;
% end
% 
% %% After IMU observations
% % idxr = 3*nPts*nPoses + (nPoses-1)*9 + 1;
% idxr = 2*nPts*nPoses + (nPoses-1)*9 + 1;
% % %% dvi
% % idxc = (nPoses-1)*6+nPts*3+1;
% % for k=1:(0)%nPoses+3
% %     dvidvi = eye(3);       
% %     J(idxr:(idxr+2), idxc:(idxc+2)) = dvidvi;
% %     idxc = idxc + 3;
% %     idxr = idxr + 3;
% % end
% 
idxr = 2*nPts*nPoses + (nPoses-1)*nIMUrate*9;%idr + 4;
%% dg, dAu2c, dTu2c % 
if(bAddZg == 1)
    %% dg
    idxc = idx_g;%idx_au2c;%       
    J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    idxr = idxr + 3;
end
if(bAddZau2c == 1)
    %% dAu2c % 
    idxc = idx_au2c;% 
    J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    idxr = idxr + 3; 
end
if(bAddZtu2c == 1)
    %% dTu2c % 
    idxc = idx_au2c + 3;% 
    J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    idxr = idxr + 3; 
end
if(bAddZbf == 1)
    %% dbf % 
    idxc = idx_bf;% 
    J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    idxr = idxr + 3;    
end
if(bAddZbw == 1)
    %% dbw % 
    idxc = idx_bw;% 
    J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    idxr = idxr + 3;    
end

% 
% % idxr = idxr +3;
% % J(idxr:(idxr+2), idxc:(idxc+2)) = dvidvi;
% % 
% % %% dbf, dbw
% % dbfdbf = eye(3);
% % idxr = idxr + 3;
% % J(idxr:(idxr+2), idx_bf:(idx_bf+2)) = dbfdbf;