function J = fnJddpvphi_IMU(J, dt, g, Jd, nPoses, nPts, x)
%% Find Jacobian for dp, dv and dphi
%% dp = Ru1 * (Tu2-Tu1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
%% dv = Ru1 * (v2-v1-g*dt) - ddvdbf*dbf - ddvdbw*dbw;
%% [a,b,g] = fnABG5R(Ru2*(Ru1)');
%% dphi = [a;b;g] - ddphidbw*dbw;
% K: camera model
% p3d0: 3D points at the first camera pose
% x: current estimate of the states (alpha, beta, gamma, T)

% J = sparse(3*N*nPoses+3*3*(nPoses-1), 3*N+6*(nPoses-1)+3*nPoses+5*3);
idx_g = (nPoses-1)*6+nPts*3+3*nPoses+1;
idx_bf = (nPoses-1)*6+nPts*3+3*nPoses+10;
idx_bw = (nPoses-1)*6+nPts*3+3*nPoses+13;

R1 = eye(3); T1 = zeros(3,1);
a1 = 0; b1 = 0; g1 = 0;
drda1 = zeros(3); drdb1 = zeros(3); drdg1 = zeros(3);
idx_v1 = (nPoses-1)*6+nPts*3+1;
v1 = x(idx_v1:(idx_v1+2));
for pid = 2:nPoses
    idx_a2 = (pid-2)*6+1;
    a2 = x(idx_a2); b2 = x(idx_a2+1); g2 = x(idx_a2+2);
    R2 = fnR5ABG(a2,b2,g2);
    T2 = x((idx_a2+3):(idx_a2+5),1);
    idx_v2 = (nPoses-1)*6+nPts*3+(pid-1)*3+1;
    v2 = x(idx_v2:(idx_v2+2));
    jddpdbf = Jd{pid}(1:3, 10:12); jddpdbw = Jd{pid}(1:3, 13:15);
    jddvdbf = Jd{pid}(4:6, 10:12); jddvdbw = Jd{pid}(4:6, 13:15);
    jddphidbw = Jd{pid}(7:9, 13:15);  
    idxr = 3*nPts*nPoses + (pid-2)*9 + 1;
    % 1. dp = R1 * (T2-T1-v1*dt-0.5*g*dt*dt) - ddpdbf*dbf - ddpdbw*dbw;
    ddpdt2 = R1;
    ddpdv1 = -R1 * dt;
    ddpdg = -0.5 * R1 * dt * dt;
    ddpdbf = -jddpdbf;
    ddpdbw = -jddpdbw;
    %% fill in Jacobian
    % dT2
    idx_t2 = (pid - 2)*6 + 4;
    J(idxr:(idxr+2), idx_t2:(idx_t2+2)) = ddpdt2;
    % v1
    idx_v1 = (nPoses-1)*6+nPts*3+3*(pid-2) + 1;    
    J(idxr:(idxr+2),idx_v1:(idx_v1+2)) = ddpdv1;
    % g
%     idx_g = (nPoses-1)*6+nPts*3+3*nPoses+1;
    J(idxr:(idxr+2), idx_g:(idx_g+2)) = ddpdg;    
    % bf & bw
%     idx_bf = (nPoses-1)*6+nPts*3+3*nPoses+10;
    J(idxr:(idxr+2), idx_bf:(idx_bf+5)) = [ddpdbf,ddpdbw];      
    if(pid > 2)
%         [drda1, drdb1, drdg1] = fabg2r_dr(a1, b1, g1);
        ddpda = drda1 * (T2-T1-v1*dt-0.5*g*dt*dt);%3x1
        ddpdb = drdb1 * (T2-T1-v1*dt-0.5*g*dt*dt);
        ddpdg = drdg1 * (T2-T1-v1*dt-0.5*g*dt*dt);
        ddpdabg1 = [ddpda,ddpdb,ddpdg];% 3x3
        ddpdt1 = -R1;        
        % dabgT1
        idx_a1 = (pid-3)*6 + 1;
        J(idxr:(idxr+2), idx_a1:(idx_a1+5)) = [ddpdabg1, ddpdt1];        
    end        
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
        J(idxr:(idxr+2), idx_a1:(idx_a1+2)) = [ddvdabg1];
    end
    % v1 & v2
%     idxc = (nPoses-1)*6+nPts*3+3*(pid-2) + 1;    
    J(idxr:(idxr+2),idx_v1:(idx_v1+5)) = [ddvdv1, ddvdv2];
    % g
%     idxc = (nPoses-1)*6+nPts*3+3*nPoses+1;
    J(idxr:(idxr+2), idx_g:(idx_g+2)) = ddvdg; 
    % dbf & dbw
%     idxc = (nPoses-1)*6+nPts*3+3*nPoses+10;
    J(idxr:(idxr+2),idx_bf:(idx_bf+5)) = [ddvdbf, ddvdbw];
    %% 3. [a,b,g] = fnABG5R(Ru2*(Ru1)'); dphi = [a;b;g] - ddphidbw*dbw;
    idxr = idxr + 3;    
    R12 = R2*(R1)';
	if(pid > 2)		
		ddphida1 = fndABGdABG(R2*(drda1'), R12);
		ddphidb1 = fndABGdABG(R2*(drdb1'), R12);
		ddphidg1 = fndABGdABG(R2*(drdg1'), R12);
%         idxc = (pid-3)*6 + 1;
        J(idxr:(idxr+2),idx_a1:(idx_a1+2)) =  [ddphida1, ddphidb1, ddphidg1];
    end 
    [drda2, drdb2, drdg2] = fabg2r_dr(a2, b2, g2);
    ddphida2 = fndABGdABG(drda2*R1', R12);
    ddphidb2 = fndABGdABG(drdb2*R1', R12);
    ddphidg2 = fndABGdABG(drdg2*R1', R12);
%     idxc = (pid-2)*6 + 1;
    J(idxr:(idxr+2),idx_a2:(idx_a2+2)) =  [ddphida2, ddphidb2, ddphidg2]; 	
	% dbw
	ddphidbw = -jddphidbw;	
    J(idxr:(idxr+2),idx_bw:(idx_bw+2)) = ddphidbw;

	a1 = a2; b1 = b2; g1 = g2;
    drda1 = drda2; drdb1 = drdb2; drdg1 = drdg2;
    T1 = T2;R1 = R2;
    v1 = v2;
end

%% dbf, dbw
dbfdbf = eye(3);
idxr = idxr + 3;
J(idxr:(idxr+2), idx_bf:(idx_bf+2)) = dbfdbf;
dbwdbw = eye(3);
idxr = idxr + 3;
J(idxr:(idxr+2), idx_bw:(idx_bw+2)) = dbwdbw;


% % Section for pose 1
% p3d0 = reshape(x(((nPoses-1)*6+1):((nPoses-1)*6+3*N), 1), 3, []);
% % x0 = (p3d0(1,:))';
% % y0 = (p3d0(2,:))';
% % z0 = (p3d0(3,:))';
% % du = [f./z0, zeros(N,1),-f*x0./(z0.*z0)]; % Nx3
% % dv = [zeros(N,1), f./z0, -f*y0./(z0.*z0)];
% % dd = repmat([0, 0, 1], N, 1);
% % duvd = [du'; dv'; dd'];% 9xN
% % duvd = (reshape(duvd, 3, []))'; % 3Nx3
% % for i = 1:N
% %     J((3*(i-1)+1):(3*i), (6*(nPoses-1)+3*(i-1)+1):(6*(nPoses-1)+3*i)) = duvd((3*(i-1)+1):(3*i),:);
% % end
% 
% idxr = (nPoses-1)*6+nPts*3+3*nPoses+4;
% a_u2c = x(idxr); b_u2c = x(1+idxr); g_u2c = x(2+idxr);
% Ru2c = fnR5ABG(a_u2c, b_u2c, g_u2c);%fRx(alpha) * fRy (beta) * fRz(gamma);         
% Tu2c = x((3+idxr):(5+idxr), 1);
% % Section for the rest poses
% for pid=1:nPoses
%     if(pid > 1)
%         a = x(1+(pid-2)*6); b = x(2+(pid-2)*6); g = x(3+(pid-2)*6); 
%         Ru = fnR5ABG(a, b, g);%fRx(alpha) * fRy (beta) * fRz(gamma);
%         Tu = x((4+(pid-2)*6):((pid-1)*6), 1);            
%     else % pid ==1, Ru2c,Tu2c
%         Ru = eye(3); 
%         Tu = zeros(3,1);
%     end
% %     R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
%     Rc = Ru2c * Ru;
%     Tc = Ru' * Tu2c + Tu;
%     p3d1 = Rc * (p3d0 - repmat(Tc, 1, N));    
%     % Find the gradient of u(x,y,z) and v(x,y,z) at the second pose
% 	[duvd] = fnuvd5xyz_dr(p3d1, f, N);%[duvd] = fxyz2uvd_dr(p3d1(1,i), p3d1(2,i), p3d1(3,i), f);
% 	% Find the gradient of xyz(alpha, beta, gamma, T) at the second pose
% % 	[dxyz] = fxyz5abgxyz_dr(alpha, beta, gamma, p3d0, N);
%     [dxyz,dxyz_u2c] = fnxyz5abgxyz_drCIU(a, b, g, a_u2c, b_u2c, g_u2c, Ru, Ru2c, ...
%                     Tu, Tu2c, X0, N);
%     duvddabgxyz = zeros(3*N, 3*N+6*(nPoses-1));
%     for i = 1:N
%         dabgxyz = zeros(3, (nPoses-1)*6+N*3+3*nPoses+3+6+6);
%         % Ru part
%         dabgxyz(:,(6*(pid-2)+1):(6*(pid-1))) = dxyz(:,(6*(i-1)+1):(6*i));%dabgT
%         dabgxyz(:, (6*(nPoses-1)+3*(i-1)+1):(6*(nPoses-1)+3*i)) = Rc;%dx0
%         % Ru2c part
%         idxr = (nPoses-1)*6+nPts*3+3*nPoses+4;
%         dabgxyz(:,(idxr):(idxr+5)) = dxyz_u2c(:,(6*(i-1)+1):(6*i));%dabgT_u
%         % Complete the chain
%         duvddabgxyz((3*(i-1)+1):(3*i), :) = duvd(((i-1)*3+1):(3*i),:) * dabgxyz;        
%     end
%     J((3*N*(pid-1)+1):(3*N*pid),:) = duvddabgxyz;
% end

