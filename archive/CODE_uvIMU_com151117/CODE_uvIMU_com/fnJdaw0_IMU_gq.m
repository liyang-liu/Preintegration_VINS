function J = fnJdaw0_IMU_gq(idRow, idCol, nJacs, nUV, nPts, x, nIMUrate, nIMUdata, ...
    bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias, nPoses)%g, 
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

    idx_g = 6*(nIMUdata)+nPts*3+3*(nIMUdata+1);
    idx_au2c = idx_g + 3;%nIMUrate*(nPoses-1) ((nPoses-1)*nIMUrate+1)(nPoses-1)*6+nPts*3+3*nPoses+4;
    g = x((idx_g+1):(idx_g+3));
    idx_bf = idx_au2c + 6;%;(nPoses-1)*6+nPts*3+3*nPoses+10;
    idx_bw = idx_bf + 3;%(nPoses-1)*6+nPts*3+3*nPoses+13;

    dt = 1.0/nIMUrate;
    
    for pid=1:(nIMUdata)%(nPoses-1)*nIMUrate
        idr = (pid-1)*9;%nIMUrate* nUV + 
        idx = (nIMUdata)*6+3*nPts+(pid-1)*3; 
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
%             J((idr+1):(idr+3), (idx_abgi+1):(idx_abgi+3)) = dwidphii;
            tidstart = tidend + 1; tidend = tidend + 3*3;
            tmpv(tidstart:tidend) = dwidphii(:);
        end
%         J((idr+1):(idr+3), (idx_abgi1+1):(idx_abgi1+3)) = dwidphii1;
%         J((idr+1):(idr+3), (idx_bw+1):(idx_bw+3)) = dwidbw;
%         idr = idr + 3;
        tidstart = tidend + 1; tidend = tidend + 3*3*2;
        tmpv(tidstart:tidend) = [dwidphii1(:);dwidbw(:)];
        
        
        
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
%             J((idr+1):(idr+3), (idx_abgi+1):(idx_abgi+3)) = daidabg;
            tidstart = tidend + 1; tidend = tidend + 3*3;
            tmpv(tidstart:tidend) = daidabg(:);
        end
%         J((idr+1):(idr+3), (idx_vi+1):(idx_vi+3)) = daidvi;
%         J((idr+1):(idr+3), (idx_vi1+1):(idx_vi1+3)) = daidvi1;
%         J((idr+1):(idr+3), (idx_g+1):(idx_g+3)) = daidg;
%         J((idr+1):(idr+3), (idx_bf+1):(idx_bf+3)) = daidbf;
%         idr = idr + 3;
        tidstart = tidend + 1; tidend = tidend + 3*3*4;
        tmpv(tidstart:tidend) = [daidvi(:);daidvi1(:);daidg(:);daidbf(:)];
        
        %% 3. bzero = Ti1-Ti-vi*dt;
        idx = idx + 3;
        Ti1 = x((idx+1):(idx+3));
        idx_ti1 = idx;
        % J of bzero = Ti1-Ti-vi*dt;
        dbzdti1 = eye(3);
        dbzdti = -eye(3);
        dbzdvi = -eye(3)*dt;
%         J((idr+1):(idr+3), (idx_ti1+1):(idx_ti1+3)) = dbzdti1;
        if(pid > 1)
%             J((idr+1):(idr+3), (idx_ti+1):(idx_ti+3)) = dbzdti;
            tidstart = tidend + 1; tidend = tidend + 3*3;
            tmpv(tidstart:tidend) = dbzdti(:);
        end
%         J((idr+1):(idr+3), (idx_vi+1):(idx_vi+3)) = dbzdvi;
        tidstart = tidend + 1; tidend = tidend + 3*3*2;
        tmpv(tidstart:tidend) = [dbzdti1(:);dbzdvi(:)];        
    end

idxr = nUV + (nIMUdata)*9;%idr + 4;
%% dg, dAu2c, dTu2c % 
if(bAddZg == 1)
    %% dg
%     idxc = idx_g;%idx_au2c;%       
%     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
%     idxr = idxr + 3;
    tidstart = tidend + 1; tidend = tidend + 3*3;
    tv = eye(3);
    tmpv(tidstart:tidend) = tv(:);    
end
if(bAddZau2c == 1)
    %% dAu2c % 
%     idxc = idx_au2c;% 
%     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
%     idxr = idxr + 3; 
    tidstart = tidend + 1; tidend = tidend + 3*3;
    tv = eye(3);
    tmpv(tidstart:tidend) = tv(:);     
end
if(bAddZtu2c == 1)
    %% dTu2c % 
%     idxc = idx_au2c + 3;% 
%     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
%     idxr = idxr + 3; 
    tidstart = tidend + 1; tidend = tidend + 3*3;
    tv = eye(3);
    tmpv(tidstart:tidend) = tv(:);     
end

if(bVarBias == 0)
    if(bAddZbf == 1)
        %% dbf % 
    %     idxc = idx_bf;% 
    %     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    %     idxr = idxr + 3; 
        tidstart = tidend + 1; tidend = tidend + 3*3;
        tv = eye(3);
        tmpv(tidstart:tidend) = tv(:);     
    end
    if(bAddZbw == 1)
        %% dbw % 
    %     idxc = idx_bw;% 
    %     J((idxr+1):(idxr+3), (idxc+1):(idxc+3)) = eye(3);
    %     idxr = idxr + 3;    
        tidstart = tidend + 1; tidend = tidend + 3*3;
        tv = eye(3);
        tmpv(tidstart:tidend) = tv(:); 
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
% 
% % idxr = idxr +3;
% % J(idxr:(idxr+2), idxc:(idxc+2)) = dvidvi;
% % 
% % %% dbf, dbw
% % dbfdbf = eye(3);
% % idxr = idxr + 3;
% % J(idxr:(idxr+2), idx_bf:(idx_bf+2)) = dbfdbf;