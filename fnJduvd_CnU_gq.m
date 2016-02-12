function [J] = fnJduvd_CnU_gq(nJacs, idRow, idCol, K, x, nPoses, nPts, nIMUdata, ...
                                ImuTimestamps, RptFeatureObs, nUV )

    global InertialDelta_options
    
    %% Objective function elements: ei = (ui' - ui)^2, ei'= (vi' - vi)^2 (i=1...N)
    % Find R, T corresponding to 3D points pi and pi'.
    % 
    % K: camera model
    % p3d0: 3D points at the first camera pose
    % x: current estimate of the states (alpha, beta, gamma, T)

    tmpv = zeros(1, nJacs);
    tidend = 0;

    nObsId_FeatureObs = 2;
    fx = K(1,1); cx0 = K(1,3); fy = K(2,2); cy0 = K(2,3);

    % Section for pose 1
    if((InertialDelta_options.bUVonly == 1) || (InertialDelta_options.bPreInt == 1))
        idx = (nPoses-1)*6;    
    else
        idx = 6*nIMUdata;%nIMUrate*(nPoses-1);
    end
    p3d0 = reshape(x((idx+1):(idx+3*nPts), 1), 3, []);    
    
    % x0 = (p3d0(1,:))';
    % y0 = (p3d0(2,:))';
    % z0 = (p3d0(3,:))';
    % du = [f./z0, zeros(nPts,1),-f*x0./(z0.*z0)]; % Nx3
    % dv = [zeros(nPts,1), f./z0, -f*y0./(z0.*z0)];
    % dd = repmat([0, 0, 1], nPts, 1);
    % duvd = [du'; dv'; dd'];% 9xN
    % duvd = (reshape(duvd, 3, []))'; % 3Nx3
    % for i = 1:nPts
    %     J((3*(i-1)+1):(3*i), (6*(nPoses-1)+3*(i-1)+1):(6*(nPoses-1)+3*i)) = duvd((3*(i-1)+1):(3*i),:);
    % end
    
    if(InertialDelta_options.bUVonly == 1)
        idx = (nPoses-1)*6+nPts*3+1;
    else
        if(InertialDelta_options.bPreInt == 1)
            idx = (nPoses-1)*6+nPts*3+3*nPoses+4;    
        else
            idx = 6*nIMUdata+nPts*3+3*(nIMUdata+1)+4;
        end
    end
    
    a_u2c = x(idx); b_u2c = x(1+idx); g_u2c = x(2+idx);
    Ru2c = fnR5ABG(a_u2c, b_u2c, g_u2c);%fRx(alpha) * fRy (beta) * fRz(gamma);         
    Tu2c = x((3+idx):(5+idx), 1);

    %%%%%%%%%
    a_cat = zeros(nPoses, 1);
    b_cat = a_cat;
    g_cat = a_cat;
    Ru_cat = zeros(3,3,nPoses);
    Tu_cat = zeros(3,nPoses);
    Rc_cat = zeros(3,3,nPoses);
    Tc_cat = zeros(3,nPoses);
    
    for(pid = 1:nPoses)        
        if(pid > 1)
            
            if((InertialDelta_options.bUVonly == 1) || (InertialDelta_options.bPreInt == 1))
                idx = (pid-2)*6;    
            else
                idx = (ImuTimestamps(pid)-ImuTimestamps(1)-1)*6;%6*nIMUrate*(pid-1)-6;
            end
            
            a_cat(pid) = x(1+idx); b_cat(pid) = x(2+idx); g_cat(pid) = x(3+idx); 
            Ru_cat(:,:,pid) = fnR5ABG(a_cat(pid), b_cat(pid), g_cat(pid));%fRx(alpha) * fRy (beta) * fRz(gamma);
            Tu_cat(:,pid) = x((4+idx):(idx+6), 1);            
            
        else % pid ==1, Ru2c,Tu2c
            
            a = 0; b = 0; g = 0;
            Ru_cat(:,:,pid) = eye(3); 
            %Tu_cat(:,pid) = zeros(3,1);
            
        end
        
        Rc_cat(:,:,pid) = Ru2c * Ru_cat(:,:,pid);
        Tc_cat(:,pid) = (Ru_cat(:,:,pid))' * Tu2c + Tu_cat(:,pid);        
    end
    
    %%%%%%%%%
    %nUV = 0;
    % Section for all of the poses
    for(fid=1:nPts)
        
        nObs = RptFeatureObs(fid, nObsId_FeatureObs);
        
        for(oid=1:nObs)
            
            pid = RptFeatureObs(fid, oid*3); 
            if(pid > nPoses)
                break;
            end
            
            a = a_cat(pid); b = b_cat(pid); g = g_cat(pid);
            Ru = Ru_cat(:,:,pid); Tu = Tu_cat(:,pid);
            Rc = Rc_cat(:,:,pid); Tc = Tc_cat(:,pid);

            ncurrentObsdPts = 1;%size(obsfeatures{pid}, 1);
            p3d1 = Rc * (p3d0(:, fid) - repmat(Tc, 1, ncurrentObsdPts));    
            
            [duvd] = fnuvd5xyz_dr_general(p3d1, fx, fy, ncurrentObsdPts);%[duvd] = fxyz2uvd_dr(p3d1(1,i), p3d1(2,i), p3d1(3,i), f);
            
            [dxyz,dxyz_u2c, ~] = fnxyz5abgxyz_drCIU(a, b, g, a_u2c, b_u2c, g_u2c, Ru, Ru2c, ...
                            Tu, Tu2c, p3d0(:, fid), ncurrentObsdPts);

                               %% d(uv)/d(xyz)f
            % d(xyz)/d(xyz)f
            dfxyz = Rc;
            
            %% d(uv)/d(xyz)f
            duvdfxyz = duvd(1:2,:) * dfxyz;        
            
            %% d(uv)/d(abgxyz)u2c part
            duvdxyz_u2c = duvd(1:2,:) * dxyz_u2c;                    

            %% d(uv)/d(abgxyz)c, 2x6
            if(pid > 1)
                
                duvddabgxyz = duvd(1:2,:) * dxyz;
                tidstart = tidend + 1;
                tidend = tidend + 12+6+12;%12;
                tmpv(tidstart:tidend) = [(duvddabgxyz(:))',(duvdfxyz(:))',...
                    (duvdxyz_u2c(:))' ];%(duvddabgxyz(:))';
                
            else
                
                tidstart = tidend + 1;
                tidend = tidend + 6 + 12;
                tmpv(tidstart:tidend) = [(duvdfxyz(:))',(duvdxyz_u2c(:))'];
                
            end             
        end
    end

    if(InertialDelta_options.bUVonly == 1)
        
        nJ8row = 3*2+1;
        nJcol = 3*nPts+6*(nPoses-1)+3*2;  
        %% Batch op
        J = [sparse(idRow, idCol, tmpv); sparse(nJ8row,nJcol)];    
        
    else
        if(InertialDelta_options.bPreInt == 1)
            nJ8row = 3*3*(nPoses-1); 
            if(InertialDelta_options.bVarBias == 0)
                nJ8col = 2*3;
                nJcol = 3*nPts+6*(nPoses-1)+3*nPoses+5*3;
            else
                nJ8col = 2*3*(nPoses-1);
                nJcol = 3*nPts+6*(nPoses-1)+3*nPoses+3*3+6*(nPoses-1);
            end
            
        else%nIMUrate*(nPoses-1) nIMUrate*(nPoses-1)
            nJ8row = 9*nIMUdata;
            if(InertialDelta_options.bVarBias == 0)
                nJ8col = 2*3;
                nJcol = 3*nPts+6*(nIMUdata)+3*(nIMUdata+1)+5*3;
            else
                nJ8col = 2*3*(nPoses-1);
                nJcol = 3*nPts+6*(nIMUdata)+3*(nIMUdata+1)+3*3+6*(nPoses-1);            
            end
        end
        
        if(InertialDelta_options.bAddZg == 1)
            nJ8row = nJ8row + 3;
        end
        
        if(InertialDelta_options.bAddZau2c == 1)
            nJ8row = nJ8row + 3;
        end 
        
        if(InertialDelta_options.bAddZtu2c == 1)
            nJ8row = nJ8row + 3;
        end 
        
        if(InertialDelta_options.bVarBias == 0)
            if(InertialDelta_options.bAddZbf == 1)
                nJ8row = nJ8row + 3;
            end        
            
            if(InertialDelta_options.bAddZbw == 1)
                nJ8row = nJ8row + 3;
            end 
            
        else
            
            nJ8row = nJ8row + 6*(nPoses - 2);
            
        end
        
        %% Batch op    
        J = [sparse(idRow, idCol, tmpv), sparse(nUV, nJ8col);sparse(nJ8row,nJcol)];    
        
    end % else UVOnly==0
