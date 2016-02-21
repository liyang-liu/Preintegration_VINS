function J = fnJacobian_dUv_dX(J, K, X, Zobs, nPoses, nPts, nIMUdata, ImuTimestamps, RptFeatureObs )
    %[J] = fnJacobian_dUv_dX( nJacs, idRow, idCol, K, x, nPoses, nPts, nIMUdata, ...
             %                   ImuTimestamps, RptFeatureObs, nUV )

    global InertialDelta_options
    
    %% Objective function elements: ei = (ui' - ui)^2, ei'= (vi' - vi)^2 (i=1...N)
    % Find R, T corresponding to 3D points pi and pi'.
    % 
    % K: camera model
    % p3d0: 3D points at the first camera pose
    % x: current estimate of the states (alpha, beta, gamma, T)

    %tmpv = zeros(1, nJacs);
    tidend = 0;
    zid = 0;
    
    
    %nObsId_FeatureObs = 2;
    fx = K(1,1); cx0 = K(1,3); fy = K(2,2); cy0 = K(2,3);

    % Section for pose 1
    if((InertialDelta_options.bUVonly == 1) || (InertialDelta_options.bPreInt == 1))
        idx = (nPoses-1)*6;    
    else
        idx = 6*nIMUdata;%nIMUrate*(nPoses-1);
    end
    %p3d0 = reshape(x((idx+1):(idx+3*nPts), 1), 3, []);    
    %p3d0 = X.feature;
    
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
        idx = (nPoses-1)*6 + nPts*3 + 1;
    else
        if(InertialDelta_options.bPreInt == 1)
            idx = (nPoses-1)*6 + nPts*3 + nPoses*3 + 4;    
        else
            idx = 6*nIMUdata + nPts*3 + (nIMUdata+1)*3 + 4;
        end
    end
    
    %a_u2c = x(idx); b_u2c = x(1+idx); g_u2c = x(2+idx);
    a_u2c = X.Au2c.val(1); b_u2c = X.Au2c.val(2); g_u2c = X.Au2c.val(3);    
    Ru2c = fnRFromABG(a_u2c, b_u2c, g_u2c); %fRx(alpha) * fRy (beta) * fRz(gamma);         
    %Tu2c = x((3+idx):(5+idx), 1);
    Tu2c = X.Tu2c.val;

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
            
            %a_cat(pid) = x(1+idx); b_cat(pid) = x(2+idx); g_cat(pid) = x(3+idx); 
            Au_cat = X.pose(pid-1).ang.val;
            a_cat(pid) = Au_cat(1); b_cat(pid) = Au_cat(2);  g_cat(pid) = Au_cat(3);
            Ru_cat(:,:,pid) = fnRFromABG(a_cat(pid), b_cat(pid), g_cat(pid));%fRx(alpha) * fRy (beta) * fRz(gamma);
            
            %Tu_cat(:,pid) = x((4+idx):(idx+6), 1);            
            Tu_cat(:,pid) = X.pose(pid-1).trans.val;
            
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
        
        nObs = RptFeatureObs(fid).nObs;
        
        for(oid=1:nObs)
            
            pid = RptFeatureObs(fid).obsv(oid).pid; 
            if(pid > nPoses)
                break;
            end
            
            a = a_cat(pid); b = b_cat(pid); g = g_cat(pid);
            Ru = Ru_cat(:,:,pid); Tu = Tu_cat(:,pid);
            Rc = Rc_cat(:,:,pid); Tc = Tc_cat(:,pid);

            %ncurrentObsdPts = 1;%size(obsfeatures{pid}, 1);
            %p3d1 = Rc * (p3d0(:, fid) - repmat(Tc, 1, ncurrentObsdPts));    
            p3d1 = Rc * (X.feature(fid).xyz - Tc);    
            
            %[duvd] = fnuvd5xyz_dr_general(p3d1, fx, fy, ncurrentObsdPts);%[duvd] = fxyz2uvd_dr(p3d1(1,i), p3d1(2,i), p3d1(3,i), f);
            [duvd] = fn_dUv_dCamFxyz_dr(p3d1, fx, fy, 1);
            %[dxyz,dxyz_u2c, ~] = fnxyz5abgxyz_drCIU(a, b, g, a_u2c, b_u2c, g_u2c, Ru, Ru2c, ...
            %                Tu, Tu2c, p3d0(:, fid), ncurrentObsdPts);
            [dxyz,dxyz_u2c, ~] = fn_dCamFxyz_dWldFxyz_dCamU2c(a, b, g, a_u2c, b_u2c, g_u2c, Ru, Ru2c, ...
                                                    Tu, Tu2c, X.feature(fid).xyz, 1); %%fnxyz5abgxyz_drCIU
                               %% d(uv)/d(xyz)f
            % d(xyz)/d(xyz)f
            dfxyz = Rc;
            
            %% d(uv)/d(xyz)f
            duvdfxyz = duvd(1:2,:) * dfxyz;        
            
            %% d(uv)/d(abgxyz)u2c part
            duvdxyz_u2c = duvd(1:2,:) * dxyz_u2c;                    

            %% d(uv)/d(abgxyz)c, 2x6
            if(pid == 1)  %dxf,dyf,dzf, (da,db,dg,dx,dy,dz)u2c
                
                %tidstart = tidend + 1;
                %tidend = tidend + 6 + 12;
                %tmpv(tidstart:tidend) = [(duvdfxyz(:))',(duvdxyz_u2c(:))'];
                zid = zid + 1;
                row = Zobs.fObs(zid).row;
                
                col = (X.feature(fid).col(:))';
                J.dUv_dX(zid).dFxyz.val = (duvdfxyz(:))';
                J.dUv_dX(zid).dFxyz.row = [ row(1) * ones(1, 3) ; row(2) * ones(1, 3) ]; 
                J.dUv_dX(zid).dFxyz.col = [ col; col ]; 
                                
                col = [X.Au2c.col(:); X.Tu2c.col(:)]';
                clen = length(col);
                rlen = length(row);
                assert( rlen*clen == length(duvdxyz_u2c(:)) );
                J.dUv_dX(zid).dATu2c.val = (duvdxyz_u2c(:))';                
                J.dUv_dX(zid).dATu2c.row = [ row(1) * ones(1, clen) ; row(2) * ones(1, clen) ]; 
                J.dUv_dX(zid).dATu2c.col = [ col ; col ]; 
                
                J.dUv_dX(zid).dAbgxyz_1 = []; %delete
                
            else % pid > 1    
                
                zid = zid + 1;
                row = Zobs.fObs(zid).row;
                
                duvddabgxyz = duvd(1:2,:) * dxyz;
                %tidstart = tidend + 1;
                %tidend = tidend + 12+6+12;%12;
                %tmpv(tidstart:tidend) = [(duvddabgxyz(:))',(duvdfxyz(:))',...
                %   (duvdxyz_u2c(:))' ];%(duvddabgxyz(:))';
                
                
                col = [(X.pose(pid-1).ang.col(:))', (X.pose(pid-1).trans.col(:))'];
                J.dUv_dX(zid).dAbgxyz_1.val = (duvddabgxyz(:))';
                J.dUv_dX(zid).dAbgxyz_1.row = [ row(1) * ones(1,6); row(2) * ones(1,6) ];
                J.dUv_dX(zid).dAbgxyz_1.col = [ col; col ];
                
                col = (X.feature(fid).col(:))';
                J.dUv_dX(zid).dFxyz.val = (duvdfxyz(:))';
                J.dUv_dX(zid).dFxyz.row = [ row(1) * ones(1,3); row(2) * ones(1,3) ];
                J.dUv_dX(zid).dFxyz.col = [ col; col ];
                                
                col = [X.Au2c.col(:); X.Tu2c.col(:)]';
                clen = length(col);
                rlen = length(row);
                assert( rlen*clen == length(duvdxyz_u2c(:)) );
                J.dUv_dX(zid).dATu2c.val = (duvdxyz_u2c(:))';
                J.dUv_dX(zid).dATu2c.row = [ row(1) * ones(1, clen) ; row(2) * ones(1, clen) ]; 
                J.dUv_dX(zid).dATu2c.col = [ col ; col ]; 
                
            end             
        end
    end
    
    return;
    
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
                nJcol = 6*(nPoses-1) + 3*nPts + 3*nPoses + 5*3;
            else
                nJ8col = 2*3*(nPoses-1);
                nJcol = 6*(nPoses-1) + 3*nPts + 3*nPoses + 3*3 + 6*(nPoses-1);
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
        J = [sparse(idRow, idCol, tmpv), sparse(nUV, nJ8col); sparse(nJ8row, nJcol)];    
        
    end % else UVOnly==0
