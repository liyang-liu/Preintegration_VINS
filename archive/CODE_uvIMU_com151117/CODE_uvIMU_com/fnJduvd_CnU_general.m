function [J] = fnJduvd_CnU_general(K, x, nPoses, nPts, nIMUdata, ImuTimestamps, ...
    RptFeatureObs, bUVonly, bPreInt)
%% Objective function elements: ei = (ui' - ui)^2, ei'= (vi' - vi)^2 (i=1...N)
% Find R, T corresponding to 3D points pi and pi'.
% 
% K: camera model
% p3d0: 3D points at the first camera pose
% x: current estimate of the states (alpha, beta, gamma, T)

nObsId_FeatureObs = 2;
fx = K(1,1); cx0 = K(1,3); fy = K(2,2); cy0 = K(2,3);
% J = sparse(3*nPts*nPoses+3*3*(nPoses-1), 3*nPts+6*(nPoses-1)+3*nPoses+5*3);
if(bUVonly == 1)
    J = sparse(2*nPts*nPoses+3*2+1, 3*nPts+6*(nPoses-1)+3*2);
else
    if(bPreInt == 1)
        J = sparse(2*nPts*nPoses+3*3*(nPoses-1), 3*nPts+6*(nPoses-1)+3*nPoses+5*3);%+3*3+3*nPoses
    else%nIMUrate*(nPoses-1) nIMUrate*(nPoses-1)
        J = sparse(2*nPts*nPoses+9*(nIMUdata-1), 3*nPts+6*(nIMUdata-1)+3*nIMUdata+5*3);
    end
end

% Section for pose 1
if((bUVonly == 1) || (bPreInt == 1))
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
if(bUVonly == 1)
    idx = (nPoses-1)*6+nPts*3+1;
else
    if(bPreInt == 1)
        idx = (nPoses-1)*6+nPts*3+3*nPoses+4;    
    else
        idx = 6*nIMUrate*(nPoses-1)+nPts*3+3*((nPoses-1)*nIMUrate+1)+4;
    end
end
a_u2c = x(idx); b_u2c = x(1+idx); g_u2c = x(2+idx);
Ru2c = fnR5ABG(a_u2c, b_u2c, g_u2c);%fRx(alpha) * fRy (beta) * fRz(gamma);         
Tu2c = x((3+idx):(5+idx), 1);
nUV = 0;
% Section for all of the poses
for(fid=1:nPts)
    nObs = RptFeatureObs(fid, nObsId_FeatureObs);
    for(oid=1:nObs)
        pid = RptFeatureObs(fid, oid*3); 
        if(pid > 1)
            if((bUVonly == 1) || (bPreInt == 1))
                idx = (pid-2)*6;    
            else
                idx = (ImuTimestamps(pid)-ImuTimestamps(1)-1)*6;%6*nIMUrate*(pid-1)-6;
            end
            a = x(1+idx); b = x(2+idx); g = x(3+idx); 
            Ru = fnR5ABG(a, b, g);%fRx(alpha) * fRy (beta) * fRz(gamma);
            Tu = x((4+idx):(idx+6), 1);            
        else % pid ==1, Ru2c,Tu2c
            a = 0; b = 0; g = 0;
            Ru = eye(3); 
            Tu = zeros(3,1);
        end
    %     R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
        Rc = Ru2c * Ru;
        Tc = Ru' * Tu2c + Tu;
        ncurrentObsdPts = 1;%size(obsfeatures{pid}, 1);
        p3d1 = Rc * (p3d0(:, fid) - repmat(Tc, 1, ncurrentObsdPts));    
        % Find the gradient of u(x,y,z) and v(x,y,z) at the second pose
        [duvd] = fnuvd5xyz_dr_general(p3d1, fx, fy, ncurrentObsdPts);%[duvd] = fxyz2uvd_dr(p3d1(1,i), p3d1(2,i), p3d1(3,i), f);
        % Find the gradient of xyz(alpha, beta, gamma, T) at the second pose
    % 	[dxyz] = fxyz5abgxyz_dr(alpha, beta, gamma, p3d0, N);
        [dxyz,dxyz_u2c, ~] = fnxyz5abgxyz_drCIU(a, b, g, a_u2c, b_u2c, g_u2c, Ru, Ru2c, ...
                        Tu, Tu2c, p3d0(:, fid), ncurrentObsdPts);%p3d0, nPts);
    %     duvddabgxyz = zeros(3*nPts, 3*nPts+6*(nPoses-1)+3*nPoses+3+6+6);
        if(bUVonly == 1)
            duvddabgxyz = sparse(2*ncurrentObsdPts, 3*nPts+6*(nPoses-1)+3*2);
        else
            if(bPreInt == 1)
                duvddabgxyz = sparse(2*ncurrentObsdPts, 3*nPts+6*(nPoses-1)+3*nPoses+3+6+6);
            else
               duvddabgxyz = sparse(2*ncurrentObsdPts, 3*nPts+6*nIMUrate*(nPoses-1)+3*((nPoses-1)*nIMUrate+1)+5*3); 
            end
        end
        for i = 1:ncurrentObsdPts%nPts
            if(bUVonly == 1)
                dabgTxyz = zeros(3, 3*nPts+6*(nPoses-1)+3*2);
            else
                if(bPreInt == 1)
                    dabgTxyz = sparse(3, 3*nPts+6*(nPoses-1)+3*nPoses+3+6+6);
                else
                   dabgTxyz = sparse(3, 3*nPts+6*nIMUrate*(nPoses-1)+3*((nPoses-1)*nIMUrate+1)+5*3); 
                end
            end
            %dabgTxyz = zeros(3, (nPoses-1)*6+nPts*3+3*nPoses+3+6+6);
            if(pid > 1)
                % Ru part
                if((bUVonly == 1) || (bPreInt == 1))
                    idx = 6*(pid-2);
                else
                   idx = (ImuTimestamps(pid)-ImuTimestamps(1)-1)*6;%6*nIMUrate*(pid-1)-6; 
                end
                dabgTxyz(:,(idx+1):(6+idx)) = dxyz(:,(6*(i-1)+1):(6*i));%dabgT            
            end
            % x0 part
            if((bUVonly == 1)||(bPreInt == 1))
                idx = 6*(nPoses-1)+3*(fid-1);
            else
               idx = 6*nIMUdata+3*(fid-1);%nIMUrate*(nPoses-1) 
            end        
            dabgTxyz(:, (idx+1):(idx+3)) = Rc;%% dx0
            % Ru2c,Tu2c part
            if(bUVonly == 1)
                idx = (nPoses-1)*6+nPts*3+1;
            else
                if(bPreInt == 1)
                    idx = (nPoses-1)*6+nPts*3+3*nPoses+4;%idx = (nPoses-1)*6+nPts*3+3*nPoses+4;
                else%nIMUrate*(nPoses-1) (nPoses-1)*nIMUrate+1
                    idx = 6*(nIMUdata-1)+nPts*3+3*nIMUdata+4;
                end  
            end
            dabgTxyz(:,(idx):(idx+5)) = dxyz_u2c(:,(6*(i-1)+1):(6*i));%dabgT_u
            % Complete the chain
    %         duvddabgxyz((3*(i-1)+1):(3*i), :) = duvd(((i-1)*3+1):(3*i),:) * dabgTxyz;        
            duvddabgxyz((2*(i-1)+1):(2*i), :) = duvd(((i-1)*3+1):(3*i-1),:) * dabgTxyz;
        end

        J((nUV+1):(nUV+2*ncurrentObsdPts),:) = duvddabgxyz;% 2*nPts*(pid-1)%2*nPts*pid
        nUV = nUV + 2*ncurrentObsdPts;
    %     J((3*nPts*(pid-1)+1):(3*nPts*pid),:) = duvddabgxyz;
    end
end

if(bUVonly == 1)
    J = J(1:(nUV+3*2+1), :);
else
    if(bPreInt == 1)
        J = J(1:(nUV+3*3*(nPoses-1)), :);%+3*3+3*nPoses
    else%nIMUrate*(nPoses-1) nIMUrate*(nPoses-1)
        J = J(1:(nUV+9*(nIMUdata-1)), :);
    end
end


