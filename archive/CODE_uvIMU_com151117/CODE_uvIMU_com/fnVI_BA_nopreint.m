function [x] = fnVI_BA_nopreint(K, x, nPoses, nPts, dt, Jd, CovMatrixInv, nMaxIter, fLowerbound_e, fLowerbound_dx)

    times = 0;    
    while(times < nMaxIter)
        fprintf('times=%d ',times);
%         [e] = fnCnUPredErr(K,x,Zobs,nPoses,nPts,bf0,bw0,dt,Jd);%g,
% Debug v2
[e] = fnCnUPredErr_lsqnonlin(x);
        %%
     fprintf('maxE = %f ', max(abs(e)));
        if(max(abs(e)) < fLowerbound_e)
            break;
        end
%         % Original        
%         J = fnJduvd_CnU(K, x, nPoses, nPts);
%% Debug v2
J = fnJduvd_CnU_dbg(K, x, nPoses, nPts);


% %% To test the UVD part only        
% J1 = J(1:(3*nPts*nPoses), 1:(3*nPts+6*(nPoses-1)));
% Info = J1'*CovMatrixInv1*J1;
% e1 = e(1:(3*nPts*nPoses));
% E = -J1'*CovMatrixInv1*e1;
%         %% Original
%         J = fnJddpvphi_IMU(J, dt, g0, Jd, nPoses, nPts, x);
%% Debug v2
J = fnJddpvphi_IMU_dbg(J, dt, Jd, nPoses, nPts, x);%g0, 
% %% Test IMU
% J1 = J((nPts*nPoses*3+1):end, 1:end);
% Info = J1'*CovMatrixInv1*J1;
% e1 = e((nPts*nPoses*3+1):end);
% E = -J1'*CovMatrixInv1*e1;
%         spy(J);
%         pause;
        Info = J'*CovMatrixInv*J;
        E = -J'*CovMatrixInv*e;
        dx = Info\E;
 fprintf('maxDx = %f\n', max(abs(dx))); 
        %dx = - (J' * J) \ J' * e;    
        if(max(abs(dx)) < fLowerbound_dx)
            break;
        end
% %% C part        
% x(1:(tidx)) = x(1:(tidx)) + dx(1:(tidx));
% %% IMU part
% x = x + dx;
        x = x + dx;
        times = times + 1;
    end    
    fprintf('\nIteration Times:\n\t N = %d\n', times);
    if( times == nMaxIter)
        fprintf(' ***REACH MAXIMUM TIMES***\n');
    end