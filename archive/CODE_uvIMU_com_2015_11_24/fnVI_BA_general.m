function [x, nReason] = fnVI_BA_general(nUV, K, x, nPoses, nPts, Jd, CovMatrixInv, nMaxIter, ...
    fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
    bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias)

	nReason = 0;
    [idRow, idCol, nJacs] = fnFndJacobianID(nIMUdata, bUVonly, bPreInt, nPoses, RptFeatureObs, ImuTimestamps);
    if(bUVonly == 1)% UVonly, add Au2c,Tu2c and Z2
        idx_au2c = (nPoses-1)*6+nPts*3;
        uidRow = [1,2,3,1,2,3,1,2,3, ... %Au2c
                  4,5,6,4,5,6,4,5,6, ... %Tu2c
                  7 ... % Z2
                  ];                  
        uidCol = [idx_au2c+1,idx_au2c+1,idx_au2c+1, ... 
                  idx_au2c+2,idx_au2c+2,idx_au2c+2, ... 
                  idx_au2c+3,idx_au2c+3,idx_au2c+3, ... %Au2c
                  idx_au2c+4,idx_au2c+4,idx_au2c+4, ... 
                  idx_au2c+5,idx_au2c+5,idx_au2c+5, ... 
                  idx_au2c+6,idx_au2c+6,idx_au2c+6, ... %Tu2c
                  6 ... %4 for Malaga
                 ]; 
        unJacs = 3*3*2+1; 
    else%if(bPreInt == 1)
        [uidRow, uidCol, unJacs] = fnFndJacIDimu(ImuTimestamps, nIMUdata, bUVonly, nPoses, nPts, bAddZg, ...
            bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias, bPreInt);        
    end
    
    times = 0;    
    while(times < nMaxIter)
        fprintf('times=%d ',times);
		%         [e] = fnCnUPredErr(K,x,Zobs,nPoses,nPts,bf0,bw0,dt,Jd);%g,
		% Debug v2
		[e] = fnCnUPredErr_lsqnonlin_general(x);
		chi2 = 2*e'*CovMatrixInv*e/nUV;
		if(isnan(chi2) || isinf(chi2))
    		nReason = -1;
		    break;
		end
		[me, id] = max(abs(e));
    	%%
	    fprintf('chi2=%0.8f, maxE = %f, id=%d ', chi2, me, id);
        if(max(abs(e)) < fLowerbound_e)
            break;
        end
  		%%%%%%%%%%%%  L M    
        % Dont do Levenbug Macquadt %
        if 0    
            if((chi2 < 1e0) && (max(abs(x)) > 1e3))%30)%- 3-15/30
                fprintf('\n');
                [x,nReason,Info] = fnleastsquaresLM(nUV, K, x, nPoses, nPts, Jd, ...
                CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, ...
                bUVonly, bPreInt, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf,bAddZbw, bVarBias);  
                break;
            end
        end
  		%%%%%%%%%%%%

		%         % Original        
		%         J = fnJduvd_CnU(K, x, nPoses, nPts);
		%% Debug v2

		% [J] = fnJduvd_CnU_general(K, x, nPoses, nPts, nIMUdata, ImuTimestamps, RptFeatureObs, ...
		%     bUVonly, bPreInt);
	[J] = fnJduvd_CnU_gq(nJacs, idRow, idCol, K, x, nPoses, nPts, nIMUdata, ImuTimestamps, ...
    	RptFeatureObs, bUVonly, bPreInt, nUV, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, ...
    	bAddZbw, bVarBias);


		% %% To test the UVD part only        
		% J1 = J(1:(3*nPts*nPoses), 1:(3*nPts+6*(nPoses-1)));
		% Info = J1'*CovMatrixInv1*J1;
		% e1 = e(1:(3*nPts*nPoses));
		% E = -J1'*CovMatrixInv1*e1;
		%         %% Original
		%         J = fnJddpvphi_IMU(J, dt, g0, Jd, nPoses, nPts, x);
		%% Debug v2

		if((bUVonly == 1) || (bPreInt == 1))
		%     J = fnJddpvphi_IMU_general(nUV, bUVonly, J, dtIMU, Jd, nPoses, nPts, x, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);%g0, 
		    J((nUV+1):end,:) = fnJddpvphi_IMU_gq(uidRow, uidCol, unJacs, nUV, bUVonly, dtIMU, Jd, nPoses, nPts, x, ...
        		    bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias);
		else
			%     J = fnJdaw0_IMU_general(nUV, J, nPts, x, nIMUrate, nIMUdata, ...
			%         bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);
		    %J = fnJdaw0_IMU_general(J, nPoses, nPts, x, nIMUdata, bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw);
		    J((nUV+1):end,:) = fnJdaw0_IMU_gq(uidRow, uidCol, unJacs, nUV, nPts, x, nIMUrate, nIMUdata, ...
		        bAddZg, bAddZau2c, bAddZtu2c, bAddZbf, bAddZbw, bVarBias, nPoses);    
		end

		% %% Test IMU
		% J1 = J((nPts*nPoses*3+1):end, 1:end);
		% Info = J1'*CovMatrixInv1*J1;
		% e1 = e((nPts*nPoses*3+1):end);
		% E = -J1'*CovMatrixInv1*e1;
		%         spy(J);
		%         pause;

        Info = J'*CovMatrixInv*J;
        E = -J'*CovMatrixInv*e;
		%         spy(Info)
		%         condnum = condest(Info);
		%         fprintf('Condnum = %f ',condnum);
        dx = Info\E;

		% R = chol(Info);
		% em = speye(size(Info,1));
		% y = R' \ em;
		% tv = R \ y;
		% dx = tv*E;
		mdx = max(abs(dx));
		 fprintf('maxDx = %f\n', mdx); 
        %dx = - (J' * J) \ J' * e;    
        if(mdx < fLowerbound_dx)
            break;
        elseif(isnan(mdx) || isinf(mdx))
            nReason = -2;
            break;
        end
		% %% C part        
		% x(1:(tidx)) = x(1:(tidx)) + dx(1:(tidx));
		% %% IMU part
		% x = x + dx;
        x = x + dx;
        times = times + 1;
    end		% while end    
    fprintf('Iteration Times:\t N = %d\n', times);
    if(times == nMaxIter)
        nReason = -3;
        fprintf(' ***REACH MAXIMUM TIMES***\n');
%     elseif((sum(isinf(x)) > 0) || (sum(isnan(x)) > 0) || isinf(chi2)...
%             || isnan(chi2))
%         nReason = -1;
%     else
%         nReason = 1;
    end