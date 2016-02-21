function [X_obj, nReason] = fnGaussNewton_GraphSLAM(K, X_obj, nPoses, nPts, Jd, CovMatrixInv, nMaxIter, ...
                fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs )
    
    global InertialDelta_options Data_config
    
    
	nReason = 0;
    if 0 % YouBing
        [idRow, idCol, nJacs] = fnFndJacobianID(nIMUdata, nPoses, RptFeatureObs, ImuTimestamps);

        if(InertialDelta_options.bUVonly == 1)% UVonly, add Au2c,Tu2c and Z2
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

        else%if(InertialDelta_options.bPreInt == 1)

            [uidRow, uidCol, unJacs] = fnFndJacIDimu(ImuTimestamps, nIMUdata, nPoses, nPts);        
        end
    end
    
    load( [ Data_config.TEMP_DIR 'Zobs.mat' ] );
    
    times = 0;    
    while(times < nMaxIter)
        
        fprintf('times=%d ',times);
		% [e] = fnCnUPredErr(K,x,Zobs,nPoses,nPts,bf0,bw0,dt,Jd);%g,
		% Debug v2
        
        
		E_obj = fnCalcPredictionError(X_obj);
        [e] = ZObject2Vector( E_obj );
        nUV = size( E_obj.fObs, 1 ) * 2;
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
        
        X_vec = XObject2Vector( X_obj );
        
  		%%%%%%%%%%%%  L M            
		if((chi2 < 1e0) && (max(abs(X_vec)) > 1e3))%30)%- 3-15/30
		    fprintf('\n');
    		[x,nReason,Info] = fnLeastSqrLM_GraphSLAM(nUV, K, x, nPoses, nPts, Jd, ...
    	    CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs );  
    		break;
		end
  		%%%%%%%%%%%%

        J_obj = InertialDelta_InitJacobian( );
        %[J] = fnJduvd_CnU_gq( nJacs, idRow, idCol, K, x, nPoses, nPts, nIMUdata, ...
        %                       ImuTimestamps, RptFeatureObs, nUV );
        J_obj = fnJacobian_dUv_dX(J_obj, K, X_obj, Zobs, nPoses, nPts, nIMUdata, ImuTimestamps, RptFeatureObs );


		if((InertialDelta_options.bUVonly == 1) || (InertialDelta_options.bPreInt == 1))
		
		    %J((nUV+1):end,:) = fnJddpvphi_IMU_gq(uidRow, uidCol, unJacs, nUV, dtIMU, Jd, nPoses, nPts, x );
            J_obj = fnJacobian_dIntlDelta_dX( J_obj, dtIMU, Jd, nPoses, nPts, X_obj, Zobs );
        else
            
			%   J = fnJdaw0_IMU_general(nUV, J, nPts, x, nIMUrate, nIMUdata);
		    %   J = fnJdaw0_IMU_general(J, nPoses, nPts, x, nIMUdata);
            
		    %J((nUV+1):end,:) = fnJdaw0_IMU_gq(uidRow, uidCol, unJacs, nUV, nPts, x, nIMUrate, nIMUdata, nPoses);    
            J((nUV+1):end,:) = fnJdaw0_IMU_gq(uidRow, uidCol, unJacs, nUV, nPts, x, nIMUrate, nIMUdata, nPoses);    
            
		end

		% %% Test IMU
		% J1 = J((nPts*nPoses*3+1):end, 1:end);
		% Info = J1'*CovMatrixInv1*J1;
		% e1 = e((nPts*nPoses*3+1):end);
		% E = -J1'*CovMatrixInv1*e1;
		%         spy(J);
		%         pause;

        J = JObject2Matrix( J_obj, Zobs, X_obj );
        Info =  J'*CovMatrixInv*J;
        E =     -J'*CovMatrixInv*e;
        
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
        %x = x + dx;
        X_vec = X_vec + dx;
        X_obj = XVector2Object( X_vec, X_obj );
        
        times = times + 1;
        
    end		% while end    
    
    fprintf('Iteration Times:\t N = %d\n', times);
    
    if(times == nMaxIter)
        nReason = -3;
        fprintf(' ***REACH MAXIMUM TIMES***\n');
    end
    
    
    %X_obj = XVector2Object( x, X_obj );