function [X_obj, nReason] = fn_GaussNewton_GraphSLAM(K, X_obj, nPoses, nPts, Jd, CovMatrixInv, nMaxIter, ...
                fLowerbound_e, fLowerbound_dx, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs )
    
    global InertialDelta_options Data_config
        
	nReason = 0;    
    
    load( [ Data_config.TEMP_DIR 'Zobs.mat' ] );
    
    times = 0;    
    while(times < nMaxIter)
        
        fprintf('times=%d ',times);        
        
		E_obj = SLAM_CalcPredictionError(X_obj);
        [e] = SLAM_Z_Object2Vector( E_obj );
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
        
        X_vec = SLAM_X_Object2Vector( X_obj );
        
  		%%%%%%%%%%%%  L M            
		if((chi2 < 1e0) && (max(abs(X_vec)) > 1e3))%30)%- 3-15/30
		    fprintf('\n');
    		[x,nReason,Info] = fn_LeastSqrLM_GraphSLAM(nUV, K, x, nPoses, nPts, Jd, ...
                        CovMatrixInv, nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs );  
    		break;
		end
  		%%%%%%%%%%%%

        J_obj = SLAM_Jacobian_Define( );
        J_obj = fn_Jacobian_dUv_dX( J_obj, K, X_obj, Zobs, nPoses, nPts, nIMUdata, ImuTimestamps, RptFeatureObs );		
        J_obj = fn_Jacobian_dIntlDelta_dX( J_obj, dtIMU, Jd, nPoses, nPts, X_obj, Zobs );

		% %% Test IMU
		% J1 = J((nPts*nPoses*3+1):end, 1:end);
		% Info = J1'*CovMatrixInv1*J1;
		% e1 = e((nPts*nPoses*3+1):end);
		% E = -J1'*CovMatrixInv1*e1;
		%         spy(J);
		%         pause;

        J = SLAM_J_Object2Matrix( J_obj, Zobs, X_obj );
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
        X_obj = SLAM_X_Vector2Object( X_vec, X_obj );
        
        times = times + 1;
        
    end		% while end    
    
    fprintf('Iteration Times:\t N = %d\n', times);
    
    if(times == nMaxIter)
        nReason = -3;
        fprintf(' ***REACH MAXIMUM TIMES***\n');
    end
    
    
    %X_obj = SLAM_X_Vector2Object( x, X_obj );