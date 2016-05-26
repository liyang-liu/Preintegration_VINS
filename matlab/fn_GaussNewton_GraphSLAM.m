function [X_obj, nReason] = fn_GaussNewton_GraphSLAM(K, X_obj, nPoses, nPts, Jd, CovMatrixInv, ...
                nIMUrate, nIMUdata, ImuTimestamps, dtIMU, RptFeatureObs, SLAM_Params )
    
    global PreIntegration_options Data_config
        
	nReason = 0;    
    
    load( [ Data_config.TEMP_DIR 'Zobs.mat' ] );
    
    times = 0;
    chi2_impv = 1;   % a ratio of in change in chi2 to chi2, 1 means significantly improved, 0 no improvement
    while(times < SLAM_Params.nMaxIter)
        
        fprintf('times=%d ',times);        
        
        if ( times > 0 )
            prev_chi2 = chi2;
        end
        
        %% calculate error and chi2
		E_obj = SLAM_CalcPredictionError( X_obj, nPoses, nPts );
        [e] = SLAM_Z_Object2Vector( E_obj );
        nUV = size( E_obj.fObs, 1 ) * 2;
		chi2 = 2 * e' * CovMatrixInv * e / nUV;
        
		if(isnan(chi2) || isinf(chi2))
    		nReason = -1;
		    break;
        end

        if ( times == 0 )
            chi2_impv = 1.0; % a ratio of current chi2 to previous chi2, 1 means no improvement
        else
            chi2_impv = 1 - chi2 / prev_chi2  ;
        end
        
		[me, id] = max(abs(e));
    	%%
	    fprintf('chi2=%0.6f, chi2_impv=%0.4f, maxE = %f, id=%d ', chi2, chi2_impv, me, id);
        
        %%if(max(abs(e)) < SLAM_Params.fLowerbound_e)
        if( chi2_impv < SLAM_Params.fLowerbound_chi2Cmpr )
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
        if ( PreIntegration_options.bPreInt == 1 )
            J_obj = fn_Jacobian_dIntlDelta_dX( J_obj, dtIMU, Jd, nPoses, nPts, X_obj, Zobs );
        else
            J_obj = fn_Jacobian_dImu_dX( J_obj, dtIMU, Jd, nPoses, nPts, nIMUrate, nIMUdata, X_obj, Zobs );
        end

		% %% Test IMU
		% J1 = J((nPts*nPoses*3+1):end, 1:end);
		% Info = J1'*CovMatrixInv1*J1;
		% e1 = e((nPts*nPoses*3+1):end);
		% E = -J1'*CovMatrixInv1*e1;
		%         spy(J);
		%         pause;

        J    = SLAM_J_Object2Matrix( J_obj, Zobs, X_obj );
        Info =  J' * CovMatrixInv * J;
        E    = -J' * CovMatrixInv * e;
        
		%         spy(Info)
		%         condnum = condest(Info);
		%         fprintf('Condnum = %f ',condnum);
        
        % dx = pinv(Info)*E;
        dx = Info\E;

		% R = chol(Info);
		% em = speye(size(Info,1));
		% y = R' \ em;
		% tv = R \ y;
		% dx = tv*E;
        
		mdx = max( abs( dx ) );
        
		fprintf('maxDx = %f\n', mdx); 
        
        %dx = - (J' * J) \ J' * e;    
        
        if( mdx < SLAM_Params.fLowerbound_dx )
            break;            
        elseif( isnan( mdx ) || isinf( mdx ))            
            nReason = -2;
            break;
        end
        
        X_vec = X_vec + dx;
        X_obj = SLAM_X_Vector2Object( X_vec, X_obj );
        
        times = times + 1;
        
    end		% while end    
    
    fprintf( 'Iteration Times:\t N = %d\n', times );
    
    if( times == SLAM_Params.nMaxIter )
        nReason = -3;
        fprintf( ' ***REACH MAXIMUM TIMES***\n' );
    end
    
    
    %X_obj = SLAM_X_Vector2Object( x, X_obj );