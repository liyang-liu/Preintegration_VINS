function [CovMatrixInv] = fn_Generate_CovMatrixInv2( nPoses, nPts, nUV, nIMUrate, Rd, SLAM_Params )
    %% Covariance matrix
    % Original     
    %     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3));
    
    global PreIntegration_options Data_config

  idr = nUV;
    
   if(PreIntegration_options.bPreInt == 1)
        utid = idr+(nPoses-1)*3*3+15;% initialized with possible maximum size.
   else
        nlenpp = nIMUrate * 3 * 3;
        utid = idr + (nPoses - 1)*nlenpp + 15;
   end

   CovMatrixInv = speye(utid);
   
    if(PreIntegration_options.bPreInt == 1)
        for pid = 2:nPoses
            covInv = 1e0*inv(Rd{pid}(1:9,1:9));    
            CovMatrixInv((idr+9*(pid-2)+1):(idr+9*(pid-1)), (idr+9*(pid-2)+1):(idr+9*(pid-1))) = covInv;
        end
        utid = idr+(nPoses-1)*3*3;
    else
        q = inv( diag( ...
                [ SLAM_Params.sigma_w_cov * SLAM_Params.sigma_w_cov * ones(3,1); ...
                    SLAM_Params.sigma_f_cov * SLAM_Params.sigma_f_cov * ones(3,1); ...
                    SLAM_Params.sigma_tv * SLAM_Params.sigma_tv * ones(3,1) ] ...
             ));
        for pid = 1:((nPoses-1)*nIMUrate)
        %CovMatrixInv((idr+1):end,(idr+1):end) =
        % kron(eye((nPoses-1)*nIMUrate),q); % not suitalbe for large scale
        % computation
            CovMatrixInv((idr+9*(pid-1)+1):(idr+9*(pid)), (idr+9*(pid-1)+1):(idr+9*(pid))) = q;
        end
        utid = idr+(nPoses-1)*nlenpp;
    end
    % Initilized additional parts
    if(PreIntegration_options.bAddZg == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(PreIntegration_options.bAddZau2c == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(PreIntegration_options.bAddZtu2c == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    if(PreIntegration_options.bAddZbf == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end 
    if(PreIntegration_options.bAddZbw == 1)
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        utid = utid + 3;
    end
    
    CovMatrixInv = CovMatrixInv(1:utid,1:utid);
    save( [Data_config.TEMP_DIR, 'CovMatrixInv.mat'],'CovMatrixInv', '-v7.3');

    