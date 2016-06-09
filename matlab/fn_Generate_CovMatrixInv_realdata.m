function CovMatrixInv = fn_Generate_CovMatrixInv_realdata( nPoses, nPts, nUV, nIMUdata, Rd, SLAM_Params )
    global PreIntegration_options
    
    % Original     
    %     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3))
    if( PreIntegration_options.bUVonly == 1 )
        utid = zidend;
        CovMatrixInv = speye(utid + 6 + 1);
    else
       if ( PreIntegration_options.bPreInt == 1 )
            utid = nUV + (nPoses-1)*3*3+15;% initialized with possible maximum size.
       else
            utid = nUV + nIMUdata*9 +15;%(nPoses - 1)*nlenpp
       end

        CovMatrixInv = speye(utid);        
        % Initialize the part corresponding to IMU data
        if ( PreIntegration_options.bPreInt == 1 )
            for pid = 2:nPoses
                covInv = 1e0*inv(Rd{pid}(1:9,1:9)); %2e0 1e0-1 -2 -4  
                CovMatrixInv((nUV+9*(pid-2)+1):(nUV+9*(pid-1)), (nUV+9*(pid-2)+1):(nUV+9*(pid-1))) = covInv;
            end
            utid = nUV+(nPoses-1)*3*3;
        else
            q = inv( diag( ...
                    [   SLAM_Params.sigma_w_cov^2 * ones(3,1); ...
                        SLAM_Params.sigma_f_cov^2 * ones(3,1); ...
                        SLAM_Params.sigma_tv^2 * ones(3,1) ] ...
                ));%1e-6
            for pid = 1:nIMUdata%((nPoses-1)*nIMUrate
            %CovMatrixInv((idr+1):end,(idr+1):end) =
            % kron(eye((nPoses-1)*nIMUrate),q); % not suitalbe for large scale
            % computation
                CovMatrixInv((nUV+9*(pid-1)+1):(nUV+9*(pid)), (nUV+9*(pid-1)+1):(nUV+9*(pid))) = q;
            end
            utid = nUV+nIMUdata*9;%(nPoses-1)*nlenpp;
        end
    % Initilized additional parts
        if ( PreIntegration_options.bAddZg == 1 )
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
               1 / (SLAM_Params.sigma_g_cov^2) * eye(3);
            utid = utid + 3;
        end
    end 
    CovMatrixInv(1:nUV,1:nUV) = 1/(SLAM_Params.sigma_uov_cov^2) * CovMatrixInv(1:nUV, 1:nUV);
    if ( PreIntegration_options.bAddZau2c == 1 )
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
            1 / ( SLAM_Params.sigma_au2c_cov ^2 ) * eye(3);
        utid = utid + 3;
    end
    
    if ( PreIntegration_options.bAddZtu2c == 1 )
        CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
            1/(SLAM_Params.sigma_tu2c_cov^2) * eye(3);
        utid = utid + 3;
    end
    
    if ( PreIntegration_options.bUVonly == 1 )% Add A2, T2 as additional observation
        CovMatrixInv((utid+1), (utid+1)) = 1e8;
        %CovMatrixInv((utid+1):(utid+6), (utid+1):(utid+6)) = 1e8*eye(6);
        utid = utid + 1;%6;        
    elseif( PreIntegration_options.bVarBias == 0 )
        if ( PreIntegration_options.bAddZbf == 1 )
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                1 / ( SLAM_Params.sigma_bf_cov^2 ) * eye(3);%1e8
            utid = utid + 3;
        end 
        if ( PreIntegration_options.bAddZbw == 1 )
            CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = ...
                1 / ( SLAM_Params.sigma_bw_cov^2 ) * eye(3);
            utid = utid + 3;
        end
    else
        for(pid=2:(nPoses-1))
            tv = eye(6);
            tv(1:3, 1:3) = (1 / SLAM_Params.sigma_bf_cov)^2 * tv(1:3, 1:3);%(bfi-bfi1)
            tv(4:6, 4:6) = (1 / ( dtIMU(pid) * SLAM_Params.sigma_bw_cov )) ^ 2 * tv(4:6, 4:6);
            CovMatrixInv( (utid+1):(utid+6), (utid+1):(utid+6) ) = tv; %1e8
            utid = utid + 6; 
        end
    end
    CovMatrixInv = CovMatrixInv(1:utid,1:utid);
