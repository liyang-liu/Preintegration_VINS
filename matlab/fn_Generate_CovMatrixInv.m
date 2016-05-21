function [CovMatrixInv] = fn_Generate_CovMatrixInv( SLAM_Params, Zobs, Rd )
    %% Covariance matrix
    % Original     
    %     CovMatrixInv = zeros((nPts*nPoses*3+(nPoses-1)*3*3));
    
    global PreIntegration_options

    if 0
        if(PreIntegration_options.bPreInt == 0)
            nlenpp = nIMUrate * 3 * 3; % IMU raw data observations between two key frames: (wi,ai,dTi), dTi refer to diff(Ti+1, Ti) ==  Zero contraints
        else
            nlenpp = 0;
        end

        if(PreIntegration_options.bPreInt == 1)
            utid = idr+(nPoses-1)*3*3+15;% initialized with possible maximum size.
        else
            utid = idr + (nPoses - 1)*nlenpp+15;
        end
    end
    
    %CovMatrixInv = speye(utid);
    
    nUV = length(Zobs.fObs) * 2;
    CovInv_Fobs_obj = struct( ...
        'val',  1 / (SLAM_Params.sigma_uov_cov * SLAM_Params.sigma_uov_cov ) * eye(nUV) ...            
        );        
    
    nPoses = length( Zobs.intlDelta ) + 1;
    
    % Initialize the part corresponding to IMU data
    if(PreIntegration_options.bPreInt == 1)
        CovInv_IntlDelta_obj = struct( ...
            'val', eye( (nPoses - 1) * 3 * 3 ) ...
            );
        for pid = 2:nPoses
            covInv = 1e0*inv(Rd{pid}(1:9,1:9));    
            %CovMatrixInv((idr+9*(pid-2)+1):(idr+9*(pid-1)), (idr+9*(pid-2)+1):(idr+9*(pid-1))) = covInv;
            CovInv_IntlDelta_obj.val( 9 * ( pid - 2) + 1: 9 * (pid - 1), 9 * ( pid - 2) + 1: 9 * (pid - 1) ) = covInv;
        end
        %utid = idr+(nPoses-1)*3*3;
    else
        CovInv_IntlDelta_obj = struct( ...
            'val', eye( (nPoses) * 3 * 3 * nIMUrate ) ...
            );
        q = inv(diag([sigma_w_cov*sigma_w_cov*ones(3,1); sigma_f_cov*sigma_f_cov*ones(3,1); sigma_tv*sigma_tv*ones(3,1)]));
        for pid = 1:((nPoses-1)*nIMUrate)
            %CovMatrixInv((idr+1):end,(idr+1):end) =
            % kron(eye((nPoses-1)*nIMUrate),q); % not suitalbe for large scale
            CovInv_IntlDelta_obj.val = kron(eye((nPoses-1)*nIMUrate), q);
            % computation
                %%CovMatrixInv((idr+9*(pid-1)+1):(idr+9*(pid)), (idr+9*(pid-1)+1):(idr+9*(pid))) = q;
            
        end
        %utid = idr+(nPoses-1)*nlenpp;
    end
    % Initilized additional parts
    if(PreIntegration_options.bAddZg == 1)
        %CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        %utid = utid + 3;
        CovInv_g_obj = struct( ...
            'val', 1e8*eye(3) ...
            );
        
    end
    if(PreIntegration_options.bAddZau2c == 1)
        %CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = %1e8*eye(3);
       % utid = utid + 3;
        CovInv_Au2c_obj = struct( ...
            'val', 1 / (SLAM_Params.sigma_au2c_cov * SLAM_Params.sigma_au2c_cov ) * eye( 3 ) ...
            );
    end
    
    if(PreIntegration_options.bAddZtu2c == 1)
        %CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        %utid = utid + 3;
        CovInv_Tu2c_obj = struct( ...
            'val', 1 / (SLAM_Params.sigma_tu2c_cov * SLAM_Params.sigma_tu2c_cov ) * eye( 3 ) ...
            );
    end
    
    if( PreIntegration_options.bAddZbf == 1)
        %CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        %utid = utid + 3;
        CovInv_Bf_obj = struct( ...
            'val', 1 / (SLAM_Params.sigma_bf_cov * SLAM_Params.sigma_bf_cov ) * eye( 3 ) ...
            );
        
    end 
    if(PreIntegration_options.bAddZbw == 1)
        %CovMatrixInv((utid+1):(utid+3), (utid+1):(utid+3)) = 1e8*eye(3);
        %utid = utid + 3;
        CovInv_Bw_obj = struct( ...
            'val', 1 / (SLAM_Params.sigma_bw_cov * SLAM_Params.sigma_bw_cov ) * eye( 3 ) ...
            );
        
    end
    
    %CovMatrixInv = CovMatrixInv(1:utid,1:utid);
    CovInv_obj = struct( ...
        'fObs',         CovInv_Fobs_obj, ...
        'intlDelta',    CovInv_IntlDelta_obj, ...
        'g',            CovInv_g_obj, ...
        'Au2c',         CovInv_Au2c_obj, ...
        'Tu2c',         CovInv_Tu2c_obj, ...
        'Bf',           CovInv_Bf_obj, ...
        'Bw',           CovInv_Bw_obj ...
        );
    
    
    nRow = Zobs.Bw.row(end); %matrix dim    
    CovMatrixInv = SLAM_CovInv_Object2Matrix( CovInv_obj, nRow );
    