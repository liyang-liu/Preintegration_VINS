function [imuData_cell, uvd_cell, Ru_cell, Tu_cell, FeatureObs, SLAM_Params] = fn_GenerateFeatureObs(  nPoses, nPts, nIMUrate, SLAM_Params )

    global PreIntegration_options Data_config
    
    %% Use previous observation data or not
    if(PreIntegration_options.bUsePriorZ == 0)   
    
        arPts = [0,0,0; 1,0.5,2;0.5,2,2.6;1.8,2.4,3.5];
        
        %     [imuData_cell, uvd_cell, Ru_cell, Tu_cell, Ru2c, Tu2c, vu] = ...
        %         fnSimIMUnCameraFeatures1Pts(arPts, nPts, Ru2c, Tu2c, g0, bf0, bw0, nIMUrate, bPreInt);%
        [imuData_cell, uvd_cell, Ru_cell, Tu_cell, vu] = ...
            fn_SimIMUnFeaturesAtNPoses_helix( nPoses, nPts, nIMUrate, SLAM_Params);% line   


        if(PreIntegration_options.bTestIMU == 1)
            fn_TestIMU_cmp(nPoses, imuData_cell, Ru_cell, Tu_cell, ...
                                vu, SLAM_Params );
        end
        
        % Define data structure for camera observation
        Observation_Def = struct( ...
            'pid', [], ...
            'uv',  zeros(1,2) ...
            );
        FeatureInfo_Def = struct( ...
            'fid',  [], ...
            'nObs', 0, ...
            'obsv', Observation_Def ... % array of observations, size will grow
            );
        FeatureObs = repmat( FeatureInfo_Def, nPts, 1);

        %% Add noise to observations
        %%
        
        %% FeatureObs = zeros(nPts, 300);%[fid, nObs, [pid, ui,vi]]
        %%     fId_FeatureObs = 1; nObsId_FeatureObs = 2;        
        %% FeatureObs(:, fId_FeatureObs) = 1:nPts;    
        fids = num2cell( 1:nPts );
        [FeatureObs(:).fid] = fids{:};        
    
        [~,nc] = size(uvd_cell{1});
        
        for pid=1:nPoses
            %	[gns] = fn_GenGaussNoise(1, nc, [1;1;0.01]);
            %   uvd_cell{pid} = uvd_cell{pid} + gns;  
            [gns] = fn_GenGaussNoise(2, nc, SLAM_Params.sigma_uov_real);
            uvd_cell{pid}(1:2,:) = uvd_cell{pid}(1:2,:) + gns;
            
            %% FeatureObs(:,nObsId_FeatureObs) = FeatureObs(:,nObsId_FeatureObs) + 1;
            %% nObs = FeatureObs(:, nObsId_FeatureObs);
            %% FeatureObs(:, 3*nObs) = pid;
            %% FeatureObs(:, (3*nObs+1):(3*nObs+2)) = (uvd_cell{pid}(1:2,:))';
            for ( f=1:nPts )
                FeatureObs(f).nObs = FeatureObs(f).nObs + 1;
                nObs = FeatureObs(f).nObs;
                FeatureObs(f).obsv(nObs).pid = pid;
                FeatureObs(f).obsv(nObs).uv = (uvd_cell{pid}(1:2, f))';
            end
            % this is to do with RGB-D cameras, depth value doesn't bother us
            [gns] = fn_GenGaussNoise(1, nc, SLAM_Params.sigma_d_real);
            uvd_cell{pid}(3,:) = uvd_cell{pid}(3,:) + gns;            
        end
        
        % Add noise to imu data
        [gns] = fn_GenGaussNoise(3, 1, SLAM_Params.sigma_bf_real);
        SLAM_Params.bf0 = SLAM_Params.bf0+gns;%fbiascef*sigmaf;
        [gns] = fn_GenGaussNoise(3, 1, SLAM_Params.sigma_bw_real);
        SLAM_Params.bw0 = SLAM_Params.bw0+gns;%fbiascef*sigmaw;
        
        [nr,nc] = size(imuData_cell{2}.samples(:, 2:4));
        for pid=2:nPoses
            %	imuData_cell{pid}.samples(:, 2:7) = imuData_cell{pid}.samples(:, 2:7) + ...
            %   repmat([sigmaw*ones(1,3),sigmaf*ones(1,3)], nIMUrate,1) .* randn(size(imuData_cell{pid}.samples(:, 2:7)))/fZnoisescale;
            [gns] = fn_GenGaussNoise(nr, nc, SLAM_Params.sigma_w_real);
            imuData_cell{pid}.samples(:, 2:4) = ...
                imuData_cell{pid}.samples(:, 2:4) + gns;
            [gns] = fn_GenGaussNoise(nr, nc, SLAM_Params.sigma_f_real);
            imuData_cell{pid}.samples(:, 5:7) = ...
                imuData_cell{pid}.samples(:, 5:7) + gns;
        end      
        save( [ Data_config.TEMP_DIR, 'SimuData.mat'], 'imuData_cell', 'uvd_cell', 'Ru_cell', 'Tu_cell', ...
            'SLAM_Params', 'vu');

    else

        load( [ Data_config.TEMP_DIR, 'SimuData.mat' ]);

        %	Zobs1(1:tid) = Zobs(1:tid); % Only substitute the uv+IMU part
        %   Zobs = Zobs1;
    end    
    
    for pid=2:nPoses%1e-2,1e-3+3*sigmaf+3*sigmaw
        [dp(:,pid), dv(:,pid), dphi(:,pid), Jd{pid}, Rd{pid}] = ...
            fn_DeltObsAccu( SLAM_Params.bf0, SLAM_Params.bw0, ...
            imuData_cell{pid}.samples, SLAM_Params.sigma_w_cov, SLAM_Params.sigma_f_cov); 
    end 
            