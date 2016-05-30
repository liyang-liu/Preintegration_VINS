function [imuData_cell, uvd_cell, noisefree_imuData_cell, noisefree_uvd_cell, Ru_cell, Tu_cell, FeatureObs, vu, SLAM_Params] = fn_GeneratePostFeatData(  nFrames, nPts, nIMUrate, SLAM_Params )

    global PreIntegration_options Data_config
    
    %% Use previous observation data or not
    if(PreIntegration_options.bUsePriorZ == 0)   
    
        arPts = [0,0,0; 1,0.5,2;0.5,2,2.6;1.8,2.4,3.5];
        
        [ noisefree_imuData_cell, noisefree_uvd_cell, Ru_cell, Tu_cell, vu] = ...
            fn_SimIMUnFeaturesAtNPoses_helix( nFrames, nPts, nIMUrate, SLAM_Params );% line   


        if(PreIntegration_options.bTestIMU == 1)
            fn_TestIMU_cmp(nFrames, noisefree_imuData_cell, Ru_cell, Tu_cell, ...
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
        
        fids = num2cell( 1:nPts );
        [FeatureObs(:).fid] = fids{:};        
    
        uvd_cell = noisefree_uvd_cell;
        [~,nc] = size(uvd_cell{1});
        for frm = 1:nFrames
            %	[gns] = fn_GenGaussNoise(1, nc, [1;1;0.01]);
            %   uvd_cell{frm} = uvd_cell{frm} + gns;  
            if PreIntegration_options.bUseNoisyData == 1
                [gns] = fn_GenGaussNoise(2, nc, SLAM_Params.sigma_uov_real);
                uvd_cell{frm}(1:2,:) = uvd_cell{frm}(1:2,:) + gns;
            end
            
            for ( f=1:nPts )
                FeatureObs(f).nObs = FeatureObs(f).nObs + 1;
                nObs = FeatureObs(f).nObs;
                FeatureObs(f).obsv(nObs).pid = frm;
                FeatureObs(f).obsv(nObs).uv = (uvd_cell{frm}(1:2, f))';
            end
            % this is to do with RGB-D cameras, depth value doesn't bother us
            if PreIntegration_options.bUseNoisyData == 1
                [gns] = fn_GenGaussNoise(1, nc, SLAM_Params.sigma_d_real);
                uvd_cell{frm}(3,:) = uvd_cell{frm}(3,:) + gns;            
            end
        end
        
        if PreIntegration_options.bUseNoisyData == 1        
            % Add noise to imu data        
            [gns] = fn_GenGaussNoise(3, 1, SLAM_Params.sigma_bf_real);
            SLAM_Params.bf0 = SLAM_Params.bf0 + gns; %fbiascef*sigmaf;
            [gns] = fn_GenGaussNoise(3, 1, SLAM_Params.sigma_bw_real);
            SLAM_Params.bw0 = SLAM_Params.bw0 + gns; %fbiascef*sigmaw;
        end
        
        imuData_cell = noisefree_imuData_cell;
        [nr,nc] = size(imuData_cell{2}.samples(:, 2:4));
        for frm=2:nFrames
            if PreIntegration_options.bUseNoisyData == 1
                [gns] = fn_GenGaussNoise(nr, nc, SLAM_Params.sigma_w_real);
                imuData_cell{frm}.samples(:, 2:4) = ...
                    imuData_cell{frm}.samples(:, 2:4) + gns;
                [gns] = fn_GenGaussNoise(nr, nc, SLAM_Params.sigma_f_real);
                imuData_cell{frm}.samples(:, 5:7) = ...
                    imuData_cell{frm}.samples(:, 5:7) + gns;
            end
        end      
        save( [ Data_config.TEMP_DIR, 'SimuData.mat'], 'imuData_cell', 'uvd_cell', 'Ru_cell', 'Tu_cell', ...
            'SLAM_Params', 'vu');

    else

        load( [ Data_config.TEMP_DIR, 'SimuData.mat' ]);

        %	Zobs1(1:tid) = Zobs(1:tid); % Only substitute the uv+IMU part
        %   Zobs = Zobs1;
    end    
    
    for frm = 2 : nFrames %1e-2,1e-3+3*sigmaf+3*sigmaw
        [dp(:,frm), dv(:,frm), dphi(:,frm), Jd{frm}, Rd{frm}] = ...
                        fn_DeltObsAccu( SLAM_Params.bf0, SLAM_Params.bw0, ...
                            imuData_cell{frm}.samples, SLAM_Params.sigma_w_cov, SLAM_Params.sigma_f_cov ); 
    end 
            