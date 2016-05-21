function [] = fn_TestIMU_cmp(nPoses, imuData_cell, Ru_cell, Tu_cell, vu, SLAM_Params)
        %   fnTestIMU_cmp(nPoses, imuData_cell, Ru_cell, Tu_cell, ...
        %                    vu, bPreInt, bf0, bw0, g0)

    global PreIntegration_options Data_config
    
        figure; hold all; 
        title('ODE method, StdEKF and midEKF V.S. Ground Truth (Translation)');
        
%        fnShowIMUgtPoses(Ru_cell, Tu_cell, nPoses, nIMUrate, bPreInt, 'Ground Truth Values');

       save([ Data_config.TEMP_DIR, 'SimuPoses.mat'],'Ru_cell','Tu_cell', 'g0', 'bf0', 'bw0', 'bPreInt', 'nPoses');  
       
       pidst = 1; piden = nPoses - 1;
       % ode-based integration method
       
       [to, xo] = fn_TestIMU([pidst piden]);
%        if(bPreInt == 1)
%            Err_ode = Tu_cell{piden} - (xo(end, 4:6))'
%        else
           Err_ode = Tu_cell{piden}(:,1) - (xo(end, 4:6))'
%        end
       % Traditional EKF method
       [pkf, vkf, phikf] = fn_IMUstdEKF(imuData_cell, Ru_cell, Tu_cell, ...
                            vu, pidst, piden, bPreInt, bf0, bw0, g0);
%        if(bPreInt == 1)
%            Err_ekf = Tu_cell{piden} - pkf(:, end)
%        else
           Err_ekf = Tu_cell{piden}(:,1) - pkf(:, end)
%        end
       % Average-based EKF method
       [mpkf, mvkf, mphikf] = fn_IMUmidEKF(imuData_cell, Ru_cell, Tu_cell, ...
                            vu, pidst, piden, bPreInt, bf0, bw0, g0);
%        if(bPreInt == 1)
%            Err_mekf = Tu_cell{piden} - mpkf(:, end)
%        else
           Err_mekf = Tu_cell{piden}(:,1) - mpkf(:, end)
%        end
       dekf = mpkf - pkf
       gT = zeros(3, nPoses);
       for(i=1:nPoses)
%            if(bPreInt == 1)
%                gT(:,i) = Tu_cell{i};           
%            else
               gT(:,i) = Tu_cell{i}(:,1);
%            end
       end
       for(i=1:3)
           subplot(3,1,i);hold on;
           plot(1:nPoses, gT(i, :), 'bp');
           plot(pidst:piden, pkf(i,:), 'r*');
           plot(pidst:piden, mpkf(i,:), 'go');
           plot(to, xo(:, i+3), 'm+');
       end