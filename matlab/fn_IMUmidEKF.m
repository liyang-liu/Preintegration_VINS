function [pkf, vkf, phikf] = fnIMUmidEKF(imuData_cell, Ru_cell, Tu_cell, ...
                            vu, pidst, piden, SLAM_Params )
            fnIMUmidEKF(imuData_cell, Ru_cell, Tu_cell, ...
            vu, pidst, piden, bPreInt, bf, bw, gn)

%% Standard EKF-based IMU state prediction.

% dataDir = '../IMUsimulator_continuous/';
% load([dataDir 'Initial_state_for_Youbing.mat']); % x0 = [Euler angle, position, velocity]
% load([dataDir 'IMUdata_for_Youbing.mat']); % dataIMU: time step, rotation rate, acceleration

    global PreIntegration_options

    if(piden <=pidst)
        error('piden should be bigger than pidst');
        return;
    end

    nts = size(imuData_cell{2}.samples, 1);
    dt = imuData_cell{2}.samples(2, 1) - imuData_cell{2}.samples(1, 1);
    %% Initialization
    if(pidst == 1)
       p0 = zeros(3,1);
       v0 = zeros(3,1);
       phi0 = zeros(3,1);
    else
        %     if(PreIntegration_options.bPreInt == 1)
        %         v0 = vu(pidst,1);    
        %         p0 = Tu_cell{pidst};    
        %         phi0 = Tu_cell{pidst};       
        %     else
            v0 = vu(nts*(pidst-1)+1,1);    
            p0 = Tu_cell{pidst}(:,1); 
            [a,b,g] = fnABG5R(Ru_cell{pidst}{1});
            phi0 = [a;b;g];
    %     end
    end

    % % Bias
    % SLAM_Params.bf = [0, 0, 0]';
    % bo = [0, 0, 0]';
    % % Weight
    % SLAM_Params.g = [0, 0, -9.8]';
    % Timesteps

    % State vectors.
    pkf = zeros(3, piden-pidst+1);
    vkf = zeros(3, piden-pidst+1);
    phikf = zeros(3, piden-pidst+1);
    % p = zeros(3, nts);
    % v = zeros(3, nts);
    % phi = zeros(3, nts);
    phi = phi0;
    p = p0;
    v = v0;

    pkf(:,1) = p;
    vkf(:,1) = v;
    phikf(:,1) = phi;
    % % Initial state uncertainty
    % P = eye(15);
    % % IMU noise covariance matrix
    % Q = eye(6);

    %% Iterative prediction of inertial values and covariance
    for(pid=pidst:piden-1)
        k = 1; 
        while(k < nts)	
            omega0 = (imuData_cell{pid+1}.samples(k, 2:4))';
            fb0 = (imuData_cell{pid+1}.samples(k, 5:7))';
            [p, v, phi] = fnMidPredict(p, v, phi, dt, fb0, omega0, SLAM_Params.bf, SLAM_Params.bw, SLAM_Params.g);
        % 	P = fnStdCovPredict(Q, P, omega0, phi(:, k), dt, fb0, SLAM_Params.bf, SLAM_Params.bw)
            k = k + 1;
        % 	disp([sprintf('T%d: [%f, %f, %f], [%f, %f, %f], [%f, %f, %f]', k, p(1, k), p(2, k), p(3, k), v(1, k), v(2, k), v(3, k), phi(1, k), phi(2, k), phi(3, k))]);	
        end
        pkf(:,pid-pidst+2) = p;
        vkf(:,pid-pidst+2) = v;
        phikf(:,pid-pidst+2) = phi;
    end

    % xe = [p;v;phi];
    % %% Graphical display
    % figure; plot(dataIMU(:, 1)', p'); xlabel('Time (s)'); ylabel('Position (m)'); legend('x','y','z'); title('Translation Results: ');
    % figure; plot(dataIMU(:, 1)', v'); xlabel('Time (s)'); ylabel('Translational Velocity (m/s)'); legend('vx','vy','vz'); title('Translational Velocity Results: ');
    % figure; plot(dataIMU(:, 1)', phi'); xlabel('Time (s)'); ylabel('Rotational Velocity (rad/s)'); legend('Ox','Oy','Oz'); title('Rotation Results: ');