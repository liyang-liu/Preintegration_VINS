function [imuData_cell, uvd_cell, Ru_cell, Tu_cell, vu] = ...
                            fn_SimIMUnFeaturesAtNPoses_helix( nPoses, nPts, nIMUrate, SLAM_Params )
    %fn_SimIMUnFeaturesAtNPoses_helix(nPoses, nPts, SLAM_Params,Ru2c, Tu2c, g,  bf, bw, nIMUrate, bPreInt)
    
    %% Simulate a group of features, n camera poses and IMU readings during each step
    %% Here I assume the features are situated on twocircles, each of which 
    %   is put at a certain distance to the camera pose 1. 
    %
    % uvd: feature images in [u; v; d] form; each pose corresponds to one cell
    % R, T: camera pose ground truth values; each pose corresponds to one cell

    global PreIntegration_options
    
    % Assume that the IMU and Camera are fixed on a rigid body, having no relative motion.
    % SLAM_Params.Ru2c = fRx(-pi/2);
    % SLAM_Params.Tu2c = [1.50; 0; 0]; %
    % SLAM_Params.g = [0; 0; -9.8];

    vu = [];
    if ( nPoses < 1 )
        imuData_cell = [];
        uvd_cell = {};
        Ru_cell = {};
        Tu_cell = {};
        return;
    end

    % Initialization of feature positions.
    if( rem(nPts, 2) == 0 )
        num_feat_per_ccl = nPts / 2; %10;% 5;%200; % number of features on one circle
        num_circle = 2; % number of circles
    else
        num_feat_per_ccl = nPts;
        num_circle = 1;
    end
    
    radius = 10;%0.1;%0.5;%5e-2;%1;%3; % the dimension of the circles
    num_feat = num_feat_per_ccl * num_circle;
    fp_w = zeros(3, num_feat); % feature in world coord frame

    % Parameters for the initial camera pose.
    %cx = 0; cy = 0; cz = 0; 
    cx0 = 240; cy0 = 320; f = 575;
    % Feature distance to the initial camera pose.
    fdcirc2camera = 50;%10;%1.0;%5;%3.0;
    zcirc = 2 * fdcirc2camera; 

    % Ru_cell{1} = eye(3);
    % Tu_cell{1} = zeros(3, 1);

    % Calculate feature positions at the first camera pose
    fangle = -[ (2*pi/num_feat_per_ccl) : (2*pi/num_feat_per_ccl) : (2*pi) ];
    deltangle = pi/num_feat_per_ccl;
    % uvd_cell{1} = zeros(3, num_feat);

    %% Initialize features in the first IMU pose and then transfer.
    for m = 1 : num_circle % on num_circle circles	
        fids = (1:num_feat_per_ccl) + ((m - 1) * num_feat_per_ccl);
        fp_w(2, fids) = zcirc; % IMUy
        %uvd_cell{1}(3, fids) = zcirc;	
        fp_w(1, fids) = SLAM_Params.Tu2c(1) + radius * cos(fangle);% IMUx
        fp_w(3, fids) = SLAM_Params.Tu2c(3) + radius * sin(fangle);% IMUz
        % 	uvd_cell{1}(1, fids) = f * fp1(1, fids) ./ fp1(3, fids) + cx0;
        % 	uvd_cell{1}(2, fids) = f * fp1(2, fids) ./ fp1(3, fids) + cy0;
        zcirc = zcirc - radius;%fdcirc2camera;
        fangle = fangle + deltangle; % rotate an angle
    end

    % Parameters for the following camera poses
    % dgamma = pi * (nPoses - 1) / (100);%0;%
    % dalpha = - pi/5;
    % dbeta = 0;%pi/20;



    % dstep = 2 * (zcirc - fdcirc2camera) * sin(dtang / 2);
    % dx = dstep * cos(dtang / 2);
    % dy = dstep * sin(dtang / 2);

    spn = 1;
    beta = 0;% pi/6;
    alpha = 0;% 
    gamma = 0;

    % IMU data generation
    dang = 0.01;%0.1;%0.02;%0.37;%0.0097;%0.0077;%0.0037;%0.0017;%0.037;%pi/10.5;%pi/5;%pi/20;%-pi/50;
    % nsamplerate = 1e2;%8e2;%6e2;%5e2;%1e3;% 
    bConstV = 0;
    t = 0 : (1.0/nIMUrate) : ( 1 - (1.0/nIMUrate) );

    for pid = 1 : nPoses
        uvd_cell{pid} = zeros(3, num_feat);
        Ru_cell{pid} = fn_RFromABG(alpha, beta, gamma);        

        if ( pid == 1 )
            Tu_cell{pid} = zeros(3,1);
            %	Ru_cell{pid} = eye(3);
        else
            %	dstep = 0.2*(pid-1);
            Tu_cell{ pid } = Tu_cell{pid - 1} + tT;      
            % Relative displacement of Pose 2 in Pose 1.
            %	T12 = Ru_cell{pid-1} * (tT - Tu_cell{pid-1});        
            %   [x0, dataIMU] = fnSimuIMU(nsamplerate, dtang, T12(1), T12(2), T12(3), g, bConstV);
            %   [x0, dataIMU] = fnSimuIMU(nsamplerate, dabg, tT(1), tT(2), tT(3), g, bConstV);

            %   [dp, dv, dphi, J, R] = fnDeltObsAccu(dataIMU);

            %   save( [Data_config.TEMP_DIR, 'Initial_state_for_Youbing1.mat'], 'x0')
            %   save( [Data_config.TEMP_DIR, 'IMUdata_for_Youbing1.mat'], 'dataIMU');
            %   x0(1:3, 1) = [alpha; beta; gamma];
            %   x0(4:6, 1) =  Tu_cell{pid-1};
            %   imuData_cell{pid}.samples = dataIMU;
            %   imuData_cell{pid}.initstates = x0;
            %   Tu_cell{pid} = Tu_cell{pid-1} + (Ru_cell{pid-1})'*tT;
            %   Ru_cell{pid} = fnR5ABG(dabg(1), dabg(2), dabg(3))*Ru_cell{pid-1};
        end
        
        %     Tu_cell{pid} = Tu_cell{pid-1} + tT;% Already in the initial coordinates 
        Rc = SLAM_Params.Ru2c * Ru_cell{pid};% Left multiply R to comply with the vectors to be multiplied on the right.
        Tc = Tu_cell{pid} + (Ru_cell{pid})' * SLAM_Params.Tu2c;
        
        %% Features at the new position;
        fp2 = Rc * (fp_w - repmat(Tc, 1, num_feat));
        uvd_cell{pid}(3, :) = fp2(3, :);
        uvd_cell{pid}(1, :) = f * fp2(1, :) ./ fp2(3, :) + cx0;
        uvd_cell{pid}(2, :) = f * fp2(2, :) ./ fp2(3, :) + cy0;

        %% Proceed to the next pose
        % Translation
        dy = 2 * fdcirc2camera * ( cos( gamma ) - cos( gamma + dang ) );%6;%10;%5;%2;%1;%0.2;%dstep;
        dx = 2 * fdcirc2camera * ( -sin( gamma ) + sin( gamma + dang ) );% 
        dz = spn * 5e-3 + (5e-3 ^ pid);%(1e1 ^ pid);%spn*2e-1 ^ (pid-1);%2e-1 ^ (pid-1);%0.0;%-0.1;%
        tT = [dx, dy, dz]';
        % Rotation

        if(bConstV == 0)
            dlta = spn * dang + 0.1^pid;%* 2  * (pid-1);%;
            dltb = dang + 0.1^pid;%;0;%
            %	dabg = dlta *[1,1,1]';
            if(rem(pid,3) == 0)
                dabg = [dlta; 0; dang];
            elseif(rem(pid,3) == 1)
                dabg = [0; 0; dang];%dlta
            else%if(rem(pid,3) == 2)
                dabg = [0; dltb; dang];%dlta  
            end
        else
            dabg = zeros(3,1);
        end   


        %% Generate IMU data based on the motion process
        if(pid < nPoses) 
            theta{pid} = [ repmat(alpha, 1, nIMUrate) + dabg(1) * 0.5 * (1 - cos(pi * t)); ...
                             repmat(beta, 1, nIMUrate) + dabg(2) * 0.5 * (1 - cos(pi * t)); ...
                             repmat(gamma, 1, nIMUrate) + dabg(3) * 0.5 * (1 - cos(pi * t));];
                 
            dthetadt = 0.5 * pi * dabg * sin(pi * t);%[0.5*pi*dalpha*sin(pi*t); 0.5*pi*dbeta*sin(pi*t); 0.5*pi*dgamma*sin(pi*t)];
            
            p{pid} = [  repmat( Tu_cell{pid}(1), 1, nIMUrate ) + tT(1) * 0.5 * (1 - cos(pi * t)); ...
                        repmat( Tu_cell{pid}(2), 1, nIMUrate ) + tT(2) * 0.5 * (1 - cos(pi * t)); ...
                        repmat( Tu_cell{pid}(3), 1, nIMUrate ) + tT(3) * 0.5 * (1 - cos(pi * t)); ];
                    
            vu = [ vu; (tT * 0.5 * pi * sin(pi*t) )' ];
            
            dvdt = 0.5 * pi * pi * tT * cos(pi*t);         
            
            [dataIMU] = fn_SimuIMU( theta{pid}, dthetadt, dvdt, SLAM_Params.g0, SLAM_Params.bf0, SLAM_Params.bw0 );  
            
            x0(1:3, 1) = [alpha; beta; gamma];
            x0(4:6, 1) =  Tu_cell{pid};
            x0(7:9, 1) =  zeros(3,1); 
            imuData_cell{ pid + 1 }.samples = [ t; dataIMU ]';
            imuData_cell{ pid + 1 }.initstates = x0;         
        end

        alpha = alpha + dabg(1);
        beta = beta + dabg(2);
        gamma = gamma + dabg(3);

        spn = -spn;


    end

    % if(PreIntegration_options.bPreInt == 0)
        % we can re-use Tu_cell{nPoses} and Ru_cell{nPoses}
        endT = Tu_cell{ nPoses };
        endR = Ru_cell{ nPoses };
        Tu_cell = [];
        Ru_cell = [];
        
        for pid = 1 : (nPoses-1)
            Tu_cell{pid} = p{pid};
            for did = 1 : nIMUrate
                Ru_cell{pid}{did} = fn_RFromAngVec( theta{pid}(:, did) );
            end
        end 
        Tu_cell{ nPoses } = endT;
        Ru_cell{ nPoses } = endR;
        vu = [ vu; zeros(1,3) ];
    % end

