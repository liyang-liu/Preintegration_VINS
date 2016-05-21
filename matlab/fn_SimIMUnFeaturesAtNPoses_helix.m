function [imuData_cell, uvd_cell, Ru_cell, Tu_cell, SLAM_Params, vu] = ...
    fn_SimIMUnFeaturesAtNPoses_helix(nPoses, nPts, nIMUrate, SLAM_Params)
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
if(nPoses < 1)
    imuData_cell = [];
	uvd_cell = {};
	Ru_cell = {};
	Tu_cell = {};
	return;
end

% Initialization of feature positions.
if(rem(nPts,2) == 0)
    nf1c = nPts/2; %10;% 5;%200; % number of features on one circle
    nc = 2; % number of circles
else
    nf1c = nPts;
    nc = 1;
end
radius = 10;%0.1;%0.5;%5e-2;%1;%3; % the dimension of the circles
np = nf1c * nc;
fp0 = zeros(3, np);
% uvd1 = fp1;
% fp2 = fp1;
% uvd2 = fp1;

% Parameters for the initial camera pose.
%cx = 0; cy = 0; cz = 0; 
cx0 = 240; cy0 = 320; f = 575;
% Feature distance to the initial camera pose.
fdcirc2camera = 50;%10;%1.0;%5;%3.0;
zcirc = 2*fdcirc2camera; 

% Ru_cell{1} = eye(3);
% Tu_cell{1} = zeros(3, 1);

% Calculate feature positions at the first camera pose
fangle = -[(2*pi/nf1c):(2*pi/nf1c):(2*pi)];
deltangle = pi/nf1c;
% uvd_cell{1} = zeros(3, np);

% Initialize features in the first IMU pose and then transfer.
for m=1:nc % on nc circles	
	pids = (1:nf1c) + ((m - 1) * nf1c);
	fp0(2, pids) = zcirc; % IMUy
	%uvd_cell{1}(3, pids) = zcirc;	
	fp0(1, pids) = SLAM_Params.Tu2c(1) + radius * cos(fangle);% IMUx
	fp0(3, pids) = SLAM_Params.Tu2c(3) + radius * sin(fangle);% IMUz
% 	uvd_cell{1}(1, pids) = f * fp1(1, pids) ./ fp1(3, pids) + cx0;
% 	uvd_cell{1}(2, pids) = f * fp1(2, pids) ./ fp1(3, pids) + cy0;
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
t = 0:(1.0/nIMUrate):(1-(1.0/nIMUrate));

for cid=1:nPoses
	uvd_cell{cid} = zeros(3, np);
	Ru_cell{cid} = fn_RFromABG(alpha, beta, gamma);        

    if(cid == 1)
        Tu_cell{cid} = zeros(3,1);
%         Ru_cell{cid} = eye(3);
    else
%         dstep = 0.2*(cid-1);
        Tu_cell{cid} = Tu_cell{cid-1} + tT;      
        % Relative displacement of Pose 2 in Pose 1.
%         T12 = Ru_cell{cid-1} * (tT - Tu_cell{cid-1});        
%         [x0, dataIMU] = fnSimuIMU(nsamplerate, dtang, T12(1), T12(2), T12(3), g, bConstV);
%         [x0, dataIMU] = fnSimuIMU(nsamplerate, dabg, tT(1), tT(2), tT(3), g, bConstV);

%         [dp, dv, dphi, J, R] = fnDeltObsAccu(dataIMU);
        
%         save( [Data_config.TEMP_DIR, 'Initial_state_for_Youbing1.mat'], 'x0')
%         save( [Data_config.TEMP_DIR, 'IMUdata_for_Youbing1.mat'], 'dataIMU');
%         x0(1:3, 1) = [alpha; beta; gamma];
%         x0(4:6, 1) =  Tu_cell{cid-1};
%         imuData_cell{cid}.samples = dataIMU;
%         imuData_cell{cid}.initstates = x0;
%         Tu_cell{cid} = Tu_cell{cid-1} + (Ru_cell{cid-1})'*tT;
%         Ru_cell{cid} = fnR5ABG(dabg(1), dabg(2), dabg(3))*Ru_cell{cid-1};
    end
%     Tu_cell{cid} = Tu_cell{cid-1} + tT;% Already in the initial coordinates 
    Rc = SLAM_Params.Ru2c * Ru_cell{cid};% Left multiply R to comply with the vectors to be multiplied on the right.
    Tc = Tu_cell{cid} + (Ru_cell{cid})' * SLAM_Params.Tu2c;
	% Features at the new position;
	fp2 = Rc * (fp0 - repmat(Tc, 1, np));
	uvd_cell{cid}(3, :) = fp2(3, :);
	uvd_cell{cid}(1, :) = f * fp2(1, :) ./ fp2(3, :) + cx0;
	uvd_cell{cid}(2, :) = f * fp2(2, :) ./ fp2(3, :) + cy0;
    
    %% Proceed to the next pose
    % Translation
    dy = 2*fdcirc2camera*(cos(gamma)-cos(gamma+dang));%6;%10;%5;%2;%1;%0.2;%dstep;
    dx = 2*fdcirc2camera*(-sin(gamma)+sin(gamma+dang));% 
    dz = spn * 5e-3 + (5e-3 ^ cid);%(1e1 ^ cid);%spn*2e-1 ^ (cid-1);%2e-1 ^ (cid-1);%0.0;%-0.1;%
    tT = [dx, dy, dz]';
    % Rotation
    
    if(bConstV == 0)
        dlta = spn * dang + 0.1^cid;%* 2  * (cid-1);%;
        dltb = dang + 0.1^cid;%;0;%
%             dabg = dlta *[1,1,1]';
        if(rem(cid,3) == 0)
            dabg = [dlta; 0; dang];
        elseif(rem(cid,3) == 1)
            dabg = [0; 0; dang];%dlta
        else%if(rem(cid,3) == 2)
            dabg = [0; dltb; dang];%dlta  
        end
    else
        dabg = zeros(3,1);
    end   
    
    
    %% Generate IMU data based on the motion process
    if(cid < nPoses)
        theta{cid} = [repmat(alpha, 1, nIMUrate)+dabg(1)*0.5*(1-cos(pi*t)); ...
                 repmat(beta, 1, nIMUrate)+dabg(2)*0.5*(1-cos(pi*t)); ...
                 repmat(gamma, 1, nIMUrate)+dabg(3)*0.5*(1-cos(pi*t));];
        dthetadt = 0.5*pi*dabg*sin(pi*t);%[0.5*pi*dalpha*sin(pi*t); 0.5*pi*dbeta*sin(pi*t); 0.5*pi*dgamma*sin(pi*t)];             
        p{cid} = [repmat(Tu_cell{cid}(1), 1, nIMUrate)+tT(1)*0.5*(1-cos(pi*t)); ...
             repmat(Tu_cell{cid}(2), 1, nIMUrate)+tT(2)*0.5*(1-cos(pi*t)); ...
             repmat(Tu_cell{cid}(3), 1, nIMUrate)+tT(3)*0.5*(1-cos(pi*t));];
        vu = [vu;(tT*0.5*pi*sin(pi*t))'];
        dvdt = 0.5*pi*pi*tT*cos(pi*t);         
        [dataIMU] = fn_SimuIMU(theta{cid}, dthetadt, dvdt, SLAM_Params.g0, SLAM_Params.bf0, SLAM_Params.bw0);  
        x0(1:3, 1) = [alpha; beta; gamma];
        x0(4:6, 1) =  Tu_cell{cid};
        x0(7:9, 1) =  zeros(3,1); 
        imuData_cell{cid+1}.samples = [t;dataIMU]';
        imuData_cell{cid+1}.initstates = x0;         
    end

    alpha = alpha + dabg(1);
    beta = beta + dabg(2);
    gamma = gamma + dabg(3);
   
	spn = -spn;
    
    
end

% if(PreIntegration_options.bPreInt == 0)
    % we can re-use Tu_cell{nPoses} and Ru_cell{nPoses}
    endT = Tu_cell{nPoses};
    endR = Ru_cell{nPoses};
    Tu_cell = [];
    Ru_cell = [];
    for cid=1:(nPoses-1)
        Tu_cell{cid} = p{cid};
        for(did=1:nIMUrate)
            Ru_cell{cid}{did} = fn_RFromABG(theta{cid}(1,did), ...
                theta{cid}(2,did), theta{cid}(3,did));
        end
    end 
    Tu_cell{nPoses} = endT;
    Ru_cell{nPoses} = endR;
    vu = [vu;zeros(1,3)];
% end
		
