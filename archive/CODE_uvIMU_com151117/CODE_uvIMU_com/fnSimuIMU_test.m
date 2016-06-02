function [data5IMU] = fnSimuIMU_test(t)%angles, p, 

%% Simuate a 1 second process, data rate = 200 Hz.
    %g = [0; 0; -9.8];
    %nsamplerate = 7e2;%200;
%     t = (1.0/nsamplerate):(1.0/nsamplerate):1;
%     t = 0:(1.0/nsamplerate):1-(1.0/nsamplerate);
    %nPoses  = 4;
    %nsamplerate = nsamplerate + 1;
    %% Rotation
    % w = constant
%     dalpha = dabg(1); dbeta = dabg(2); dgamma = dabg(3);
%     theta = [0.5*dalpha*(1-cos(pi*t)); 0.5*dbeta*(1-cos(pi*t)); 0.5*dgamma*(1-cos(pi*t))];%+-pi/7*t
%     dthetadt = [0.5*pi*dalpha*sin(pi*t); 0.5*pi*dbeta*sin(pi*t); 0.5*pi*dgamma*sin(pi*t)];
%     dthetadt = repmat(dthetadt, 1, nsamplerate);
%     %% Position
%     if(bConstV == 1)        % dvdt = 0
%         p = [dx * t; dy * t; dz * t];% for accelerating .* t
%         initialvelocity = [dx;dy;dz];
%         dvdt = repmat([0;0;0], 1, nsamplerate);
%         initialtheta = zeros(3,1);
%         initialposition = zeros(3,1);
%     else        % dvdt != 0
%         % Here we suppose the velocity change according to sine curve from t=[0,1]:
%         % v = m*sin(pi*t) ==> p = m*(1-cos(pi*t))/pi;   m = d * pi/2;
%         p = 0.5*[dx; dy; dz]*(1-cos(pi*t));
%         dvdt = 0.5*pi*pi*[dx; dy; dz]*cos(pi*t);
%         initialvelocity = [0; 0; 0];
% %         p = [dx * t; dy * t; dz * (t .* t)];% for accelerating         
% %         dvdt = repmat([0;0;0], 1, nsamplerate);    
% %         dvdt(3,:) = 2 * dz;% Acceleration   
%      
%     end
%% Given theta(t), dthetadt(t) and dvdt, generate IMU output
% Based on:
% 1. dthetadt = J_koinv(theta)*w, where w corresponds to the angular velocity output by IMU
% 2. dpdt = v; dvdt = (R(theta))'*B+g, where B is the acceleration part output by IMU, R is the 
%    rotation matrix.
% So, w = Jko(theta)*dthetadt;
%     B = R(theta)*(dp2dt2-g);
load('SimuPoses.mat');

cid = floor(t);
t = t - cid;

% if(bPreInt == 0)
    if(cid == 1)
        alpha = 0; beta = 0; gamma = 0;    
    else
        [alpha, beta, gamma] = fnABG5R(Ru_cell{cid}{1});
    end
    if(cid < (nPoses - 1))
        [alpha1, beta1, gamma1] = fnABG5R(Ru_cell{cid+1}{1});
    else
        [alpha1, beta1, gamma1] = fnABG5R(Ru_cell{cid+1});
    end
    tT = Tu_cell{cid+1}(:,1) - Tu_cell{cid}(:,1);
% else
%     if(cid == 1)
%         alpha = 0; beta = 0; gamma = 0;    
%     else
%         [alpha, beta, gamma] = fnABG5R(Ru_cell{cid});
%     end
%     [alpha1, beta1, gamma1] = fnABG5R(Ru_cell{cid+1});
%     tT = Tu_cell{cid+1} - Tu_cell{cid};
% end

dabg = [alpha1-alpha; beta1-beta; gamma1-gamma];
theta = [alpha; beta; gamma] + dabg*0.5*(1-cos(pi*t));
% repmat([alpha; beta; gamma], 1, nIMUrate) + dabg*0.5*(1-cos(pi*t));
%     [repmat(alpha, 1, nIMUrate)+dabg(1)*0.5*(1-cos(pi*t)); ...
%              repmat(beta, 1, nIMUrate)+dabg(2)*0.5*(1-cos(pi*t)); ...
%              repmat(gamma, 1, nIMUrate)+dabg(3)*0.5*(1-cos(pi*t));];
dthetadt = 0.5*pi*dabg*sin(pi*t);  

dvdt = 0.5*pi*pi*tT*cos(pi*t);

% angular velocity part
% Generate angular velocity
nt = size(theta, 2);
w = zeros(3, nt);
B = w;
for idx=1:nt    
    w(:,idx) = bw0 + Jac_ko(theta(:,idx)) * dthetadt(:,idx);         %Jac_RotInv_xyz angular velocity 
    % acceleration part
    Ri2b = fnR5ABG(theta(1,idx), theta(2,idx), theta(3,idx)); % = (EulerAngle(theta(:,idx)))'
    % Generate acceleration part
    B(:,idx) = bf0 + Ri2b * (dvdt(:,idx) - g0);
end

% x0=[zeros(6,1);initialvelocity];%initialtheta;initialposition;

data5IMU=[w;B];% [t;w;B]';


