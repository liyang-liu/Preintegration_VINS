function [T, X] = fnTestIMU(t)
%% fnTestIMU: 
%   Check whether the IMU simulator is correct

initialtheta=[0 0 0]';
initialposition=[0 0 0]';
initialvelocity=[0 0 0]';%[-2 0 0]';%

x0=[initialtheta;initialposition;initialvelocity];
    
[T,X]=ode45(@fndXdt,t,x0); %[1 2]:1e-2: 

% drawPic;

end

