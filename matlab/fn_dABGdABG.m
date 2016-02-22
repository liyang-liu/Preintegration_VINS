% ===============================================================================================
% Linear SLAM: A Linear Solution to the Pose Feature and Pose Graph SLAM based on Submap Joining 
% Version: 1.0
% ===============================================================================================
% 
% Copyright (C) 2013 Liang Zhao, Shoudong Huang and Gamini Dissanayake
% University of Technology, Sydney, Australia
% 
% Authors:  Liang Zhao         -- Liang.Zhao-1@uts.edu.au 
%           Shoudong Huang     -- Shoudong.Huang@uts.edu.au
%           Gamini Dissanayake -- Gamini.Dissanayake@uts.edu.au
% 
%           Centre for Autonomous Systems
%           Faculty of Engineering and Information Technology
%           University of Technology, Sydney
%           NSW 2007, Australia
% 
% License
% 
% Linear SLAM by Liang Zhao, Shoudong Huang, Gamini Dissanayake is licensed under a 
% Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.
% 
% Please contact Liang Zhao {Liang.Zhao-1@uts.edu.au} if you have any questions/comments about the code.
% 
%%
function dABGdABG = fndABGdABG(dRdABG,R)

F1 = R(1,2)/R(1,1);
F3 = R(2,3)/R(3,3);
F5 = R(1,1)^2+R(1,2)^2;
F4 = sqrt(F5);
F2 = -R(1,3)/F4;

% AAAAA = cos(atan(F2));
% if abs(AAAAA)<0.1;
%     AAAAA
% end;

dAdF1 = Func_datan(F1);
dBdF2 = Func_datan(F2);
dGdF3 = Func_datan(F3);

dF1d = ( dRdABG(1,2) * R(1,1) - R(1,2) * dRdABG(1,1) ) / R(1,1)^2;
dF3d = ( dRdABG(2,3) * R(3,3) - R(2,3) * dRdABG(3,3) ) / R(3,3)^2;

dF4dF5 = Func_dDikdD2( F5 );
dF5d = 2 * R(1,1) * dRdABG(1,1) + 2 * R(1,2) * dRdABG(1,2);
dF4d = dF4dF5 * dF5d;

dF2d = ( -dRdABG(1,3) * F4 + R(1,3) * dF4d ) / F5;

dABGdABG = [ dGdF3 * dF3d; dBdF2 * dF2d; dAdF1 * dF1d ];
