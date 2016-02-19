function [Alpha,Beta,Gamma] = fABGfrmR(R)
%% Get abg from R
% Input: 
%%            R: rotation matrix, assumed in coordinate rotation form!!!!!!
% Output:
%%        Alpha,Beta,Gamma: correspond to Roll, Pitch, Yaw respectively!!!! 


 Beta=atan2(-(R(1,3)),sqrt(R(1,1)^2+R(1,2)^2));
 if (abs(cos(Beta)) <= eps)%cos(Beta)==0)
     Alpha = atan2(R(1,2),R(2,2));
     Beta = pi/2;
     Gamma = 0;
 else
     Alpha = atan2(R(2,3)/cos(Beta),R(3,3)/cos(Beta));
     Gamma = atan2(R(1,2)/cos(Beta),R(1,1)/cos(Beta));
 end


end
