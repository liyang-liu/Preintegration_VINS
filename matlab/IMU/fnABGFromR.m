function [Alpha,Beta,Gamma] = fnABGFromR(K)
%% Get abg from R
% Input: 
%%            R: rotation matrix, assumed in coordinate rotation form!!!!!!
% Output:
%%        Alpha,Beta,Gamma: correspond to Roll, Pitch, Yaw respectively!!!! 


 Beta=atan2(-(K(1,3)),sqrt(K(1,1)^2+K(1,2)^2));
 if (abs(cos(Beta)) <= eps)%cos(Beta)==0)
     Alpha = atan2(K(1,2),K(2,2));
     Beta = pi/2;
     Gamma = 0;
 else
     Alpha = atan2(K(2,3)/cos(Beta),K(3,3)/cos(Beta));
     Gamma = atan2(K(1,2)/cos(Beta),K(1,1)/cos(Beta));
 end


end
