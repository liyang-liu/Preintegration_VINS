function [Rzgamma] = fn_Rz(gamma)
%% Rz(gamma) 

Rzgamma = [  cos(gamma), sin(gamma),  0;
            -sin(gamma), cos(gamma),  0;
                      0,          0,  1 ];

