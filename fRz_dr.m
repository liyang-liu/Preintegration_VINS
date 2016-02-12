function [dRzgamma] = fRz_dr(gamma)
%% dRz(gamma) 

dRzgamma = [-sin(gamma), cos(gamma), 0; -cos(gamma), -sin(gamma), 0; 0, 0, 0];

