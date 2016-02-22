function [dRxalpha] = fn_Rx_dr(alpha)
%% dRx(alpha)/d(alpha)

dRxalpha = [0, 0, 0; 0, -sin(alpha), cos(alpha); 0, -cos(alpha), -sin(alpha)];
