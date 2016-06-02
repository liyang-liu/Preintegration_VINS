function [dRybeta] = fRy_dr(beta)
%% dRy(beta)/d(beta)

dRybeta = [-sin(beta), 0, -cos(beta); 0, 0, 0; cos(beta), 0, -sin(beta)];

