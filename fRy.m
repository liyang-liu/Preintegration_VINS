function [Rybeta] = fRy(beta)
%% Ry(beta) 

Rybeta = [cos(beta), 0, -sin(beta); 0, 1, 0; sin(beta), 0, cos(beta)];

