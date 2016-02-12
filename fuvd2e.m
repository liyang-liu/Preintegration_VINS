function [e] = fuv2e(u, v, d, u0, v0, d0)
%% ei = (ui' - ui)^2 + (vi' - vi)^2

e = [(u - u0)'; (v - v0)'; (d - d0)'];
e = e(:);

