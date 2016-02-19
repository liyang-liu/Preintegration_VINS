function [de] = fuv2e_dr(u, v, u0, v0)
%% ei = (ui' - ui')^2 + (vi' - vi)^2

de = [(u - u0), (v - v0)];

end