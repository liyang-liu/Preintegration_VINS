function [dRdphi] = fn_dRdphi(phi, f)
%% % y=R_c(c)*R_y(b)*R_x(a)

alpha = phi(1); beta = phi(2); gamma = phi(3);
dRdphi = [  fn_Rx_dr(alpha) * fn_Ry(beta) * fn_Rz(gamma) * f, ...
            fn_Rx(alpha) * fn_Ry_dr(beta) * fn_Rz(gamma) * f, ...
            fn_Rx(alpha) * fn_Ry(beta) * fn_Rz_dr(gamma) * f ];
