function [dRdphi] = fndRdphi(phi, f)
%% % y=R_c(c)*R_y(b)*R_x(a)

alpha = phi(1); beta = phi(2); gamma = phi(3);
dRdphi = [-fRz(-gamma)*fRy(-beta)*fRx_dr(-alpha) * f, -fRz(-gamma)*fRy_dr(-beta)*fRx(-alpha) * f, -fRz_dr(-gamma)*fRy(-beta)*fRx(-alpha) * f];
