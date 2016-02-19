function [dRdphi] = fndRdphi(phi, f)
%% % y=R_c(c)*R_y(b)*R_x(a)

alpha = phi(1); beta = phi(2); gamma = phi(3);
dRdphi = [fRx_dr(alpha)*fRy(beta)*fRz(gamma) * f, fRx(alpha)*fRy_dr(beta)*fRz(gamma) * f, fRx(alpha)*fRy(beta)*fRz_dr(gamma) * f];
