function dEdphi = fn_dEdphi(phi, om)

dEdphi = [fn_dEdalpha(phi) * om, fn_dEdbeta(phi) * om, fn_dEdgamma(phi) * om];
