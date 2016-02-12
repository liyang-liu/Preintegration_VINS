function dEdphi = fndEdphi(phi, om)

dEdphi = [fndEdalpha(phi) * om, fndEdbeta(phi) * om, fndEdgamma(phi) * om];
