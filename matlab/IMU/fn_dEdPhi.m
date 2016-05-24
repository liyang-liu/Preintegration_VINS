function dEdphi = fn_dEdphi(phi, om)

dEdphi = [fn_dEdAlpha(phi) * om, fn_dEdBeta(phi) * om, fn_dEdGamma(phi) * om];
