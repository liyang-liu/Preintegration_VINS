%% Jacobian 
% ei = I2(ui', vi') - I2(ui, vi);
u = f * x / z + x0; v= f * y / z + y0;
dudalpha = f / z * dxdalpha - f * x / z^2 * dzdalpha;
dvdalpha = f / z *
dei/dalpha = dI2fdu * du/dalpha + dI2/dv * dv/dalpha;
