function  [dEdg]=fndEdgamma(RotationAngle)

% y=(1/cos(x2))*[cos(x2) sin(x1)*sin(x2) cos(x1)*sin(x2);0 cos(x1)*cos(x2) -sin(x1)*cos(x2);0 sin(x1) cos(x1)];

dEdg = zeros(3);;
