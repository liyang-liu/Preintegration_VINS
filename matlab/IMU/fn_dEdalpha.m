function  [dEda]=fn_dEdalpha(RotationAngle)

% y=(1/cos(x2))*[cos(x2) sin(x1)*sin(x2) cos(x1)*sin(x2);0 cos(x1)*cos(x2) -sin(x1)*cos(x2);0 sin(x1) cos(x1) ];

x1=RotationAngle(1);
x2=RotationAngle(2);
dEda=(1/cos(x2))*[0 cos(x1)*sin(x2) -sin(x1)*sin(x2);0 -sin(x1)*cos(x2) -cos(x1)*cos(x2);0 cos(x1) -sin(x1) ];
