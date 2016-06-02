function y=Jac_koInv(RotationAngle)
% RotationAnle=Eulerangle,e.g, (roll,pitch,yaw)
% output is rotional rate matrix


x1=RotationAngle(1);
x2=RotationAngle(2);
y=(1/cos(x2))*[cos(x2) sin(x1)*sin(x2) cos(x1)*sin(x2);0 cos(x1)*cos(x2) -sin(x1)*cos(x2);0 sin(x1) cos(x1) ];
end