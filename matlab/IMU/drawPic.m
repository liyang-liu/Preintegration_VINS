% clc
% close all
[N,~]=size(T);
a=.3;
for i=1:N
   x0=X(i,:)';
theta=x0(1:3);
position=x0(4:6);

rotation=EulerAngle(theta);   
xd=rotation(:,1);
yd=rotation(:,2);
zd=rotation(:,3);
   

quiver3(position(1),position(2),position(3),a*xd(1),a*xd(2),a*xd(3),'r');
hold on

quiver3(position(1),position(2),position(3),a*yd(1),a*yd(2),a*yd(3),'g');
hold on

quiver3(position(1),position(2),position(3),a*zd(1),a*zd(2),a*zd(3),'b');
hold on
    
grid off
%axis([-3 3 -3 3 -2 30])
pause(1/30)
   
end

        