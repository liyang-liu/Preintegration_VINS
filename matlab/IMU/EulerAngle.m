function y = EulerAngle( X )

% y=R_c(c)*R_y(b)*R_x(a)
%     1        2      3

a=X(1);
b=X(2);
c=X(3);

y=[cos(c)*cos(b) -sin(c)*cos(a)+cos(c)*sin(b)*sin(a) sin(c)*sin(a)+cos(c)*cos(a)*sin(b);...
    sin(c)*cos(b)  cos(c)*cos(a)+sin(a)*sin(b)*sin(c)  -cos(c)*sin(a)+sin(b)*sin(c)*cos(a);...
    -sin(b) cos(b)*sin(a) cos(b)*cos(a)];




end

