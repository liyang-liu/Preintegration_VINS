function [e] = fnUVDErr(K, x, Zobs, nPoses, N)

f = K(1,1); cx0 = K(1,3); cy0 = K(2,3);
p3d0 = reshape(x(((nPoses-1)*6+1):end, 1), 3, []);
e = Zobs;
%N = size(p3d0, 2);

% Reprojection at pose 1
u1 = f * p3d0(1, :) ./ p3d0(3, :) + repmat(cx0, 1, N);
v1 = f * p3d0(2, :) ./ p3d0(3, :) + repmat(cy0, 1, N);
d1 = p3d0(3, :);
et = [u1;v1;d1];
et = et(:);
e(1:(N*3), 1) = et - e(1:(N*3), 1);
% Reprojection at pose 2
for pid=2:nPoses
    alpha = x(1+(pid-2)*6); beta = x(2+(pid-2)*6); gamma = x(3+(pid-2)*6); 
    T1 = x((4+(pid-2)*6):((pid-1)*6), 1);
    R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
    p3d1 = R1 * p3d0 + repmat(T1, 1, N);
    u1 = f * p3d1(1, :) ./ p3d1(3, :) + repmat(cx0, 1, N);
    v1 = f * p3d1(2, :) ./ p3d1(3, :) + repmat(cy0, 1, N);
    d1 = p3d1(3, :);
    et = [u1;v1;d1];
    et = et(:);
    e((N*3*(pid-1)+1):(N*3*pid), 1) = et - e((N*3*(pid-1)+1):(N*3*pid), 1);
end