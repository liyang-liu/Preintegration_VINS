function [e] = fnPredictUVDErr(K, x, u0, v0, d0, u, v, d)

f = K(1,1); cx0 = K(1,3); cy0 = K(2,3);

alpha = x(1); beta = x(2); gamma = x(3); T1 = x(4:6, 1);
p3d0 = reshape(x(7:end, 1), 3, []);
N = size(p3d0, 2);

% Reprojection at pose 1
u1 = f * p3d0(1, :) ./ p3d0(3, :) + repmat(cx0, 1, N);
v1 = f * p3d0(2, :) ./ p3d0(3, :) + repmat(cy0, 1, N);
d1 = p3d0(3, :);
[e] = fuvd2e(u1', v1', d1', u0, v0, d0);
% Reprojection at pose 2
R1 = fRx(alpha) * fRy (beta) * fRz(gamma);
p3d1 = R1 * p3d0 + repmat(T1, 1, N);
u1 = f * p3d1(1, :) ./ p3d1(3, :) + repmat(cx0, 1, N);
v1 = f * p3d1(2, :) ./ p3d1(3, :) + repmat(cy0, 1, N);
d1 = p3d1(3, :);
[e1] = fuvd2e(u1', v1', d1', u, v, d);
e = [e; e1];
