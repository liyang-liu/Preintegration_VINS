function [uvd] = funSimNextStep(fp, K, R, T)
%% Given feature positions (fp), camera model (K) and motion (R, T) in global frame, calculate the new observations of the feature sets.
%
% uvd: 3xn, [u;v;d], n feature images at the new camera pose
% fp: 3xn, [x;y;z], n points in global frame
% K: camera model
% R, T: next camera pose


% 1. feature points positions at the new camera pose.
fp1 = R * fp + T;
% 2. Feature images in the camera. uv = K * fp1;
f = K(1,1); cx = K(1,3); cy = K(2,3);
u = f * fp1(1, :) ./ fp1(3, :) + cx;
v = f * fp2(2, :) ./ fp1(3, :) + cy;
d = fp1(3, :);
uvd = [u; v; d];






