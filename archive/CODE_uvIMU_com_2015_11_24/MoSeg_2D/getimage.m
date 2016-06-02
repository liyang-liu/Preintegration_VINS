function [img, imask, msk2p] = getimage(x1, x2)
%% restore uv to image
% x1: 2xn points [u;v]
% x2: 2xn points 
np = size(x1, 2);
dx = x2 - x1;
dx = complex(dx(1,:), dx(2,:));
img = zeros(ceil(max(x1(2, :))), ceil(max(x1(1, :))));
imask = img;
msk2p = [];
for(i = 1:np)
    c = round(x1(1, i));
    r = round(x1(2, i));
    img(r, c) = dx(i);
    imask(r, c) = 1;
    msk2p(r,c) = i;
end
    