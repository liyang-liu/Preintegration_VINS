function [gns] = fnGenGaussNoise(nr, nc, fsigma)

fbnd = 3*fsigma;%
gns = repmat(fsigma, nr, nc) .* randn(nr,nc);%size(fsigma, 1)*
gns(gns > fbnd) = fbnd;
gns(gns < -fbnd) = -fbnd;