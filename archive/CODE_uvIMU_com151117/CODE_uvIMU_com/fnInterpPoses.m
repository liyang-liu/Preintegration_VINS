function [p, v, a, pp] = fnInterpPoses(nPoses, arPts, t)

pp = spline(1:nPoses, arPts);
[~, pxcoef, nln, nOd, ~] = unmkpp(pp);
% px
for(i=1:(nPoses-1))
    p{i} = 0;
    for(j=1:nOd)
        p{i} = p{i} + pxcoef(i,j)*t.^(nOd-j);% t= x-b(i)=dt
    end
end
% vx
for(i=1:(nPoses-1))
    v{i} = 0;
    for(j=1:nOd-1)
        v{i} = v{i} + (nOd-j)*pxcoef(i,j)*t.^(nOd-j-1);% t= x-b(i)=dt
    end
end
% ax
for(i=1:(nPoses-1))
    a{i} = 0;
    for(j=1:nOd-2)
        a{i} = a{i} + (nOd-j)*(nOd-j-1)*pxcoef(i,j)*t.^(nOd-j-2);% t= x-b(i)=dt
    end
end