function [xg] = fnCal9RelativePoses(xg, nPoses, tv)
%% Calculate new relative pose to tv(:,1), which is composed of (A,B,G,T1)'
%    tv: 6xn

    R10 = fnR5ABG(tv(1, 1), tv(2, 1), tv(3, 1));
    T10 = tv(4:6, 1);
    idend = 0;
    for(pid=2:nPoses)
        Ri0 = fnR5ABG(tv(1, pid), tv(2, pid), tv(3, pid));
        Ti0 = tv(4:6, pid); 
        Ri1 = Ri0*R10';
        [Ai1, Bi1, Gi1] = fnABG5R(Ri1);
        Ti1 = R10'*(Ti0 - T10);
        idstart = idend + 1; idend = idend + 6;
        xg(idstart:idend) = [Ai1;Bi1;Gi1;Ti1];
    end