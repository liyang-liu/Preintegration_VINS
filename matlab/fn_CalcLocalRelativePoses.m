function [ Xg_obj, xg_col ] = fn_CalcLocalRelativePoses(Xg_obj, xg_col, nPoses, tv)
%% Calculate new relative pose to tv(:,1), which is composed of (A,B,G,T1)'
%    tv: 6xn

    R10 = fn_RFromABG(tv(1, 1), tv(2, 1), tv(3, 1));
    T10 = tv(4:6, 1);
    idend = 0;
    for(pid=2:nPoses)
        Ri0 = fn_RFromABG(tv(1, pid), tv(2, pid), tv(3, pid));
        Ti0 = tv(4:6, pid); 
        Ri1 = Ri0 * R10';
        [Ai1, Bi1, Gi1] = fn_ABGFromR(Ri1);
        Ti1 = R10'*(Ti0 - T10);
        %idstart = idend + 1; idend = idend + 6;
        %Xg_obj(idstart:idend) = [Ai1;Bi1;Gi1;Ti1];
        Xg_obj.pose(pid-1).ang.val = [Ai1; Bi1; Gi1];
        Xg_obj.pose(pid-1).ang.col = (1:3) + xg_col;    xg_col = xg_col + 3;
        Xg_obj.pose(pid-1).trans.xyz = Ti1;
        Xg_obj.pose(pid-1).trans.col = (1:3) + xg_col;    xg_col = xg_col + 3;
    end
    