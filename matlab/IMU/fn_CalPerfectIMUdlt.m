function [dlt] = fn_CalPerfectIMUdlt(x, nPoses, nPts, J, dt, bf0, bw0)

dt = 1;
dlt = 9*(nPoses - 1);
id1x = ((nPoses-1)*6+nPts*3+3*nPoses+10);
dbf = x(id1x:(id1x+2),1)- bf0;
dbw = x((id1x+3):(id1x+5),1)- bw0;
id1x = (nPoses-1)*6+nPts*3+3*nPoses+1;
g = x(id1x:(id1x+2),1);
Ru1 = eye(3); 
Tu1 = zeros(3,1);
% Reprojection at each pose
for pid=2:nPoses
    id1x = ((pid-2)*6+1);%(nPoses-1)*6+nPts*3+3*nPoses + 9
    alpha = x(id1x);beta = x(id1x + 1); gamma = x(id1x + 2);
    Ru2 = fn_RFromABG(alpha, beta, gamma);
    Tu2 = x((id1x+3):(id1x+5));
    id1x = ((nPoses-1)*6+nPts*3+(pid-2)*3+1);
    v1 = x(id1x:(id1x+2)); v2 = x((id1x+3):(id1x+5));
    [dp,dv,dphi] = fn_PredictIMUdlt(Tu1,Tu2,Ru1,Ru2,v1,v2,g,dbf,dbw,dt,J{pid});    
    id1x = (pid-2)*9+1;
    dlt(id1x:(id1x+8)) = [dp;dv;dphi];% - Zobs(id1x:(id1x+8));
    Tu1 = Tu2; Ru1 = Ru2;
end
