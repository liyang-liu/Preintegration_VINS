function [dp, dv, dphi] = fn_CalPerfectIMUdlt(X_obj, nPoses, nPts, J, dt, SLAM_Params )

dt = 1;
dlt = 9*(nPoses - 1);
%id1x = ((nPoses-1)*6+nPts*3+3*nPoses+10);

dbf = X_obj.bf - SLAM_Params.bf0;
dbw = X_obj.bw - SLAM_Params.bw0;
%dbf = x(id1x:(id1x+2),1)- SLAM_Params.bf0;
%dbw = x((id1x+3):(id1x+5),1)- SLAM_Params.bw0;
%id1x = (nPoses-1)*6+nPts*3+3*nPoses+1;
%g = x(id1x:(id1x+2),1);
g = X_obj.g;

Ru1 = eye(3); 
Tu1 = zeros(3,1);

% Reprojection at each pose
%intlDelta = [];
dp = [];
dv = [];
dphi = [];

for pid=2:nPoses
    %id1x = ((pid-2)*6+1);%(nPoses-1)*6+nPts*3+3*nPoses + 9
    %alpha = x(id1x);beta = x(id1x + 1); gamma = x(id1x + 2);
    %Tu2 = x((id1x+3):(id1x+5));
    Ru2 = fn_RFromAngVec( X_obj.pose(pid-1).ang.val );
    Tu2 = X_obj.pose(pid-1).trans.xyz;
    
    %id1x = ((nPoses-1)*6+nPts*3+(pid-2)*3+1);
    %v1 = x(id1x:(id1x+2)); v2 = x((id1x+3):(id1x+5));
    v1 = X_obj.velocity(pid-1).xyz;
    v2 = X_obj.velocity(pid).xyz;
    
    [dp_i, dv_i, dphi_i] = fn_PredictIMUdlt(Tu1,Tu2,Ru1,Ru2,v1,v2,g,dbf,dbw,dt,J{pid});    
    %id1x = (pid-2)*9+1;
    %%dlt(id1x:(id1x+8)) = [dp;dv;dphi];% - Zobs(id1x:(id1x+8));
    
    dp = [dp, dp_i];
    dv = [dv, dv_i];
    dphi = [dphi, dphi_i];
    
    if 0
        intlDelta(end+1) = struct( ...
            'deltaP',   struct( 'val', dp,   'row', (1:3) + zrow ), ...
            'deltaV',   struct( 'val', dv,   'row', (4:6) + zrow ), ...
            'deltaPhi', struct( 'val', dphi, 'row', (7:9) + zrow ) );
        zrow = zrow + 9;
    end
    
    Tu1 = Tu2; Ru1 = Ru2;
end
