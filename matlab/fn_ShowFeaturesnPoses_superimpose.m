function [] = fnShowFeaturesnPoses_general(Xg_obj, X_obj, stitle)
%% Show the postions of features and poses according to the state vector x.
%% Input: 
% x: composed of nPoses poses, nPts 3D features and others. 
global InertialDelta_options
fig = figure(); hold on;

nPts = length( Xg_obj.feature );
nPoses = 1 + length ( Xg_obj.pose );

p=[]; 
for pid=1:nPts; 
    p(:, pid) = Xg_obj.feature(pid).xyz; 
end; 
plot3(p(1,:), p(2,:), p(3,:),'p', 'color', 'r');

p=[]; 
for pid=1:nPts; 
    p(:, pid) = X_obj.feature(pid).xyz; 
end; 
plot3(p(1,:), p(2,:), p(3,:),'p', 'color', 'b');

pc=[];
for pid=2:nPoses; 
    pc(:, pid) = Xg_obj.pose(pid-1).trans.val;
end; 
plot3(pc(1,:), pc(2,:), pc(3,:),'--o', 'color', 'r');

pc=[];
for pid=2:60; 
    pc(:, pid) = X_obj.pose(pid-1).trans.val;
end; 

plot3(pc(1,:), pc(2,:), pc(3,:),'--o', 'color', 'b');

grid on;
lgd={ 'Feature-3D GT', 'Feature-3D Slam', 'Pose-3D GT', 'Pose-3D Slam' }; 
l = legend( lgd ,'FontSize',8,'FontWeight','bold' );
title(l, 'Ground Truth vs SLAM')

hold off;
