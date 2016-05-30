function [] = fnShowFeaturesnPoses_all(Xg_obj, X_init, X_final )
%% Show the postions of features and poses according to the state vector x.
%% Input: 
% x: composed of nPoses poses, nPts 3D features and others. 
global PreIntegration_options
fig = figure(); hold on;

nPts = length( Xg_obj.feature );
nPoses = 1 + length ( Xg_obj.pose );

% features
p_gt=[]; 
p_i=[];
p_f=[]; 
for fid=1:nPts; 
    p_gt(:, fid) = Xg_obj.feature(fid).xyz; 
    p_i(:, fid) = X_init.feature(fid).xyz; 
    p_f(:, fid) = X_final.feature(fid).xyz;     
end; 
plot3(p_gt(1,:), p_gt(2,:), p_gt(3,:),'p', 'color', 'r');
plot3(p_i(1,:), p_i(2,:), p_i(3,:),'p', 'color', 'g');
plot3(p_f(1,:), p_f(2,:), p_f(3,:),'p', 'color', 'b');
grid on;
view(-45, 20);

% poses
pc_gt = [];
pc_i  = [];
pc_f  = [];
for pid=2:nPoses; 
    pc_gt(:, pid) = Xg_obj.pose(pid-1).trans.xyz;
    pc_i(:, pid)  =  X_init.pose(pid-1).trans.xyz;
    pc_f(:, pid)  =  X_final.pose(pid-1).trans.xyz;
end; 
plot3( pc_gt(1,:), pc_gt(2,:), pc_gt(3,:), '--o', 'color', 'r' );
plot3( pc_i(1,:), pc_i(2,:), pc_i(3,:),    '--o', 'color', 'g' );
plot3( pc_f(1,:), pc_f(2,:), pc_f(3,:),    '--o', 'color', 'b' );

lgd={ 'Feature GT', 'Feature Init', 'Feature Slam', ...
    'Pose GT', 'Pose Init', 'Pose Slam' }; 

l = legend( lgd ,'FontSize',8,'FontWeight','bold' );
%t = title(l, 'GT vs Init, Final');
set(l, 'Location', 'northeast');

hold off;
