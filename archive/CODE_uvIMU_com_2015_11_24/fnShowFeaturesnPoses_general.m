function [] = fnShowFeaturesnPoses_general(x, nPoses, nPts, nIMUdata, bPreInt, stitle)
%% Show the postions of features and poses according to the state vector x.
%% Input: 
% x: composed of nPoses poses, nPts 3D features and others. 


R_cell = [];
T_cell = [];
R_cell{1} = eye(3);
T_cell{1} = zeros(3,1);


pid = 1;
ratio = 0.2;
fh = figure; 
set(gcf, 'Position', get(0, 'ScreenSize'));
title(stitle); hold on;
quiver3(T_cell{pid}(1), T_cell{1}(2), T_cell{1}(3), R_cell{pid}(1,1), R_cell{pid}(1,2), R_cell{pid}(1,3), ratio);
quiver3(T_cell{pid}(1), T_cell{1}(2), T_cell{1}(3), R_cell{pid}(2,1), R_cell{pid}(2,2), R_cell{pid}(2,3), ratio);
quiver3(T_cell{pid}(1), T_cell{1}(2), T_cell{1}(3), R_cell{pid}(3,1), R_cell{pid}(3,2), R_cell{pid}(3,3), ratio);
if(bPreInt == 1)
    nMax = nPoses;
    nVidEnd = 6*(nPoses-1)+3*nPts;
    nVidStart = nVidEnd + 1;
    nVidEnd = nVidEnd + 3*nPoses;
else
    nMax = nIMUdata;%(nPoses-1)*nIMUrate;
    nVidEnd = 6*nIMUdata+3*nPts;
    nVidStart = nVidEnd + 1;
    nVidEnd = nVidEnd + 3*(nIMUdata+1);    
end


for pid=2:nMax
    id = (pid-2)*6+1;
    R_cell{pid} = fnR5ABG(x(id), x(id+1), x(id+2)); 
    T_cell{pid} = x((id+3):(id+5),1); 
    pc(:, pid) = T_cell{pid};
    figure(fh);
    quiver3(x(id+3), x(id+4), x(id+5), R_cell{pid}(1,1), R_cell{pid}(1,2), R_cell{pid}(1,3), ratio);
    quiver3(x(id+3), x(id+4), x(id+5), R_cell{pid}(2,1), R_cell{pid}(2,2), R_cell{pid}(2,3), ratio);
    quiver3(x(id+3), x(id+4), x(id+5), R_cell{pid}(3,1), R_cell{pid}(3,2), R_cell{pid}(3,3), ratio);
    pause(0.1);
end

if(bPreInt == 1)
    id0 = (nPoses-1)*6;
else
    id0 = nIMUdata*6;%(nPoses-1)*nIMUrate
end

for pid=1:nPts
    id = id0 + (pid-1)*3+1;
    p(:, pid) = x(id:(id+2));
    
end

figure(fh);
plot3(pc(1,:), pc(2,:), pc(3,:),'--o');%b
if(nPts > 0)
    plot3(p(1,:), p(2,:), p(3,:),'p');%r
end
view(-45, 20);

nfontsize = 25;%22;%18;
sDir = 'xyz';
figure(); 
set(gcf, 'Position', get(0, 'ScreenSize'));
tv = x(nVidStart:nVidEnd);
tv = reshape(tv, 3, []);

for(i=1:3)
    subplot(3,1,i);hold all; 
    set(gca,'FontSize', nfontsize);
    set(get(gca,'XLabel'), 'FontSize', nfontsize);
    set(get(gca,'YLabel'), 'FontSize', nfontsize);
    plot((tv(i,:))', '*:'); 
    legend(sprintf('V%c', sDir(i)));    
end
subplot(3,1,1);
title('IMU Velocity');
    
    
