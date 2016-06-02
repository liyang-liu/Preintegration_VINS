function [] = fnShowEstmBnd_general(ef, Td, nPoses, nPts, nIMUdata, bPreInt, bUVonly, bVarBias)
%%
% Au,Tu,f,vu, 

nfontsize = 25;%22;%18;
sDir = 'xyz';%{'x', 'y', 'z'};
%% Display poses
% Au
if(bPreInt == 1)
    nMax = 6*(nPoses - 1);
else
    nMax = 6*nIMUdata;%(nPoses - 1)*nIMUrate;
end

tv = ef(1:nMax);
tv = reshape(tv, 6, []);
tvb = Td(1:nMax);
tvb = reshape(tvb, 6, []);

% Au
figure('Name', 'imuRotation: Final Error V.S. 3\sigma Bound', 'NumberTitle', 'off'); 
set(gcf, 'Position', get(0, 'ScreenSize'));
for(i=1:3)
    subplot(3,1,i);hold all; 
    set(gca,'FontSize', nfontsize);
    set(get(gca,'XLabel'), 'FontSize', nfontsize);
    set(get(gca,'YLabel'), 'FontSize', nfontsize);
    plot((tv(i,:))', '*:'); plot((tvb(i,:))', 'or-.');plot((-tvb(i,:))', 'or-.'); 
    legend(sprintf('Au%c', sDir(i)), '3\sigma');    
end
subplot(3,1,1);
title('IMU Rotation: Final Error V.S. 3\sigma Bound');
% set(gca,'FontSize', nfontsize);
% set(get(gca,'XLabel'), 'FontSize', nfontsize);
% set(get(gca,'YLabel'), 'FontSize', nfontsize);

% Tu
figure('Name', 'imuTranslation: Final Error V.S. 3\sigma Bound', 'NumberTitle', 'off');  
set(gcf, 'Position', get(0, 'ScreenSize'));
for(i=4:6)
    subplot(3,1,i-3);hold all; 
    set(gca,'FontSize', nfontsize);
    set(get(gca,'XLabel'), 'FontSize', nfontsize);
    set(get(gca,'YLabel'), 'FontSize', nfontsize);
    plot((tv(i,:))', '*:'); plot((tvb(i,:))', 'or-.');plot((-tvb(i,:))', 'or-.'); 
    legend(sprintf('Tu%c', sDir(i-3)), '3\sigma');    
end
subplot(3,1,1);
title('IMU Translation: Final Error V.S. 3\sigma Bound');
% title('The Final IMU Translation Error Compared with the Uncertainty');
% set(gca,'FontSize', nfontsize);
% set(get(gca,'XLabel'), 'FontSize', nfontsize);
% set(get(gca,'YLabel'), 'FontSize', nfontsize);

%% Display Features
if(bPreInt == 1)
    nMin = 6*(nPoses - 1)+1;
    nMax = 6*(nPoses - 1)+3*nPts;
else
    nMin = 6*nIMUdata + 1;%(nPoses - 1)*nIMUrate
    nMax = 6*nIMUdata + 3*nPts;%(nPoses - 1)*nIMUrate
end

tv = ef(nMin:nMax);
tv = reshape(tv, 3, []);
tvb = Td(nMin:nMax);
tvb = reshape(tvb, 3, []);

% f
figure('Name', 'featurePosition: Final Error V.S. 3\sigma Bound', 'NumberTitle', 'off');
set(gcf, 'Position', get(0, 'ScreenSize'));
for(i=1:3)
    subplot(3,1,i);hold all; 
    set(gca,'FontSize', nfontsize);
    set(get(gca,'XLabel'), 'FontSize', nfontsize);
    set(get(gca,'YLabel'), 'FontSize', nfontsize);
    plot((tv(i,:))', '*:'); plot((tvb(i,:))', 'or-.');plot((-tvb(i,:))', 'or-.'); 
    legend(sprintf('f%c', sDir(i)), '3\sigma');    
end
subplot(3,1,1);
title('Feature Position: Final Error V.S. 3\sigma Bound');
% title('The Final Feature Position Error Compared with the Uncertainty');
% set(gca,'FontSize', nfontsize);
% set(get(gca,'XLabel'), 'FontSize', nfontsize);
% set(get(gca,'YLabel'), 'FontSize', nfontsize);

if(bUVonly == 1)
    return;
end
%% Display Velocities
if(bPreInt == 1)
    nMin = 6*(nPoses - 1)+3*nPts+1;
    nMax = 6*(nPoses - 1)+3*nPts+3*nPoses;
else
    nMin = 6*nIMUdata + 3*nPts+1;
    nMax = 6*nIMUdata + 3*nPts + 3*(nIMUdata+1);
end

tv = ef(nMin:nMax);
tv = reshape(tv, 3, []);
tvb = Td(nMin:nMax);
tvb = reshape(tvb, 3, []);

% v
figure('Name', 'imuVelocity: Final Error V.S. 3\sigma Bound', 'NumberTitle', 'off');
set(gcf, 'Position', get(0, 'ScreenSize'));
for(i=1:3)
    subplot(3,1,i);hold all; 
    set(gca,'FontSize', nfontsize);
    set(get(gca,'XLabel'), 'FontSize', nfontsize);
    set(get(gca,'YLabel'), 'FontSize', nfontsize);
    plot((tv(i,:))', '*:'); plot((tvb(i,:))', 'or-.');plot((-tvb(i,:))', 'or-.'); 
    legend(sprintf('V%c', sDir(i)), '3\sigma');    
end
subplot(3,1,1);
title('IMU Velocity: Final Error V.S. 3\sigma Bound');
% title('The Final Velocity Error Compared with the Uncertainty');
% set(gca,'FontSize', nfontsize);
% set(get(gca,'XLabel'), 'FontSize', nfontsize);
% set(get(gca,'YLabel'), 'FontSize', nfontsize);

%% g, Au2c, Tu2c, bf, bw
if(bPreInt == 1)
    nMin = 6*(nPoses - 1)+3*nPts+3*nPoses+1;
    if(bVarBias == 0)
        nMax = 6*(nPoses - 1)+3*nPts+3*nPoses+15;
    else
        nMax = 6*(nPoses - 1)+3*nPts+3*nPoses+3*3 + (nPoses-1)*6;
    end
else
    nMin = 6*nIMUdata + 3*nPts + 3*(nIMUdata+1)+1;
    if(bVarBias == 0)
        nMax = 6*nIMUdata + 3*nPts + 3*(nIMUdata+1)+15;
    else
        nMax = 6*nIMUdata + 3*nPts + 3*(nIMUdata+1)+3*3 + (nPoses-1)*6;
    end
end
tv = ef(nMin:nMax);
tv = reshape(tv, 3, []);
tvb = Td(nMin:nMax);
tvb = reshape(tvb, 3, []);
% z
figure('Name', '[g, Au2c, Tu2c, bf, bw]: Final Error V.S. 3\sigma Bound', 'NumberTitle', 'off');
set(gcf, 'Position', get(0, 'ScreenSize'));
for(i=1:3)
    subplot(3,1,i);hold all; 
    set(gca,'FontSize', nfontsize);
    set(get(gca,'XLabel'), 'FontSize', nfontsize);
    set(get(gca,'YLabel'), 'FontSize', nfontsize);
    plot((tv(i,:))', '*:'); plot((tvb(i,:))', 'or-.');plot((-tvb(i,:))', 'or-.'); 
    legend(sprintf('Z%c', sDir(i)), '3\sigma');    
end
subplot(3,1,1);
title('[g, Au2c, Tu2c, bf, bw]: Final Error V.S. 3\sigma Bound');
% title('The Final (g, Au2c, Tu2c, bf, bw) Error Compared with the Uncertainty');
% set(gca,'FontSize', nfontsize);
% set(get(gca,'XLabel'), 'FontSize', nfontsize);
% set(get(gca,'YLabel'), 'FontSize', nfontsize);

end
    