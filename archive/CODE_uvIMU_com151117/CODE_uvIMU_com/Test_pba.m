    
datadir = ['.' filesep 'Whole170R' filesep 'Result' filesep];
%     imgdir = ['.' filesep 'Whole170R' filesep];
%     imufulldir = ['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
gtFile = ['.' filesep 'Whole170R' filesep 'GT_P0_PA.mat'];

load([datadir 'PBAPose.mat']);
load(gtFile);
nPoses = 170;
    figure(); hold on;
    plot(GT_P0(:,4),GT_P0(:,6),'-+r');
    plot(PBAPose(:,4),PBAPose(:,6),'-*b'); 
    axis equal;
    
    figure();
    err = (PBAPose(1:nPoses,4:6) - GT_P0(1:nPoses, 4:6))';
    ce = complex(err(1,:), err(2,:));
    ce = complex(abs(ce), err(3,:));  
    plot(1:nPoses, abs(ce), 'p');