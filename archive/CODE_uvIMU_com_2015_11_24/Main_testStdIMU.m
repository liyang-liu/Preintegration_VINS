    clear;
    close all;
    clc;

    addpath(genpath('IMU'));

    imufulldir = ['.' filesep 'Malaga' filesep 'IMUrawData.mat'];
    gtFile = ['.' filesep 'Whole170R' filesep 'GT_P0_PA.mat'];
    load(imufulldir);
    
    Au2c = [-87.23; -2.99; -88.43]*pi/180;%[-86.19;-3.53;-90.31]*pi/180;%[0;0;0];%
    Ru2c = fnR5ABG(Au2c(1), Au2c(2), Au2c(3));
    Tu2c = [2.2-0.25;-0.427-0.029;0.025+(23-13.9)*1e-3];%[0;0;0];%
    dt = 1e-2;
    bf = zeros(3,1);
    bw = zeros(3,1);
    g0 = [0;0;-9.8];

    load('ImuTimestamps.mat');
    load('dtIMU.mat');
    nPoses = 5;
    ntimesteps = ImuTimestamps(nPoses)-ImuTimestamps(1);
    Timu = zeros(3, ntimesteps);
    v = zeros(3,ntimesteps);
    phi = zeros(3,ntimesteps);    
    
    k = 2;

    for(tid=ImuTimestamps(1):(ImuTimestamps(nPoses)-1))
        w = (IMUparking6L(tid, 2:4))';
        fb = (IMUparking6L(tid, 5:7))';
        [Timu(:,k), v(:,k), phi(:,k)] = fnStdPredict(Timu(:,k-1), v(:,k-1), phi(:,k-1), ...
            dt, fb, w, bf, bw, g0);
        k = k + 1;
    end
    
    Tcam = zeros(3, nPoses);
    for(tid = 1:(k-1))
        %Rcam = Ru2c*Rimu*Ru2c;
        Rimu = fnR5ABG(phi(1,tid), phi(2,tid), phi(3,tid));
        Tcam(:, tid) = Ru2c*(Timu(:, tid) - Tu2c + Rimu'*Tu2c);
    end
    
    load(gtFile);
    figure(); hold on;
    plot3(GT_P0(:,4),GT_P0(:,5),GT_P0(:,6),'-+r');
    plot3(Tcam(1,:),Tcam(2,:),Tcam(3,:),'-*b');
    plot3(Timu(1,:),Timu(2,:), Timu(3,:),'-pg');
    axis equal;