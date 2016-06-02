%% The main function.

clear;
close all;
clc;

addpath(genpath('ms3D'));

% Choose to use simulated data or or real data.
bSimData = 1;

if(bSimData)
    nPoses = 2; nPts = 20;
	cx0 = 240; cy0 = 320; f = 575;
	K = [f 0 cx0; 0 f cy0; 0 0 1];
	[uvd_cell, R_cell, T_cell] = funSimFeaturesAtNPoses(nPoses, nPts);%[uvd1, uvd2, R, T] = funSimFeaturesAtTwoPoses();
    %fp1 = uvd_cell{1}; fp2 = uvd_cell{2};%
    R = R_cell{2}; T = T_cell{2};
    uvd1 = uvd_cell{1}; uvd2 = uvd_cell{2};
    fp1 = uvd1; fp2 = uvd2;
    fp1(1, :) = (uvd1(1, :) - cx0) .* fp1(3, :) / f;
    fp1(2, :) = (uvd1(2, :) - cy0) .* fp1(3, :) / f;
    fp2(1, :) = (uvd2(1, :) - cx0) .* fp2(3, :) / f;
    fp2(2, :) = (uvd2(2, :) - cy0) .* fp2(3, :) / f;
	p10 = fp1'; p20 = fp2';
	u10f = (uvd1(1, :))'; v10f = (uvd1(2, :))';
	u20f = (uvd2(1, :))'; v20f = (uvd2(2, :))';
    
%     if(isunix) 
%         imgdir = '/home/yowang/Documents/segDataset/data10/rgb0000.png';
%     else
%         imgdir = 'E:\uDocs\Research\MyPapers\acra2013\SegData\data10\rgb0000.png';
%     end
%     I1 = imread(imgdir);    
    I1 = [];
    [alpha, beta, gamma] = fABGfrmR(R);
    x = [alpha; beta; gamma; T; fp1(:)];
    %fprintf('Ground Truth Value:\n\t Xg=[%f,%f,%f,%f,%f,%f]\n', x);
    fprintf('Ground Truth Value:\n\t Xg=[');
    fprintf('%f ', x);
    fprintf(']\n');
else	
	cSift = 1;%0;% % 0--use sift provided by matlab; 1--use another sift realization
	if(cSift == 1)%
		addpath(genpath('sift'));
		featurename = 'SIFT';
	else
		featurename = 'SURF';
	end
	cDataset = 0;%1;%1;% 0--My own dataset; 1--Freigburg dataset; 2-- face
	bMyApril = 1;%0;%
	datasetnum = 10;
	datasetname = sprintf('data%d', datasetnum);
	dltnp = 1;%4;

		if(cDataset == 0)
		    fx = 525.0; %# focal length x
		    fy = 525.0; %# focal length y
		    cx = 319.5; %# optical center x
		    cy = 239.5; %# optical center y
		elseif(cDataset == 2)%%data parameters
		    fx = 575.8157496;%# focal length x
		    fy = 575.8157496;%# focal length y
		    cx = 320.;%# optical center x
		    cy = 240.;%# optical center y        
		elseif(cDataset == 1)%%Freigburg data parameters
		    fx = 575.8157;%# focal length x
		    fy = 575.8157;%# focal length y
		    cx = 320.;%# optical center x
		    cy = 240.;%# optical center y
		end

	K = [fx 0 cx; 0 fy cy; 0 0 1];

	% Initialization
	alpha = pi/3;
	beta = pi/6;
	gamma = pi/4;
	T = [1, 2, 3.5]';

	% 1. p3d0, alpha, beta, gamma, T => p3d
	%p3d0 = 
	% 2. p3d0, p3d, K => p2d0, p2d

	% 1. Read images I1, I2
	dmin = 0.01;%0.5;%0.8;%1.;%1.2;%
	dmax = 4;%100;%3.5;%
	Td = 0.035;% 0.02;
	Tnd = Td;
	dfactor = 255./4;
	% Matlab SURF
	fMthreshold = 1000.;%500.;%2000.;%1500.;%10.;%1.;%
	nSlevel = 4;%5;%6;%

	if(isunix) 
	   dataDir = ['/home/yowang/Documents/segDataset/', datasetname, filesep];% cluster %(HP)'/mnt/B0A18DDEEC101C79/uDocs/Research/MyPapers/acra2013/code/SegData/data10/'; %
	   %saveDir = ['../datasetinfo/', datsetname, '/unix/'];
	else % my pc
	   dataDir = ['E:\uDocs\Research\MyPapers\acra2013\code\SegData\', datasetname, filesep]; 
	   %saveDir = fullfile('..', 'datasetinfo', datasetname, filesep);
	end
	saveDir = fullfile('..', 'datasetinfo', datasetname, featurename, filesep);

	%% Extract features
	% Batch processing
	if(~exist(saveDir, 'dir'))
		mkdir(saveDir);
	end

	for nPose = 0:dltnp:dltnp
		%sprintf('../segDataset/data%d/', ndg);
		sfrgb1= sprintf('%srgb%04d.png', dataDir, nPose);%ni{ndg}()
		sfd1= sprintf('%sdepth%04d.png', dataDir, nPose);%ni{ndg}()
		if(~exist(fullfile(saveDir, sprintf('%04d.mat', nPose))))            
		    %sfrgb2= sprintf('%srgb%04d.png', dir, nPose+1);%ni{ndg}()
		    %sfd2= sprintf('%sdepth%04d.png', dir, nPose+1);%ni{ndg}()

		    [fn, ~] = Collectimgfs(sfrgb1, sfd1, nPose, dfactor, K, ...
		                                fMthreshold, nSlevel, cSift, cDataset, dmin, dmax);
		    ImgInfo.featureset = fn;
		    ImgInfo.rgb = sfrgb1;
		    ImgInfo.depth = sfd1;
		    s = [saveDir sprintf('%04d.mat', nPose)];%['../Output/mbdSLAM/' savename '.mat'];
		    save(s, 'ImgInfo', '-v7.3');%, 'nPose');
		end
	end


	%% Match features
	nPose = 0;
	load(fullfile(saveDir, sprintf('%04d.mat', nPose)));
	I2 = imreadbw(ImgInfo.rgb); % from I2->I1, R/T correspond to camera I1->I2
	ImgInfo2 = ImgInfo;
	nPose = nPose + dltnp;
	load(fullfile(saveDir, sprintf('%04d.mat', nPose)));
	I1 = imreadbw(ImgInfo.rgb);
	ImgInfo1 = ImgInfo;

	if(cSift == 0) 
		matchpairs = matchFeatures(ImgInfo1.featureset(:, 8:end), ImgInfo2.featureset(:, 8:end), 'Prenormalized', false);%, 'MatchThreshold', 100.0) ;
		mnp1 = matchpairs;
	else
		% By passing to integers we greatly enhance the matching speed (we use
		% the scale factor 512 as Lowe's, but it could be greater without
		% overflow)
	%                 descr1=uint8(512*fn(:, 8:end)) ;
	%                 descr2=uint8(512*fk(:, 8:end)) ;
	%                 matchpairs = siftmatch(descr1', descr2');%(fn(:, 8:end))', (fk(:, 8:end))');
		matchpairs = siftmatch((ImgInfo1.featureset(:, 8:end))', (ImgInfo2.featureset(:, 8:end))');%, 5);
		mnp1 = matchpairs';
	end

	% get rid of repeating items
	[index_pairs] = GetUniqueMatch(mnp1);
	   

	%% Segmentation and visual odometry
	% ACRA method 
	%bPrev = 0; nGroup = 0; bStatic = 0; bTdAdpt = 0;
	%[Yaw, Pitch, Roll, R, T, mnp1] = findInliers(bPrev, nGroup, nPose, I1, I2, bStatic, Td, ...
	%                                bTdAdpt, K, cSift, ImgInfo1.featureset, ImgInfo2.featureset, ... index_pairs);%img{nPose}, img{nPose+1}

	% WCICA method
		u10f = ImgInfo1.featureset(index_pairs(:, 1), 3);
		v10f = ImgInfo1.featureset(index_pairs(:, 1), 4);
		z1 = ImgInfo1.featureset(index_pairs(:, 1), 7);
		p10 = ImgInfo1.featureset(index_pairs(:, 1), 5:7);        
		u20f = ImgInfo2.featureset(index_pairs(:, 2), 3);
		v20f = ImgInfo2.featureset(index_pairs(:, 2), 4);
		z2 = ImgInfo2.featureset(index_pairs(:, 2), 7);
		p20 = ImgInfo2.featureset(index_pairs(:, 2), 5:7); 
end

% Prepare for motion segmentation
    y(1:2,:,1) = [u10f, v10f]';
    y(3,:,1) = 1;
    y(1:2,:,2) = [u20f, v20f]'; 
    y(3,:,2) = 1;
	nminp = 3; 
	fthreshold=0.5; 
	nminclustersize = 3; 
	colors = 'brgymkwcbrgymkwcbrgymkwc';
	bDisplay = 1;
	sinfo = 'Motion3DSeg';
    [groupid, R, T1, ninliers] = Motion3DSegment(I1, y, p10', p20', nminp, fthreshold, ...
                                    nminclustersize, colors, bDisplay, sinfo); 
	%T1
    [alpha1, beta1, gamma1] = fABGfrmR(R);
%     T1 = T1 + randn(3,1)/12;
%     alpha1 =alpha1 + randn()/12;
%     beta1 = beta1 + randn()/12;
%     gamma1 = gamma1 + randn()/12;
    u1f = u10f(groupid);
    v1f = v10f(groupid);
    u2f = u20f(groupid);
    v2f = v20f(groupid);
    d1f = p10(groupid, 3);
	d2f = p20(groupid, 3);
    p1 = (p10(groupid, :))';   
    x = [alpha1; beta1; gamma1; T1; p1(:)];%x(1), x(2), x(3), x(4), x(5), x(6)
    x = x + 0.1*ones(size(x));%randn(size(x))/6;
    Np = size(u1f, 1) * 2;
	covInv = eye(3); covInv(3,3) = 0.01;
    covInv = inv(covInv);
    CovMatrixInv = kron(eye(Np), covInv);
    fprintf('Initial Value:\n\t X0=[');
    fprintf('%f ', x);
    fprintf(']\n');
% 3. Compute Jacobian
nMax = 20; fLowerbound = 1e-6;
times = 0;
while(times < nMax)
    fprintf('times=%d ',times);
    [e] = fnPredictUVDErr(K, x, u1f, v1f, d1f, u2f, v2f, d2f);
    if(max(abs(e)) < fLowerbound)
        break;
    end
    J = fJ3duv(K, x);
	Info = J'*CovMatrixInv*J;
	E = -J'*CovMatrixInv*e;
	dx = Info\E;
	%dx = - (J' * J) \ J' * e;    
	if(max(abs(dx)) < fLowerbound)
        break;
    end

    x = x + dx;
    times = times + 1;
end    
fprintf('\nIteration Times:\n\t N = %d\n', times);
if( times == nMax)
    fprintf(' ***REACH MAXIMUM TIMES***\n');
end
%fprintf('Final Value:\n\t Xf=[%f,%f,%f,%f,%f,%f]\n', x);
fprintf('Final Value:\n\t Xf=[');
fprintf('%f ', x);
fprintf(']\n');
