%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% This is a block of code discarded from Main_inc, never used, possibly
% from Yubin's thesis, may be valuable in future as it contains SIFT
% and extraction of feature using motion analysis
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	cSift = 1;%0;% % 0--use sift provided by matlab; 1--use another sift realization
	if(cSift == 1)%
		addpath(genpath('sift'));
		featurename = 'SIFT';
	else
		featurename = 'SURF';
	end
	cDataset = 3;%0;%1;%1;% 0--My own dataset; 1--Freigburg dataset; 2-- face; 3--KITTI
	bMyApril = 1;%0;%
	datasetnum = 10;
	datasetname = '2011_09_30_drive_0020_sync';%sprintf('data%d', datasetnum);
    dataDir_root = '/home/youbwang/Documents/KITTI/2011_09_30/2011_09_30_drive_0020_sync/image_00/';
	dltnp = 10;%4;

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
        elseif(cDataset == 3)
		    fx = 575.8157;%# focal length x
		    fy = 575.8157;%# focal length y
		    cx = 320.;%# optical center x
		    cy = 240.;%# optical center y          
        elseif(cDataset == 4) % KITTI dataset
		    fx = 7.215377e+02;%# focal length x
		    fy = 7.215377e+02;%# focal length y
		    cx = 6.095593e+02;%# optical center x
		    cy = 1.728540e+02;%# optical center y             
		end

	K = [fx 0 cx; 0 fy cy; 0 0 1];
% 
% 	% Initialization
% 	alpha = pi/3;
% 	beta = pi/6;
% 	gamma = pi/4;
% 	T = [1, 2, 3.5]';
% 
% 	% 1. p3d0, alpha, beta, gamma, T => p3d
% 	%p3d0 = 
% 	% 2. p3d0, p3d, K => p2d0, p2d
% 
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
	   dataDir = [dataDir_root, 'data', filesep];% cluster %(HP)'/mnt/B0A18DDEEC101C79/uDocs/Research/MyPapers/acra2013/code/SegData/data10/'; %
       nfiles = size(dir(dataDir),1)-3;%2;%5;%
	   %saveDir = ['../datasetinfo/', datsetname, '/unix/'];%/home/yowang/Documents/segDataset/
	else % my pc
	   %dataDir = ['E:\uDocs\Research\MyPapers\acra2013\code\SegData\', datasetname, filesep]; 
	   %saveDir = fullfile('..', 'datasetinfo', datasetname, filesep);
	end
	saveDir = fullfile(dataDir_root, 'datasetinfo', featurename, filesep);

	%% Extract features
	% Batch processing
	if(~exist(saveDir, 'dir'))
		mkdir(saveDir);
	end

	for nPose = 0:dltnp:nfiles
		%sprintf('../segDataset/data%d/', ndg);
        sfrgb1= sprintf('%s%010d.png', dataDir, nPose);
% 		sfrgb1= sprintf('%srgb%04d.png', dataDir, nPose);%ni{ndg}()
% 		sfd1= sprintf('%sdepth%04d.png', dataDir, nPose);%ni{ndg}()
		if(~exist(fullfile(saveDir, sprintf('%010d.mat', nPose))))            
		    %sfrgb2= sprintf('%srgb%04d.png', dir, nPose+1);%ni{ndg}()
		    %sfd2= sprintf('%sdepth%04d.png', dir, nPose+1);%ni{ndg}()

% 		    [fn, ~] = Collectimgfs(sfrgb1, sfd1, nPose, dfactor, K, ...
% 		                                fMthreshold, nSlevel, cSift, cDataset, dmin, dmax);
            [fdes,fp, Ig1] = DetectMonoFeatures(sfrgb1, fMthreshold, nSlevel, cSift);
		    ImgInfo.fdes = fdes;
            ImgInfo.fp = fp;
		    ImgInfo.rgb = sfrgb1;
		    %ImgInfo.depth = sfd1;
		    s = [saveDir sprintf('%010d.mat', nPose)];%['../Output/mbdSLAM/' savename '.mat'];
		    save(s, 'ImgInfo', '-v7.3');%, 'nPose');
		end
	end


	%% Match features
	nPose = 0;
	load(fullfile(saveDir, sprintf('%010d.mat', nPose)));
	I1 = imreadbw(ImgInfo.rgb); % from I2->I1, R/T correspond to camera I1->I2
	ImgInfo1 = ImgInfo;
	nPose = nPose + dltnp;
	load(fullfile(saveDir, sprintf('%010d.mat', nPose)));
	I2 = imreadbw(ImgInfo.rgb);
	ImgInfo2 = ImgInfo;

	if(cSift == 0) 
		matchpairs = matchFeatures(ImgInfo1.fdes(:, 1:end), ImgInfo2.fdes(:, 1:end), 'Prenormalized', false, 'MatchThreshold', 100.0);%);%8
		mnp1 = matchpairs;
	else
		% By passing to integers we greatly enhance the matching speed (we use
		% the scale factor 512 as Lowe's, but it could be greater without
		% overflow)
	%                 descr1=uint8(512*fn(:, 8:end)) ;
	%                 descr2=uint8(512*fk(:, 8:end)) ;
	%                 matchpairs = siftmatch(descr1', descr2');%(fn(:, 8:end))', (fk(:, 8:end))');
		matchpairs = siftmatch((ImgInfo1.fdes(:, 1:end))', (ImgInfo2.fdes(:, 1:end))',10);%, 100);%8  5
		mnp1 = matchpairs';
	end

	% get rid of repeating items
% 	[index_pairs] = GetUniqueMatch(mnp1);
    index_pairs = mnp1;%(1:500, :)index_pairs(1:10, :);
% 	   
% 
	%% Segmentation and visual odometry
% 	% ACRA method 
% 	%bPrev = 0; nGroup = 0; bStatic = 0; bTdAdpt = 0;
% 	%[Yaw, Pitch, Roll, R, T, mnp1] = findInliers(bPrev, nGroup, nPose, I1, I2, bStatic, Td, ...
% 	%                                bTdAdpt, K, cSift, ImgInfo1.featureset, ImgInfo2.featureset, ... index_pairs);%img{nPose}, img{nPose+1}
% 
	% WCICA method
		u10f = ImgInfo1.fp(index_pairs(:, 1), 1);%3
		v10f = ImgInfo1.fp(index_pairs(:, 1), 2);%4
% 		z1 = ImgInfo1.featureset(index_pairs(:, 1), 7);
% 		p10 = ImgInfo1.featureset(index_pairs(:, 1), 5:7);        
		u20f = ImgInfo2.fp(index_pairs(:, 2), 1);
		v20f = ImgInfo2.fp(index_pairs(:, 2), 2);
% 		z2 = ImgInfo2.featureset(index_pairs(:, 2), 7);
% 		p20 = ImgInfo2.featureset(index_pairs(:, 2), 5:7); 
% end
% 
% Prepare for motion segmentation
    y(1:2,:,1) = [u10f, v10f]';
    y(3,:,1) = 1;
    y(1:2,:,2) = [u20f, v20f]'; 
    y(3,:,2) = 1;
    X1 = y(:,:,1); X2 = y(:,:,2);
    addpath(genpath('Ransac'));
    addpath(genpath('Petercorke'));
    bRansac = 1;
    if(bRansac == 1)
        [F,inliers] = ransacF(X1,X2);
        %inliers = inliers(1:20);
        tmpim1 = im2double(I1);
        tmpim2 = im2double(I2);
        if 1
            % Display images and ALL matches
            hold on
            % Prepare lines to plot
            X = [X1(1,:)',X2(1,:)'];
            Y = [X1(2,:)',X2(2,:)'+size(tmpim1,1)];
            clf; imshow([tmpim1;tmpim2]);
            hold on;plot(X1(1,:),X1(2,:),'bo');plot(X2(1,:),X2(2,:)+size(tmpim1,1),'bo');hold on;%line(X',Y',zeros(size(X')),'Color','b');
            % Display onli inliers
            % Prepare lines to plot
            Xi = [X1(1,inliers)',X2(1,inliers)'];
            Yi = [X1(2,inliers)',X2(2,inliers)'+size(tmpim1,1)];
            hold on;line(Xi',Yi',zeros(size(Xi')),'Color','r');

        end;
        hold off;  
    end
    
%%%%%%%%%%%%%%%%%%%%%%%    
	nminp = 8;%3; 
    nModelid = 1;%2;%1;%Fundamental matrix; %
    bAdaptive = 1;
	fthreshold = 0.5; 
    fthreshold1 = 2.5;%3;%3.5;%4;%2.5;%0.5;%10;
	nminclustersize = 8;%3; 
	colors = 'brgymkwcbrgymkwcbrgymkwc';
	bDisplay = 1;
	sinfo = 'MoSeg_2D';%'Motion3DSeg';
    [grouppointid] = MotionSegment(y, nModelid, nminp, fthreshold, fthreshold1, ...
                                            nminclustersize, bAdaptive, colors, bDisplay, sinfo);
     [groupid, R, T1, ninliers] = MotionSegment(I1, y, p10', p20', nminp, fthreshold, ...
                                    nminclustersize, colors, bDisplay, sinfo);     
%     [groupid, R, T1, ninliers] = Motion3DSegment(I1, y, p10', p20', nminp, fthreshold, ...
%                                     nminclustersize, colors, bDisplay, sinfo); 
% 	%T1
%     [alpha1, beta1, gamma1] = fABGfrmR(R);
%     u1f = u10f(groupid);
%     v1f = v10f(groupid);
%     u2f = u20f(groupid);
%     v2f = v20f(groupid);
%     d1f = p10(groupid, 3);
% 	d2f = p20(groupid, 3);
%     p1 = (p10(groupid, :))';   

%% 1. Compose Observation Vector Zobs and nPoses, nPts:
    sFileFullPath = '\data\';
    [Zobs, nPoses, nPts, PosFt_mat] = fnRdFeature5PBAfile(sFileFullPath);

%% 2. Construct State Vector x:
    [alpha1, beta1, gamma1] = fABGfrmR(R);
    x = [alpha1; beta1; gamma1; T1; p1(:)];%x(1), x(2), x(3), x(4), x(5), x(6)
    x = x + randn(size(x))/6;
    Np = size(u1f, 1) * 2;
	covInv = eye(3); covInv(3,3) = 0.01;
    covInv = inv(covInv);
    CovMatrixInv = kron(eye(Np), covInv);
    fprintf('Initial Value:\n\t X0=[');
    fprintf('%f ', x);
    fprintf(']\n');
    
%% 3. VINS-BA
[x] = fnVI_BA(K, x, nPoses, nPts, dt, Jd, CovMatrixInv, nMaxIter, fLowerbound_e, fLowerbound_dx);

fprintf('Final Value:\n\t Xf=[');
fprintf('%f ', x);
fprintf(']\n');


