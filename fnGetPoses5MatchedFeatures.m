function [Rp, Ap, Tp, Feature3D] = fnGetPoses5MatchedFeatures(nPoses, nPts, ...
    K, fscaleGT, RptFeatureObs, kfids)%, bAddInitialNoise, sigmauov)

    global InertialDelta_options
    
R = eye(3);
Ap = zeros(3, nPoses);
Tp = zeros(3, nPoses);
Rp = zeros(3, 3, nPoses);
Rp(:,:,1) = eye(3);

if(bSimData == 0)
    nPts = 60000;
end
Feature3D = zeros(nPts, 50);%[fid, nObs, [pidi, pidi1, xi, yi, zi]]
fId_FeatureObs = 1; nTrianguTimes = 2;
Feature3D(:, fId_FeatureObs) = 1:nPts;

% %% Get the first image
% pid = 1;
% if(bSimData == 1)
%     obsfeatures_i = uvd_cell{1}(1:2, :);
%     fidset_i = (1:nPts)';
% elseif(bMalaga == 1)
%     load(sprintf('%sImage%d.mat', imgdir, pid));
%     obsfeatures_i = Image(2:end, 1:3);
%     fidset_i = Image(2:end, 1);
% % elseif(bDinuka == 1)
% %     imid = kfids(pid);%1+(pid-1)*kfspan);
% %     load(sprintf('%simage_%d.mat', imgdir, imid));
% %     obsfeatures_i = features;
% %     fidset_i = features(:, 1);            
% end

% if(bDinuka == 0)
%     for(pid=2:nPoses)
%         if(bSimData == 1)
%             obsfeatures_i1 = uvd_cell{pid}(1:2, :);
%             fidset_i1 = (1:nPts)';
%         elseif(bMalaga == 1)
%             load(sprintf('%sImage%d.mat', imgdir, pid));
%             obsfeatures_i1 = Image(2:end, 1:3);
%             fidset_i1 = Image(2:end, 1);           
%         end
% 
%         [comid, id_i, id_i1] = intersect(fidset_i, fidset_i1);
%         if(bSimData == 1)
%             comfeatures = [obsfeatures_i(:, id_i); obsfeatures_i1(:, id_i1)];
%         else
%             comfeatures = ([obsfeatures_i(id_i, 2:3), ...
%                 obsfeatures_i1(id_i1, 2:3)])';
%         end
% 
% 
%         [Ri, Ti, E] = FuncComputeRT(comfeatures, K);
%         fscale = fscaleGT(pid)/norm(Ti);
%         Ti = fscale * Ti;
%         %Tp(:, pid) = Tp(:, pid-1) + R'*Ti;
%         R = Ri*R;
%         Rp(:, :, pid) = R;
%         [Ap(1, pid), Ap(2, pid), Ap(3, pid)] = fnABG5R(R);
%         Tp(:, pid) = Tp(:, pid-1) - R'*Ti;   
%         nComPts = size(comid, 1);
%         [p3d] = fnTrianguFeatures(K, Rp(:, :, pid-1), Tp(:, pid-1), ...
%             Rp(:, :, pid), Tp(:, pid), comfeatures);
%         Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
%         ntrigtms = Feature3D(comid, nTrianguTimes);
%         Feature3D(comid, (3+5*(ntrigtms-1)):(7+5*(ntrigtms-1))) = ...
%             [repmat([pid-1, pid], nComPts, 1), -p3d];
% 
%         fidset_i = fidset_i1; 
%         obsfeatures_i = obsfeatures_i1;
%     end
% else% if(bDinuka == 1)

    nRptfs = size(RptFeatureObs, 1);
    for(pid=2:nPoses) 
        comfeatures = [];
        comid = [];        
        for(fid=1:nRptfs)
            obsId = 1;
            while((RptFeatureObs(fid, 3*obsId) ~= 0) && (RptFeatureObs(fid, 3*obsId) < pid-1))
                obsId = obsId + 1;
            end
            if((RptFeatureObs(fid, 3*obsId) == (pid-1)) && ...
                    (RptFeatureObs(fid, 3*(obsId+1)) == pid))
                comfeatures = [comfeatures; RptFeatureObs(fid, (3*obsId+1):(3*obsId+2)), ...
                    RptFeatureObs(fid, (3*obsId+4):(3*obsId+5))];
                comid = [comid; RptFeatureObs(fid, 1)];
            end
        end
        comfeatures = comfeatures';
%         %%%
%         nfs = size(comfeatures, 2);
%         X1 = [comfeatures(1:2, :); ones(1, nfs)];
%         X2 = [comfeatures(3:4, :); ones(1, nfs)];        
%         [~, bestmodel] = ransacF(X1,X2);
%         comfeatures = comfeatures(:, bestmodel);
%         comid =  comid(bestmodel);
%         %%%
        [Ri, Ti, E] = FuncComputeRT(comfeatures, K);
        fscale = fscaleGT(pid)/norm(Ti);
        Ti = fscale * Ti;
        %Tp(:, pid) = Tp(:, pid-1) + R'*Ti;
        R = Ri*R;
        Rp(:, :, pid) = R;
        [Ap(1, pid), Ap(2, pid), Ap(3, pid)] = fnABG5R(R);
        Tp(:, pid) = Tp(:, pid-1) - R'*Ti;   
        nComPts = size(comid, 1);
        [p3d] = fnTrianguFeatures(K, Rp(:, :, pid-1), Tp(:, pid-1), ...
            Rp(:, :, pid), Tp(:, pid), comfeatures);
        Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
        ntrigtms = Feature3D(comid, nTrianguTimes);
        for(fid=1:nComPts)
            Feature3D(comid(fid), (3+5*(ntrigtms(fid)-1)):(7+5*(ntrigtms(fid)-1))) = ...
                [pid-1, pid, -p3d(fid, :)]; 
        end
        
    end        
end


