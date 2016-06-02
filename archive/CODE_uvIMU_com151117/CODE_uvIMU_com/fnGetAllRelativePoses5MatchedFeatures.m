function [dRi, dTi] = fnGetAllRelativePoses5MatchedFeatures(...
    nAllposes, K, fscaleGT, RptFeatureObs)%, bAddInitialNoise, sigmauov)

% R = eye(3);
% Ap = zeros(3, nPoses);
dTi = zeros(3, nAllposes);
dRi = zeros(3, 3, nAllposes);

% Feature3D = zeros(nPts, 50);%[fid, nObs, [pidi, pidi1, xi, yi, zi]]
% fId_FeatureObs = 1; nTrianguTimes = 2;
% Feature3D(:, fId_FeatureObs) = 1:nPts;
% vObsFids{1} = [];

    nRptfs = size(RptFeatureObs, 1);
    for(pid=2:nAllposes) 
%         vObsFids{pid} = vObsFids{pid-1};
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
        [dRi(:,:,pid), dTi(:,pid), ~] = FuncComputeRT(comfeatures, K);
        fscale = fscaleGT(pid)/norm(dTi);
        dTi = fscale * dTi;

%         R = Ri*R;
%         Rp(:, :, pid) = R;
%         [Ap(1, pid), Ap(2, pid), Ap(3, pid)] = fnABG5R(R);
%         Tp(:, pid) = Tp(:, pid-1) - R'*Ti;   
%         nComPts = size(comid, 1);
%         [p3d] = fnTrianguFeatures(K, Rp(:, :, pid-1), Tp(:, pid-1), ...
%             Rp(:, :, pid), Tp(:, pid), comfeatures);
%         Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
%         ntrigtms = Feature3D(comid, nTrianguTimes);
%         for(fid=1:nComPts)
%             Feature3D(comid(fid), (3+5*(ntrigtms(fid)-1)):(7+5*(ntrigtms(fid)-1))) = ...
%                 [pid-1, pid, -p3d(fid, :)]; 
%         end
        
    end        
end


