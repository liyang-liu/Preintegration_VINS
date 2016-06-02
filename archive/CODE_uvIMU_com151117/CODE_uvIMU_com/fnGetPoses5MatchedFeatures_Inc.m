function [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5MatchedFeatures_Inc(...
    nPoseOld, nPoses, R0imu, T0imu, K, fscaleGT, Feature3D, RptFeatureObs, ...
    Tu2c, Ru2c)
    %, bAddInitialNoise, sigmauov)

%     nPoses = nPoseNew - nPoseOld;
    Acam = zeros(3, nPoses);
    Tcam = zeros(3, nPoses);
    Rcam = zeros(3, 3, nPoses);
    
    Tcam(:, 1) = Ru2c*(T0imu - Tu2c + (R0imu)'*Tu2c);
    Rcam(:, :, 1) = Ru2c*R0imu*Ru2c';

    %[fid, nObs, [pidi, pidi1, xi, yi, zi]] %fId_FeatureObs = 1; 
    nTrianguTimes = 2;

    nRptfs = size(RptFeatureObs, 1);
    for(pid=2:(nPoses+1))% (nPoseOld+1):nPoseNew) 
        overallpid = pid + nPoseOld - 1;
        comfeatures = [];
        comid = [];        
        for(fid=1:nRptfs)
            obsId = 1;
            while((RptFeatureObs(fid, 3*obsId) ~= 0) && (RptFeatureObs(fid, 3*obsId) < overallpid-1))
                obsId = obsId + 1;
            end
            if((RptFeatureObs(fid, 3*obsId) == (overallpid-1)) && ...
                    (RptFeatureObs(fid, 3*(obsId+1)) == overallpid))
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
        [Ri, Ti, ~] = FuncComputeRT(comfeatures, K);
        fscale = fscaleGT(overallpid)/norm(Ti);
        Ti = fscale * Ti;
        %Tp(:, pid) = Tp(:, pid-1) + R'*Ti;
%         R1 = Ri*R0;
        Rcam(:, :, pid) = Ri*Rcam(:, :, pid-1);
        [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = ...
            fnABG5R(Rcam(:, :, pid));
%         T1 = T0 - R1'*Ti;  
        Tcam(:, pid) = Tcam(:, pid-1) - (Rcam(:, :, pid))'*Ti;
        nComPts = size(comid, 1);
        [p3d] = fnTrianguFeatures(K, Rcam(:,:, pid-1), Tcam(:, pid-1), ...
            Rcam(:,:, pid), Tcam(:,:, pid), comfeatures);
        Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
        ntrigtms = Feature3D(comid, nTrianguTimes);
        for(fid=1:nComPts)
            Feature3D(comid(fid), (3+5*(ntrigtms(fid)-1)):(7+5*(ntrigtms(fid)-1))) = ...
                [overallpid-1, overallpid, -p3d(fid, :)]; 
        end
%         T0 = T1; R0 = R1;
    end        
end


