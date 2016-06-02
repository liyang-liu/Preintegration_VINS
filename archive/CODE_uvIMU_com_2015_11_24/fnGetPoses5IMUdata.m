function [Rcam, Acam, Tcam, Feature3D] = fnGetPoses5IMUdata(dtIMU, g0, dp, dv, dphi,nPoses, nPts, ...
                    K, bSimData, bMalaga, RptFeatureObs, Tu2c, Ru2c)
                
    Timu = zeros(3,nPoses);
    vimu = zeros(3,nPoses);
    if(bMalaga == 1)
        vimu(:,1) = [3.214028;0.18041;-0.0637];%[3.2299;0.1264;-0.0466];
    end
    Aimu = zeros(3,nPoses);
    Rimu = zeros(3,3,nPoses);
    Rimu(:,:,1) = eye(3);
    Tcam = zeros(3,nPoses);
    vcam = zeros(3,nPoses);
    Acam = zeros(3,nPoses);
    Rcam = zeros(3,3,nPoses);
    pid = 1;
    Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + (Rimu(:,:,pid))'*Tu2c);
    Rcam(:,:,pid) = Ru2c*Rimu(:,:,pid)*Ru2c';
    [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = fnABG5R(Rcam(:,:,pid));    
    
    for(pid = 2:nPoses)
       vimu(:, pid) = vimu(:, pid-1) + dtIMU(pid)*g0 + (Rimu(:,:,pid-1))'*dv(:,pid);
       Timu(:, pid) = Timu(:, pid-1) + 0.5*dtIMU(pid)*(vimu(:, pid-1)+...
           vimu(:, pid))+0.5*dtIMU(pid)...
           *dtIMU(pid)*g0+(Rimu(:,:,pid-1))'*dp(:, pid);       
       dR = fnR5ABG(dphi(1, pid), dphi(2, pid), dphi(3, pid));
       Rimu(:,:,pid) = dR * Rimu(:,:,pid-1);
       [Aimu(1, pid), Aimu(2, pid), Aimu(3, pid)] = fnABG5R(Rimu(:,:,pid));
       Tcam(:, pid) = Ru2c*(Timu(:, pid) - Tu2c + (Rimu(:,:,pid))'*Tu2c);
       Rcam(:,:,pid) = Ru2c*Rimu(:,:,pid)*Ru2c';
       [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = fnABG5R(Rcam(:,:,pid));
    end

    if(bSimData == 0)
        nPts = 60000;
    end
    Feature3D = zeros(nPts, 50);%[fid, nObs, [pidi, pidi1, xi, yi, zi]]
    fId_FeatureObs = 1; nTrianguTimes = 2;
    Feature3D(:, fId_FeatureObs) = 1:nPts;

    nRptfs = size(RptFeatureObs, 1);
    % Select poses to triangulate features: distant frames, maximal changes
    % in UVs
    for(fid=1:nRptfs)
%        nObs = RptFeatureObs(fid, 2);
%        tv = (RptFeatureObs(fid, 3:(3*nObs+2)))'; 
%        tv = reshape(tv, 3, []);
%        fuvdis = tv(2:3,:) - repmat(tv(2:3,1), 1, nObs);
%        fuvdis = complex(fuvdis(1,:), fuvdis(2,:));
%        fuvdis = abs(fuvdis);
%        [~, idx] = sort(fuvdis);
%        idx = idx(end);
%        comid = RptFeatureObs(fid, fId_FeatureObs);
%        pid1 = tv(1,1);
%        pid2 = tv(1,idx);
%        comfeatures = [tv(2:3,1); tv(2:3,idx)];
%        [p3d1] = fnTrianguFeatures(K, Rcam(:, :, pid1), Tcam(:, pid1), ...
%             Rcam(:, :, pid2), Tcam(:, pid2), comfeatures);
%         Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
%         ntrigtms = Feature3D(comid, nTrianguTimes);
%         Feature3D(comid, (3+5*(ntrigtms-1)):(7+5*(ntrigtms-1))) = ...
%             [pid1, pid2, -p3d];
        %%%%%%%%%%%%%%%%% According to pose distance
       nObs = RptFeatureObs(fid, 2);
       tv = (RptFeatureObs(fid, 3:(3*nObs+2)))'; 
       tv = reshape(tv, 3, []);
       pids = tv(1,:);
       fpdis = Tcam(:, pids) - repmat(Tcam(:, pids(1)), 1, nObs);
       fpdis = complex(fpdis(1,:), fpdis(2,:));
       fpdis = abs(fpdis);
       [~, idx] = sort(fpdis);
       idx = idx(end);
       comid = RptFeatureObs(fid, fId_FeatureObs);
       pid1 = tv(1,1);
       pid2 = tv(1,idx);
       comfeatures = [tv(2:3,1); tv(2:3,idx)];
       [p3d] = fnTrianguFeatures(K, Rcam(:, :, pid1), Tcam(:, pid1), ...
            Rcam(:, :, pid2), Tcam(:, pid2), comfeatures);
%         if(norm(p3d) > 1e3)%p3d2-p3d1
%             continue;
%         end
        Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
        ntrigtms = Feature3D(comid, nTrianguTimes);
        Feature3D(comid, (3+5*(ntrigtms-1)):(7+5*(ntrigtms-1))) = ...
            [pid1, pid2, -p3d];       
    end  
%%%%%%%%%%%%%%%%%%%%%%%%        
%     for(pid=2:nPoses) 
%         comfeatures = [];
%         comid = [];        
%         for(fid=1:nRptfs)
%             obsId = 1;
%             while((RptFeatureObs(fid, 3*obsId) ~= 0) && (RptFeatureObs(fid, 3*obsId) < pid-1))
%                 obsId = obsId + 1;
%             end
%             if((RptFeatureObs(fid, 3*obsId) == (pid-1)) && ...
%                     (RptFeatureObs(fid, 3*(obsId+1)) == pid))
%                 comfeatures = [comfeatures; RptFeatureObs(fid, (3*obsId+1):(3*obsId+2)), ...
%                     RptFeatureObs(fid, (3*obsId+4):(3*obsId+5))];
%                 comid = [comid; RptFeatureObs(fid, 1)];
%             end
%         end
%         comfeatures = comfeatures';
% %         %%%
% %         nfs = size(comfeatures, 2);
% %         X1 = [comfeatures(1:2, :); ones(1, nfs)];
% %         X2 = [comfeatures(3:4, :); ones(1, nfs)];        
% %         [~, bestmodel] = ransacF(X1,X2);
% %         comfeatures = comfeatures(:, bestmodel);
% %         comid =  comid(bestmodel);
% %         %%%
%   
%         nComPts = size(comid, 1);
%         [p3d] = fnTrianguFeatures(K, Rcam(:, :, pid-1), Tcam(:, pid-1), ...
%             Rcam(:, :, pid), Tcam(:, pid), comfeatures);
%         Feature3D(comid, nTrianguTimes) = Feature3D(comid, nTrianguTimes) + 1;
%         ntrigtms = Feature3D(comid, nTrianguTimes);
%         for(fid=1:nComPts)
%             Feature3D(comid(fid), (3+5*(ntrigtms(fid)-1)):(7+5*(ntrigtms(fid)-1))) = ...
%                 [pid-1, pid, -p3d(fid, :)]; 
%         end
%         
%     end        
end
                