function [Rcam, Acam, Tcam, Feature3D] = fn_GetPosesFromIMUdata( nPoses, nPts, dtIMU, dp, dv, dphi, ...
                    K, RptFeatureObs, SLAM_Params)
 %fn_GetPosesFromIMUdata(dtIMU, g0, dp, dv, dphi,nPoses, nPts, ...
 %                    K, bSimData, bMalaga, RptFeatureObs, Tu2c, Ru2c)
                
    global PreIntegration_options
                
    Timu = zeros(3,nPoses);
    vimu = zeros(3,nPoses);
    if(PreIntegration_options.bMalaga == 1)
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
    
    Tcam(:, pid) = SLAM_Params.Ru2c * (Timu(:, pid) - SLAM_Params.Tu2c + (Rimu(:,:,pid))' * SLAM_Params.Tu2c);
    Rcam(:,:,pid) = SLAM_Params.Ru2c * Rimu(:,:,pid) * SLAM_Params.Ru2c';
    [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = fn_ABGFromR( Rcam(:,:,pid) );    
    
    for(pid = 2:nPoses)
       vimu(:, pid) = vimu(:, pid-1) + dtIMU(pid) * SLAM_Params.g0 + (Rimu(:,:,pid-1))' * dv(:,pid);
       Timu(:, pid) = Timu(:, pid-1) + 0.5 * dtIMU(pid) * (vimu(:, pid-1) ...
                       + vimu(:, pid)) + 0.5 * dtIMU(pid) * dtIMU(pid) * SLAM_Params.g0 ...
                       + (Rimu(:,:,pid-1))' * dp(:, pid);       
       dR = fn_RFromABG(dphi(1, pid), dphi(2, pid), dphi(3, pid));
       Rimu(:,:,pid) = dR * Rimu(:,:,pid-1);
       [Aimu(1, pid), Aimu(2, pid), Aimu(3, pid)] = fn_ABGFromR(Rimu(:,:,pid));
       Tcam(:, pid) = SLAM_Params.Ru2c * (Timu(:, pid) - SLAM_Params.Tu2c + (Rimu(:,:,pid))' * SLAM_Params.Tu2c );
       Rcam(:,:,pid) = SLAM_Params.Ru2c * Rimu(:,:,pid) * SLAM_Params.Ru2c';
       [Acam(1, pid), Acam(2, pid), Acam(3, pid)] = fn_ABGFromR( Rcam(:,:,pid) );
    end

    if( PreIntegration_options.bSimData == 0 )
        nPts = 60000;
    end
    
    %% Define data structure for Feature3D
    TriangulateInfo_Def = struct( ...
        'pid1', [], ...
        'pid2', [], ...
        'p3D',  zeros(3,1) ...  % 3D co-ordinates (x, y, z)
        );
    Feature3DInfo_Def = struct( ...
        'fid',  [], ...
        'numTriangs', 0, ...
        'triangs', TriangulateInfo_Def ... % is array of triangulates, size will grow
        );
    Feature3D = repmat( Feature3DInfo_Def, nPts, 1);
    fids = num2cell( 1:nPts );
    [Feature3D(:).fid] = fids{:};        
    
    nRptfs = size(RptFeatureObs, 1);
    % Select poses to triangulate features: distant frames, maximal changes
    % in UVs
    for(fid=1:nRptfs)
        %%%%%%%%%%%%%%%%% According to pose distance
        nObs = RptFeatureObs(fid).nObs;
        tv = RptFeatureObs(fid).obsv; 
        %%%%%
        pids = [tv.pid];
        fpdis = Tcam(:, pids) - repmat( Tcam(:, pids(1)), 1, nObs );
        fpdis = complex(fpdis(1,:), fpdis(2,:));
        fpdis = abs(fpdis);
        [~, idx] = sort(fpdis);
        idx = idx(end);

        comid = RptFeatureObs(fid).fid;
        %%%

        pid1 = tv(1).pid;%1;%
        pid2 = tv(idx).pid;%2;%
        %%%

        comfeatures = [tv(1).uv, tv(idx).uv]';
        [p3d] = fn_TrianguFeatures(K, Rcam(:, :, pid1), Tcam(:, pid1), ...
                            Rcam(:, :, pid2), Tcam(:, pid2), comfeatures);

        Feature3D(comid).numTriangs = Feature3D(comid).numTriangs + 1;
        ntrigtms = Feature3D(comid).numTriangs;
        Feature3D(comid).triangs(ntrigtms) = ...
                            struct(  'pid1', pid1, ...
                                     'pid2', pid2, ...
                                     'p3D',  -p3d' ...
                            );             
    end  
end
                