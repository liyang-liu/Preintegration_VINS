function [Rcam, Acam, Tcam, Feature3D] = fn_GetPosesFromIMUdata( nFrames, nPts, dtIMU, dp, dv, dphi, ...
                    K, RptFeatureObs, SLAM_Params)
 %fn_GetPosesFromIMUdata(dtIMU, g0, dp, dv, dphi,nFrames, nPts, ...
 %                    K, bSimData, bMalaga, RptFeatureObs, Tu2c, Ru2c)
                
    global PreIntegration_options
    
    %% For each image frame, Compute camera's R, T & Euler angles, in first camera's frame, i.e. Rcam, Tcam, Acam    
    % This is done via IMU's R, T at each frame time.
    Timu = zeros(3, nFrames);
    vimu = zeros(3, nFrames);
    if(PreIntegration_options.bMalaga == 1)
        vimu(:,1) = [ 3.214028; 0.18041; -0.0637 ];%[3.2299;0.1264;-0.0466];
    end
    
    Aimu = zeros(3, nFrames);
    Rimu = zeros(3, 3, nFrames);
    Rimu(:, :, 1) = eye(3);
    Tcam = zeros(3, nFrames);
    vcam = zeros(3, nFrames);
    Acam = zeros(3, nFrames);
    Rcam = zeros(3, 3, nFrames);
    
    frm = 1;    
    Tcam(:, frm) = SLAM_Params.Ru2c * ( Timu(:, frm) - SLAM_Params.Tu2c + (Rimu(:,:,frm))' * SLAM_Params.Tu2c );
    Rcam(:,:,frm) = SLAM_Params.Ru2c * Rimu(:,:,frm) * SLAM_Params.Ru2c';
    [ Acam(1, frm), Acam(2, frm), Acam(3, frm) ] = fn_ABGFromR( Rcam(:,:,frm) );    
    
    for(frm = 2:nFrames)
       vimu(:, frm) = vimu(:, frm-1) + dtIMU(frm) * SLAM_Params.g0 + (Rimu(:,:,frm-1))' * dv(:,frm);
       Timu(:, frm) = Timu(:, frm-1) ...
                        + 0.5 * dtIMU(frm) * ( vimu(:, frm-1) + vimu(:, frm) ) ... %You-Bing might have made a mistake here !!!
                        ...%+ dtIMU(frm) * vimu(:, frm)
                        + 0.5 * dtIMU(frm) * dtIMU(frm) * SLAM_Params.g0 ...
                        + (Rimu(:,:,frm-1))' * dp(:, frm);       
       dR = fn_RFromABG( dphi(1, frm), dphi(2, frm), dphi(3, frm) );
       Rimu(:, :, frm) = dR * Rimu( :, :, frm-1 );
       [Aimu(1, frm), Aimu(2, frm), Aimu(3, frm)] = fn_ABGFromR(Rimu(:,:,frm));
       Tcam(:, frm) = SLAM_Params.Ru2c * (Timu(:, frm) - SLAM_Params.Tu2c + (Rimu(:,:,frm))' * SLAM_Params.Tu2c );
       Rcam(:, :, frm) = SLAM_Params.Ru2c * Rimu(:,:,frm) * SLAM_Params.Ru2c';
       [ Acam(1, frm), Acam(2, frm), Acam(3, frm) ] = fn_ABGFromR( Rcam(:,:,frm) );
    end

    %% Get initial estimate for each feature's 3D position
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
        frms = [tv.pid]; % All frames where feature has occurred
        fpdis = Tcam(:, frms) - repmat( Tcam(:, frms(1)), 1, nObs ); %get each camera's position in 1st camera's frame
        fpdis = complex( fpdis(1,:), fpdis(2,:) );
        fpdis = abs( fpdis );
        [~, idx] = sort(fpdis);
        idx = idx(end);           % found frame with largest horizontal shift

        comid = RptFeatureObs(fid).fid;
        %%%

        pid1 = tv(1).pid;%1;%
        pid2 = tv(idx).pid;%2;%
        %%%

        comfeatures = [tv(1).uv, tv(idx).uv]';
        [p3d] = fn_TrianguFeatures( K, Rcam(:, :, pid1), Tcam(:, pid1), ...
                            Rcam(:, :, pid2), Tcam(:, pid2), comfeatures );

        Feature3D(comid).numTriangs = Feature3D(comid).numTriangs + 1;
        ntrigtms = Feature3D(comid).numTriangs;
        Feature3D(comid).triangs(ntrigtms) = ...
                            struct(  'pid1', pid1, ...
                                     'pid2', pid2, ...
                                     'p3D',  -p3d' ...
                            );             
    end  
end
                