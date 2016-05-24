function [Z_vec] = SLAM_Z_Object2Vector( Z_obj )

    global PreIntegration_options
    
    Z_vec = [];
    
    %
    % Z_obj definition
    % Z_Def = struct( ...
    %    'fObs',         repmat( UV_Def, nPts * nPoseNew, 1 ), ... % maybe redundant, will be cleaned
    %    'intlDelta',    repmat( IntlDelta_Def, nPoseNew-1, 1 ), ...
    %    'g',            Gravity_Def
    %    'Au2c',         Angle_Def, ...
    %    'Tu2c',         Trans_Def, ...
    %    'Bf',           Bf_Def, ...
    %    'Bw',           Bw_Def ...
    % );
    
    %% UV
    numUV = length(Z_obj.fObs);
    for i=1:numUV
        Z_vec = [Z_vec; Z_obj.fObs(i).uv(:)];
    end
    
    if ( PreIntegration_options.bPreInt == 1 )
        %% inertial Detla
        %
        % IntlDelta_Def = struct ( ...
        %    'deltaP',   DeltaP_Def, ...
        %    'deltaV',   DeltaV_Def, ...
        %    'deltaPhi', DeltaPhi_Def ...
        %    );

        numIntlDelta = length( Z_obj.intlDelta );
        for i=1:numIntlDelta
            Z_vec = [Z_vec; Z_obj.intlDelta(i).deltaP.val(:)];
            Z_vec = [Z_vec; Z_obj.intlDelta(i).deltaV.val(:)];
            Z_vec = [Z_vec; Z_obj.intlDelta(i).deltaPhi.val(:)];
        end
    else
        numImu = length( Z_obj.imu );
        for i = 1:numImu
            Z_vec = [Z_vec; Z_obj.imu(i).w.val(:)];
            Z_vec = [Z_vec; Z_obj.imu(i).acc.val(:)];
            Z_vec = [Z_vec; Z_obj.imu(i).deltaT.val(:)];
        end
    end
    
    if ( PreIntegration_options.bAddZg == 1 )
        Z_vec = [Z_vec; Z_obj.g.val(:) ];
    end
    
    %% Au2c, Tu2c, Bf, Bw
    Z_vec = [Z_vec; Z_obj.Au2c.val(:) ];
    
    Z_vec = [Z_vec; Z_obj.Tu2c.val(:) ];    
    
    Z_vec = [Z_vec; Z_obj.Bf.val(:); ];
    
    Z_vec = [Z_vec; Z_obj.Bw.val(:); ];
    