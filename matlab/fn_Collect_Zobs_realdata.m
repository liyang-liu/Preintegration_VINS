function Zobs = fn_Collect_Zobs_realdata( RptFeatureObs, Xg_obj, dataIMU, nPoses, nPts, nIMUrate, dp, dv, dphi, SLAM_Params )

    global PreIntegration_options

    % Firstly allocate a maximal space.
    if(PreIntegration_options.bMalaga == 1)
        z2 = Xg_obj.pose(1).trans.xyz(1); % Timu(:,4) correspond to Tcam(:,6)==> x-z
    elseif(PreIntegration_options.bDinuka == 1)
        z2 = Xg_obj.pose(1).trans.xyz(3);
    end 
    
    Zobs = SLAM_Z_Define( nPoses, nPts, nIMUrate );

    % Order UVs according to fid
    zid = 0;
    zrow = 0;

    for(fid=1:nPts)% local id
        nObs = RptFeatureObs(fid).nObs;

        for(oid=1:nObs)
            tv = RptFeatureObs(fid).obsv(oid).uv';
            zid = zid + 1;
            Zobs.fObs(zid).uv = tv(:);
            Zobs.fObs(zid).row = (1:2) + zrow;
            zrow = zrow + 2;
        end
    end
    Zobs.fObs(zid+1:end) = []; % delete unused spaces
    nUV = zrow; %num UV rows
        
        
    
    %%%%%%%%%%%%%%%%%
    if ( PreIntegration_options.bUVonly == 0 )
        %% Put IMU data into observation vector z:         
        if ( PreIntegration_options.bPreInt == 0 ) % Non-pre-integration: put raw data  
            idx_ImuObs = 0;
            for pid = 2 : nPoses
                ndata = size(dataIMU{pid}, 1);
                tv = [ dataIMU{pid}(:, 2:7 ), zeros(ndata, 3) ]';%wi,ai,0

                for j = 1 : ndata
                    idx_ImuObs = idx_ImuObs + 1;
                    Zobs.imu(idx_ImuObs).w.val = tv(1:3, j); % linear accleartion observation
                    Zobs.imu(idx_ImuObs).w.row = (1:3) + zrow; zrow = zrow + 3;
                    Zobs.imu(idx_ImuObs).acc.val = tv(4:6, j); %angular rate observation
                    Zobs.imu(idx_ImuObs).acc.row = (1:3) + zrow; zrow = zrow + 3;
                    Zobs.imu(idx_ImuObs).deltaT.val = tv(7:9, j); % a constraint enforcing 0 = T(i+1,j) - T(i,j) - v(i,j)*deltaT, set to zero before for-loop
                    Zobs.imu(idx_ImuObs).deltaT.row = (1:3) + zrow; zrow = zrow + 3;

                end
            end
        else    
            % Add interated IMU observations
            if ( PreIntegration_options.bPerfectIMUdlt == 1 )
                [dp, dv, dphi] = ... %fn_CalPerfectIMUdlt(X_obj, nPoses, nPts, Jd, SLAM_Params );             
                                 fn_CalPerfectIMUdlt_general( X_obj, nPoses, nPts, Jd, dtIMU, SLAM_Params ); 
            end


            for p = 2 : nPoses
                Zobs.intlDelta(p-1).deltaP.val = dp(:, p);
                Zobs.intlDelta(p-1).deltaP.row = (1:3) + zrow;
                zrow = zrow + 3;

                Zobs.intlDelta(p-1).deltaV.val = dv(:, p);
                Zobs.intlDelta(p-1).deltaV.row = (1:3) + zrow;
                zrow = zrow + 3;

                Zobs.intlDelta(p-1).deltaPhi.val = dphi(:, p);
                Zobs.intlDelta(p-1).deltaPhi.row = (1:3) + zrow;
                zrow = zrow + 3;
            end
        end
    else

        %         %%%%%%%
        %         pid = 2;
        %         Rcam = fnR5ABG(ABGcam(1,pid), ABGcam(2,pid), ABGcam(3,pid));
        %         Rimu = Ru2c'*Rcam*Ru2c;
        %         [ABGimu(1,pid), ABGimu(2,pid), ABGimu(3,pid)] = fnABG5R(Rimu);
        %         Timu(:, pid) = Tu2c + Ru2c'*Tcam(:, pid) - Rimu'*Tu2c;                
        Zobs((utid+1)) = z2;%Zobs((utid+1):(utid+6)) = x(1:6);
        utid = utid + 1; %6       
    
    end
    
    if(PreIntegration_options.bAddZg == 1)
        % Add pseudo observation of g        
        Zobs.g.val = SLAM_Params.g0;
        Zobs.g.row = (1:3) + zrow;
        zrow = zrow + 3;                    
    end
    
    if(PreIntegration_options.bAddZau2c == 1)
        % Add pseudo observation of Tu2c
        [alpha, beta, gamma] = fn_ABGFromR( SLAM_Params.Ru2c );
        Zobs.Au2c.val = [alpha; beta; gamma];
        Zobs.Au2c.row = (1:3) + zrow;
        zrow = zrow + 3;        
    end
    
    if(PreIntegration_options.bAddZtu2c == 1)
        % Add pseudo observation of Tu2c
        Zobs.Tu2c.val = SLAM_Params.Tu2c;
        Zobs.Tu2c.row = (1:3) + zrow;
        zrow = zrow + 3;
        
    end
    
    if ( PreIntegration_options.bVarBias == 0 )
        
        if(PreIntegration_options.bAddZbf == 1)
            % Add pseudo observation of bf
            Zobs.Bf.val = SLAM_Params.bf0;
            Zobs.Bf.row = (1:3) + zrow;
            zrow = zrow + 3;

        end
        if(PreIntegration_options.bAddZbw == 1)
            % Add pseudo observation of bf
            Zobs.Bw.val = SLAM_Params.bw0;
            Zobs.Bw.row = (1:3) + zrow;
            zrow = zrow + 3;
        end

    end