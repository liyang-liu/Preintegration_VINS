function Zobs = InertialDelta_InitZ( Zobs, RptFeatureObs, nPoseNew, nPts, dp, dv, dphi, SLAM_Params )
    global PreIntegration_options
    
    %% camera observations (u,v)
    %zidend = 0; 
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
    Zobs.fObs(zid+1:end) = [];
    nUV = zid*2; %size(Zobs.fObs(1).uv, 1)

    %%%%%%%%%%%%%%%%%
    %utid = zidend;
    %idr = zidend;    
    if(PreIntegration_options.bUVonly == 0)
        %% Put IMU data into observation vector z: 
        if(PreIntegration_options.bPreInt == 0) % Non-pre-integration: put raw data  
            for pid=2:nPoseNew
                ndata = size(dataIMU{pid}, 1);
                tv = [dataIMU{pid}(:,2:7), zeros(ndata,3)]';%wi,ai,0
                Zobs((idr+1):(idr+ndata*9)) = tv(:);
                idr = idr+ndata*9;
            end 
            utid = idr;% + (nPoses - 1)*nlenpp;
        else    
            % Add interated IMU observations
            if(PreIntegration_options.bPerfectIMUdlt == 0)
                %Zobs((idr+1):(idr+9*(nPoseNew-1)),1) = reshape([dp(:,2:nPoseNew);dv(:,2:nPoseNew);dphi(:,2:nPoseNew)],[],1);
                for p = 2 : nPoseNew
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
            else
                %dt = 1;
                %Zobs((idr+1):(idr+9*(nPoseNew-1)),1) = fnCalPerfectIMUdlt_general(x, nPoseNew, nPts, Jd, dtIMU, bf0, bw0); 
                Zobs.intlDelta = fnCalPerfectIMUdlt_general(X_obj, nPoseNew, nPts, Jd, dtIMU, bf0, bw0); 
            end
            %utid = idr + (nPoseNew - 1)*9;
        end 

     %% Continue filling in Zobs with psedu observations related to IMU
        if(PreIntegration_options.bAddZg == 1)
            % Add pseudo observation of g        
            %Zobs((utid+1):(utid+3)) = g0; 
            %utid = utid + 3;
            Zobs.g.val = g0;
            Zobs.g.row = (1:3) + zrow;
            zrow = zrow + 3;
            
        end
    end
    if(PreIntegration_options.bAddZau2c == 1)
        % Add pseudo observation of Tu2c
        [alpha, beta, gamma] = fnABGFromR(SLAM_Params.Ru2c);
        %Zobs((utid+1):(utid+3)) = [alpha;beta;gamma];
        %utid = utid + 3;
        Zobs.Au2c.val = [alpha; beta; gamma];
        Zobs.Au2c.row = (1:3) + zrow;
        zrow = zrow + 3;
    end
    if(PreIntegration_options.bAddZtu2c == 1)
        % Add pseudo observation of Tu2c
        %Zobs((utid+1):(utid+3)) = Tu2c;
        %utid = utid + 3;
        Zobs.Tu2c.val = SLAM_Params.Tu2c;
        Zobs.Tu2c.row = (1:3) + zrow;
        zrow = zrow + 3;
    end

    if(PreIntegration_options.bUVonly == 1)% Add A2, T2 as additional observation
        Zobs((utid+1)) = z2;%Zobs((utid+1):(utid+6)) = x(1:6);
        utid = utid + 1; %6       
    else
        if(PreIntegration_options.bVarBias == 0)
            if(PreIntegration_options.bAddZbf == 1)
                % Add pseudo observation of bf
                %Zobs((utid+1):(utid+3)) = bf0; 
                %utid = utid + 3;            
                Zobs.Bf.val = SLAM_Params.bf0;
                Zobs.Bf.row = (1:3) + zrow;
                zrow = zrow + 3;
            end
            
            if(PreIntegration_options.bAddZbw == 1)
                % Add pseudo observation of bf
                %Zobs((utid+1):(utid+3)) = bw0; 
                %utid = utid + 3;            
                Zobs.Bw.val = SLAM_Params.bw0;
                Zobs.Bw.row = (1:3) + zrow;
                zrow = zrow + 3;
            end
        else
            for(pid=2:(nPoseNew-1))
               Zobs((utid+1):(utid+3)) = 0; %bfi-bfi1 = 0
               utid = utid + 3; 
               Zobs((utid+1):(utid+3)) = 0; %bwi-bwi1 = 0
               utid = utid + 3;               
            end
        end
    end
    %Zobs = [Zobs(1:utid)];%(idr+9*(nPoses-1))    
