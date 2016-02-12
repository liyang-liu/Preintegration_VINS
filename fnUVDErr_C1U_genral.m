function [e, nUV] = fnUVDErr_C1U_genral(RptFeatureObs, K, x, Zobs, nPoses, nPts, ImuTimestamps)

    global InertialDelta_options

    nObsId_FeatureObs = 2;
    fx = K(1,1); cx0 = K(1,3); fy = K(2,2); cy0 = K(2,3);

    if((InertialDelta_options.bUVonly == 1) || (InertialDelta_options.bPreInt == 1))
        idx = (nPoses-1)*6;
    else
        nIMUdata = ImuTimestamps(nPoses)-ImuTimestamps(1);
        idx = (nIMUdata)*6;%(nPoses-1)*nIMUrate
    end
    
    p3d0 = reshape(x((idx+1):(idx+3*nPts), 1), 3, []);
    e = Zobs;
    
    % index of Au2c, Tu2c
    if(InertialDelta_options.bUVonly == 1)
        idx = (nPoses-1)*6+nPts*3;
    else
        if(InertialDelta_options.bPreInt == 1)
            idx = ((nPoses-1)*6+nPts*3+3*nPoses+3);%(nPoses-1)*6+nPts*3+3*nPoses + 9
        else
            idx = (nIMUdata)*6+3*nPts+3*(nIMUdata+1)+3;%(nPoses-1)*nIMUrate ((nPoses-1)*nIMUrate+1)
        end
    end
    
    alpha = x(idx+1);beta = x(idx + 2); gamma = x(idx + 3);
    Ru2c = fnR5ABG(alpha, beta, gamma);
    Tu2c = x((idx+4):(idx+6));
    nUV = 0;
    % Reprojection at each pose
    nfs = size(RptFeatureObs,1);
    for(fid=1:nfs)
        nObs = RptFeatureObs(fid, nObsId_FeatureObs);
        for(oid=1:nObs)
            pid = (RptFeatureObs(fid, oid*3))'; 
            if(pid > nPoses)
                break;
            elseif(pid > 1)
                if((InertialDelta_options.bUVonly == 1) ||(InertialDelta_options.bPreInt == 1))
                    idx = (pid-2)*6;
                else
                    idx = (ImuTimestamps(pid)-ImuTimestamps(1)-1)*6;%  (pid-1)*nIMUrate -6???
                end
                alpha = x(1+idx); beta = x(2+idx); gamma = x(3+idx); 
                Tu = x((4+idx):(idx+6), 1);
                Ru = fnR5ABG(alpha, beta, gamma);%Rx(alpha) * fRy (beta) * fRz(gamma);
            else % Pose 1 is special
                Tu = zeros(3,1); 
                Ru = eye(3);
            end
            Rc = Ru2c * Ru;
            Tc = Tu + (Ru)' * Tu2c;
            ncurrentObsdPts = 1;%size(obsfeatures{pid}, 1);    
            p3d1 = Rc * (p3d0(:, fid) - repmat(Tc, 1, ncurrentObsdPts));
        %     p3d1 = Rc * (p3d0(:, obsfeatures{pid}(:,1))) - repmat(Tc, 1, ncurrentObsdPts);
            u1 = fx * p3d1(1, :) ./ p3d1(3, :) + repmat(cx0, 1, ncurrentObsdPts);
            v1 = fy * p3d1(2, :) ./ p3d1(3, :) + repmat(cy0, 1, ncurrentObsdPts);
        %     d1 = p3d1(3, :);
        %     et = [u1;v1;d1];
            et = [u1;v1];
            et = et(:);%nPts*2*(pid-1)%nPts*2*pid
            e((nUV+1):(nUV+2*ncurrentObsdPts), 1) = et - Zobs((nUV+1):(nUV+2*ncurrentObsdPts), 1);    
            nUV = nUV + 2*ncurrentObsdPts;
        %     e((N*3*(pid-1)+1):(N*3*pid), 1) = et - e((N*3*(pid-1)+1):(N*3*pid), 1);
        end
    end
    