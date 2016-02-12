function [FeatureObs] = fnCollectfObs5Imgs( kfids, pid, imgdir, sigma_uov_real, FeatureObs, nObsId_FeatureObs)
    global InertialDelta_options

            if(InertialDelta_options.bMalaga == 1)
                load(sprintf('%sImage%d.mat', imgdir, pid));
%                 obsfeatures{pid} = Image(2:end, 1:3);
                fidset = Image(2:end, 1);
            elseif(InertialDelta_options.bDinuka == 1)
                imid = kfids(pid);%1+(pid-1)*kfspan);
                load(sprintf('%simage_%d.mat', imgdir, imid));
%                 obsfeatures{pid} = features;
                fidset = features(:, 1);            
            end

            FeatureObs(fidset, nObsId_FeatureObs) = ...
                FeatureObs(fidset, nObsId_FeatureObs) + 1;
            for(fid=1:size(fidset,1))
                nObs = FeatureObs(fidset(fid), nObsId_FeatureObs);
                FeatureObs(fidset(fid), 3*nObs) = pid;
                if(InertialDelta_options.bMalaga == 1)
                    FeatureObs(fidset(fid), (3*nObs+1):(3*nObs+2)) = Image(fid+1, 2:3);
                elseif(InertialDelta_options.bDinuka == 1)
                    FeatureObs(fidset(fid), (3*nObs+1):(3*nObs+2)) = features(fid, 2:3) + ...
                        fnGenGaussNoise(1, 2, sigma_uov_real);                
                end
            end