function [idx, npmax, ng, ginlierids] = funSeg3D(np, points, dp, ...
                        nminp, p3d1, p3d2, grouppids, epsf, fthreshv3d, fthreshdis)

    m = 0;
    idx = 0;
    npmax = 0;
    ng = 0;
    ginlierids = [];
    mgpids = [];
    gid = 1; 
    v3dsim = zeros(1, np);
    %% 1. Find inlier patch for each point
    while(m < np)
        m = m + 1;
%         if(~isempty(mgpids))
%             while((m < np) && (any(mgpids(:, m))))
%                 m = m + 1;
%             end
%         else
% %             m = m + 1;
%         end
        for(pi = 1: points)%
%             v3dsim(pi) = norm(dp(:, (pi)) - dp(:, (m)))/(norm(p3d1(:,pi)-p3d1(:,m))*(norm(dp(:, (m))) + epsf));
%             v3dsim(pi) = norm(dp(:, (pi)) - dp(:, (m)))/((norm(dp(:, (m))) + epsf));
            v3dsim(pi) = norm(dp(:, (pi)) - dp(:, (m)))/(.5* (norm(dp(:, (pi))) + norm(dp(:, (m)))) + epsf);
        end       
        smid = find(v3dsim < fthreshv3d); % Look for points with similar vectors
        if(length(smid) < nminp) % Outliers?
                continue;
        end

        inliers = smid; % Based on 3D flow only, superficial!
        % Get the first R/T model related inliers, and this is the real starting point!
        [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, inliers, np, nminp, fthreshdis);
        if(isempty(T))
            continue;
%         elseif(length(inliers) <= nminp)
%             [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, inliers, np, nminp, fthreshdis+(1e-3));
        end
        
        bestInliers = inliers;        
%         bestInliers = [];
%         while((length(inliers) > nminp) && (length(bestInliers) < length(inliers)))
%             bestInliers = inliers;
%             [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, inliers, np, nminp, fthreshdis);
%         end

        %%%%%%%%%%%
        nlocopt = 0;
        nmaxiteration = 10;%20;%100;%5;%50;%
        npts = length(bestInliers);
        if(npts > 5)%4)%6)%10)%nminp
            nresamp = 5;%4;%floor(npts/3);% 6;%10;%       
            bestInliers_0 = bestInliers;
            while(nlocopt < nmaxiteration)
                nlocopt = nlocopt + 1;
                if ~exist('randsample', 'file')
                    ind = randomsample(npts, nresamp);
                else
                    ind = randsample(npts, nresamp);
                end
                [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, bestInliers_0(ind), np, nminp, fthreshdis);
                if(length(bestInliers) < length(inliers))
                    bestInliers = inliers;
                end
            end
        end
%         npts1 = length(bestInliers)
        if((isempty(T)) || (length(bestInliers) <= nminp))
%         if((isempty(T)) || (length(inliers) <= nminp))
            continue;
        end  
        allpid = zeros(1, np);
        allpid(bestInliers) = 1;
        grouppids{gid} = allpid; 
        mgpids(gid, :) = allpid;
        gid = gid + 1;
%         if(length(bestInliers) < 0.8*points)%/2)
%             inlierIds{m} = bestInliers;
%             ninliers(m) = length(bestInliers);
%         else
%             grouppointid = bestInliers;
%             break;
%         end
        
    end
    
    if(isempty(grouppids))
        warning('Too few inliers!');
        T = [];
        return;
    end    
    ng = size(grouppids, 2);
    %% 2. Cluster groups having intersections
    for(gi = 1:ng)
        ci = grouppids{gi};
        ci = find(ci > 0);
        for(gj=(gi+1):(ng))
            cj = grouppids{gj};
            cj = find(cj > 0);
            cincj = intersect(ci,cj);
            if((length(cincj)/(length(ci)) > 0.5) || (length(cincj)/(length(cj)) > 0.5))
                pcincj = union(ci, cj);
                allpid = zeros(1, np);                
                allpid(pcincj) = 1;
                grouppids{gj} = allpid; % Cluster to gj
                grouppids{gi} = [];% Make gi empty
                break; % Finish looking for a cluster for gi
            end
        end        
    end
    
    grouppids = grouppids(~cellfun('isempty', grouppids));
    ng = size(grouppids, 2);    
    
    ginlierids = cell(ng, 1);
 % The model can cover the most-- sub-optimal solution
    for(gi = 1:ng)
        ginlierids{gi} = find(grouppids{gi} > 0);
        ngpi = length(ginlierids{gi});
        if(ngpi > npmax)
            idx = gi;
            npmax = ngpi;
        end
    end