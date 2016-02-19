function [grouppointid, R, T, npmax] = Motion3DSegment(I1, y, p3d1, p3d2, nminp, fthreshold, ...
                                    nminclustersize, colors, bDisplay, sinfo)

    grouppointid = [];
    R = [];
    T = [];    
    npmax = [];
    np = size(p3d1, 2);
    points = np;%size(y, 2);
    if(points < nminp)%20)
        warning('Too few points!');
        return;
    end
% %     elseif(points < 50)
% %         fthreshv3d = 0.04;
% %         fthreshdis = 8e-3;        
% %     elseif(points < 100)
% %         fthreshv3d = 0.02;
% %         fthreshdis = 6e-3;
%     else % 0.05/5e-3
        fthreshv3d = 0.05;%0.1;%0.01;%
        fthreshdis = 1e-1;%8e-2;%5e-2;%1e-2;%5e-1;%1;%5e-3;%8e-3;%4e-3;%2e-3;%     
%     end
    
    sinfo = sprintf('%sNp = %d, threshold = %f/%f', sinfo, points, fthreshv3d, fthreshdis);
    
    dy = y(:,:, 2) - y(:,:,1);
    pts(:,:,1) = y(1:2, :, 1);
    pts(:,:,2) = y(1:2, :, 2);
    
    dp = p3d2 - p3d1; % 3xn
    
    epsf = 1.0;    
    groupP = [];
    groupModel=[];
    grouppids = [];
    mgpids = [];
    
    m = 0;
    gid = 1; 
    v3dsim = zeros(1, np);
    %% 1. Find inlier patch for each point
    while(m < np)
        m = m + 1;
        if(~isempty(mgpids))
            while((m < np) && (any(mgpids(:, m))))
                m = m + 1;
            end
        else
%             m = m + 1;
        end
        for(pi = 1: points)
            v3dsim(pi) = norm(dp(:, (pi)) - dp(:, (m)))/(.5*(norm(dp(:, (pi))) + norm(dp(:, (m)))) + epsf);
        end       
        %smid = find(v3dsim < fthreshv3d); % Look for points with similar vectors
        %if(length(smid) < nminp) % Outliers?
        %        continue;
        %end
		[~, sids] = sort(v3dsim);
		smid = sids(1:nminp);

        inliers = smid; % Based on 3D flow only, superficial!
        % Get the first R/T model related inliers, and this is the real starting point!
        [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, inliers, np, nminp, fthreshdis);
        if(isempty(T))
            continue;
        end
        bestInliers = [];
        while((length(inliers) > nminp) && (length(bestInliers) < length(inliers)))
            bestInliers = inliers;
            [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, inliers, np, nminp, fthreshdis);
        end
        if((isempty(T)) || (length(inliers) <= nminp))
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
    
    idx = 0;
    npmax = 0;
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

    if(bDisplay)
        Displaygroups(I1, y, dy, 1, points, ginlierids, colors, sinfo, ng);
    end
    %npmax
    grouppointid = ginlierids{idx};
    %% Get the final R/T
    [rt] = Trans3D([p3d1(:, grouppointid); p3d2(:, grouppointid)]');
    R = rt(:, 1:3);
    T = rt(:, 4);      

    if(bDisplay)
        DisplayCurrentgroup(I1, y, dy, 1, points, grouppointid, colors, sinfo, 1);
    end
%     
%     [img, imask, msk2p] = getimage(y(1:2,:,1), y(1:2,:,2));
%     groups = floor(imask);
%     [nr, nc] = size(img);
%     gid = 0; % 1;
%     epsf = 1.0;
%     
%     groupPts{1} = zeros(1, points);
%     groupP = [];
%     groupModel=[];
%     m = 1;
%     while(1)
%         gid = gid + 1;
%         grouppointid{gid} = [];
%         m = m + 1;
%         while((m <= nr*nc) && (groups(m) ~= 1) && (groups(m) ~= -1))
%             m = m + 1;        
%         end
%         if(m > nr*nc)
%             break;
%         end
%         v = mod(m, nr);
%         u = 1 + floor(m/nr);
%         if(v == 0)
%             v = nr;
%             u = u - 1;
%         end        
% 
%         gpid = 1;
%         ng = 0;  
%         dr = 0;
%         while(ng < nminp)%3
%             dr = dr + 1;
%             if(dr > min(nr, nc)/5)
%                 break;
%             end
%             [nf, pos, nvalues, dr] = findneighbors(img, imask, v, u, dr);
%             ni = 1;            
%             gpos = [v, u];
%             grouppointid{gid} = [msk2p(v, u)];
%             ng = 1;
% %             bfindn = 0;
%             while(ni <= nf)                
%                 p1 = pos(ni,:);           
%                 a = norm(img(v,u) - img(p1(2), p1(1)))/(.5*(norm(img(v,u)) + norm(img(p1(2), p1(1)))) + epsf);
%                 if(a <= fthreshold)%abs(a-0.5) <= 0.1)%
%                     grouppointid{gid} = [grouppointid{gid}, msk2p(p1(2), p1(1))];
%                     gpos = [gpos;p1];% save for the next searching round
%                     ng = ng + 1;
%                 end
%                 ni = ni + 1;
%             end
%         end
%             if(size(gpos,1) >= nminp)
%                 p0 = pts(:, grouppointid{gid}(1), 1);
%                 p1 = pts(:, grouppointid{gid}(1), 2);
%                 [P, groupModel] = GetModel(nModelid, y, 1, 2, grouppointid, groupModel, gid, pts, p0, p1);
% 
%                     if(isempty(P))% Outliers
%                        gid = gid - 1;
%                        continue;
%                     else
%                         npt = points;
%                         allpi = 1:npt;
%                         if(nModelid == 2)
%                             nerr = Calpp2vErr_general(p0, p1, P, pts, epsf);
%                             Kh = ceil(0.2*npt);
%                             if(bAdaptive == 1)                            
%                                 [h] = IKOSE(nerr, Kh);   
%     %                             E = 1./5;%2;%3;%10;%10;%4;%2.5;% 6;%h = 1/2.5;%3.5
%                                 fthreshold1 = E * h;
%                             else
%     %                             fthreshold1 = 10;%20;%5;%h/E;%0.1%0.015;%                            
%                             end
%                         elseif((nModelid == 1) || (nModelid == 3))
%                             x1 = y(:, :, 1);
%                             x2 = y(:, :, 2);
%                             if(groupModel{gid} == 1)
%                                 d = calFundamentalMatrixErr(P, x1, x2);
%                             elseif(groupModel{gid} == 2)
%                                 d = calHomographyErr(P, x1,x2);
%                             end
%                             nerr = abs(d);
% %                             fthreshold1 = 200;%500;%2;%8e-4;%
%                         end
%                         inlieridx = find(nerr < fthreshold1);
%                         
%                         if(length(inlieridx) > nminclustersize)%ceil(0.1*npt)
%                                
%                         else
%                             gid = gid - 1;
%                             continue;
%                         end
%                         
%                         grouppointid{gid} = inlieridx;
%                         groupPts{gid} = zeros(1, npt);
%                         groupPts{gid}(inlieridx) = gid;
%                         groupP{gid} = P;
%                         groupPbase{gid} = [p0, p1];
%                         nl = length(inlieridx);
%                         for(t=1:nl)
%                             p=y(1:2, inlieridx(t),1);
%                             u = round(p(1));
%                             v = round(p(2));
%                             groups(v, u) = gid;             
%                         end
%                     end
%             else
%                gid = gid - 1;
%             end
% 
%        
%     end
% 
%     ntotalgroups = gid - 1;
%     
% %     % Cluster groups having intersections==> Least-square P
%     for(gi=2:ntotalgroups)
%         ci = grouppointid{gi};        
%         for(gj=(gi+1):(ntotalgroups+1))
%             cj = grouppointid{gj};
%             cincj = intersect(ci,cj);
%             if((length(cincj)/(length(ci)) > 0.5) || (length(cincj)/(length(cj)) > 0.5))
%                 grouppointid{gj} = union(ci, cj);
%                 groupPts{gj} = zeros(1, npt);
%                 groupPts{gj}(grouppointid{gj}) = gj; % Re-set the ownership
%                 [P, groupModel] = GetModel(nModelid, y, 1, 2, grouppointid, groupModel, gj, pts, groupPbase{gj}(:,1), groupPbase{gj}(:,2));
%                 groupP{gj} = P;
% 
%                 groupP{gi} = [];
%                 groupPts{gi} = [];
%                 grouppointid{gi} = [];
%                 nl = length(grouppointid{gj});
%                 for(t=1:nl)
%                     p=y(1:2, grouppointid{gj}(t),1);
%                     u = round(p(1));
%                     v = round(p(2));
%                     groups(v, u) = gj;             
%                 end
%                 break;
%             end
%         end
%     end
% 
% % Displaygroups(y, dy, i, points, grouppointid, colors, 'Groups after clustering', ntotalgroups);
% 
% %% Make sure each point only belong to one group
%   ngroups = length(groupP);
%   validgid = [];
%   for(ni=2:ngroups)
%       if(~isempty(groupP{ni}))
% %         gpts(ni,:) = [groupPts{ni}];
%         validgid = [validgid, ni];
%       end
%   end
% 
%     newgid = 1;
%     ntotalgroups = length(validgid);
%     if(ntotalgroups > 0)
%         for(gi =1: length(validgid))
%             grouppointid{gi} = grouppointid{validgid(gi)};
%             grouppointid{validgid(gi)} = [];
%         end
%         grouppointid = grouppointid(~cellfun('isempty', grouppointid));
%         groupP = groupP(~cellfun('isempty', groupP));
%         ng = size(groupP, 2);
%         if(bDisplay)
%             Displaygroups(y, dy, 1, points, grouppointid, colors, sinfo, ntotalgroups);
%         end
%     else
%         grouppointid = [];
%     end
    
