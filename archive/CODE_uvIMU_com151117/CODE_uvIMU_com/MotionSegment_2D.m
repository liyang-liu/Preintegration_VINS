function [F, biggestPtGroup] = MotionSegment_2D(y, nminp, fthreshold)

    addpath(genpath('Ransac'));
    grouppointid = [];
    npts = size(y, 2);
    X1 = y(:, :, 1);
    X2 = y(:, :, 2);
    dy = X2 - X1;% (dui, dvi)
    complexVar = complex(dy(1,:), dy(2, :));
    pts(:,:,1) = y(1:2, :, 1);
    pts(:,:,2) = y(1:2, :, 2);
    
    epsf = 1.0;
    
    groupPts{1} = zeros(1, npts);
%     groupP = [];
%     groupModel=[];
    m = 0;
    gid = 0;
    ngids = [];
    
    while(1)
        gid = gid + 1;
        grouppointid{gid} = [];
        m = m + 1;
        if(m > npts)
            break;
        end
        
        tv = repmat(complexVar(m),1,npts);
        a = abs(tv - complexVar) ./ (0.5*(abs(complexVar) + norm(tv)) + epsf);
        idv = find(a <= fthreshold);
        if(size(idv, 2) < nminp) 
            gid = gid - 1;
            continue;
        end
        
        %[~, ids] = sort(a);
        
        
        sX1 = y(:, idv, 1);%idvids(1:nminp) ids(1:nminp)
        sX2 = y(:, idv, 2);
        [inlierIds, Ft] = fnGetFundamentalMatrix_8p(sX1, sX2, X1, X2);
        if(~isempty(inlierIds))
            grouppointid{gid} = inlierIds;
            ngids(gid) = size(inlierIds, 2);
        else
            gid = gid - 1;
            continue;
        end
    end
    
    [~, maxGid] = max(ngids);
    biggestPtGroup = grouppointid{maxGid};
    F = eightpoint(X1(:,biggestPtGroup),X2(:,biggestPtGroup));
    
   
%         gpid = 1;
%         ng = 0;  
%         dr = 0;
%         while(ng < nminp)%3
%             dr = dr + 1;
%             if(dr > min(nr, nc)/5)
%                 break;
%             end
%             [nf, pos, ~, dr] = findneighbors(img, imask, v, u, dr, 2*nminp);
%             ni = 1;            
%             gpos = [v, u];
%             grouppointid{gid} = [msk2p(v, u)];
%             ng = 1;
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
% %                     if((nModelid == 1) || (nModelid == 3))
% %                         x1 = y(:, grouppointid{gid}, i);
% %                         x2 = y(:, grouppointid{gid}, ne);
% %                         if(nModelid == 3)
% %                             [P] = affinefundmatrix(x1, x2);
% %                             groupModel{gid} = 1;
% %                         else
% %                             [P, A] = fundmatrix_A(x1, x2);
% %                             t = null(A);
% %                             if(~isempty(t))
% %                                 groupModel{gid} = 1;% Fundamental Matrix 
% %                             else
% %                                 P = homography2d(x1, x2);
% %                                 groupModel{gid} = 2;% homography                           
% %                             end
% %                         end
% % %                     elseif(nModelid == 2)
% %                         allpi = grouppointid{gid};  
% %                         p0 = pts(:, allpi(1), 1);
% %                         p1 = pts(:, allpi(1), 2);
% %                         P = pp2v_general(p0, p1, pts, allpi);
% %                     end
%                     if(isempty(P))% Outliers
%                        gid = gid - 1;
%                        continue;
%                     else
%                         npt = npts;
%                         allpi = 1:npt;
%                         if(nModelid == 2)
%                             nerr = Calpp2vErr_general(p0, p1, P, pts, epsf);
%                             Kh = ceil(0.2*npt);%
%                             if(bAdaptive == 1)                            
%                                 [h] = IKOSE(nerr, Kh);   
%                                 E = 25.;%5;%1.;%/2;%2.5;%5;%3;%10;%4;%2.5;% 6;%h = 1/2.5;%3.5
%                                 fthreshold1 = E * h;
%                             else
%     %                             fthreshold1 = 10;%20;%5;%h/E;%0.1%0.015;%                            
%                             end
%                         elseif((nModelid == 1) || (nModelid == 3))
%                             x1 = y(:, :, 1);
%                             x2 = y(:, :, 2);
% %                             [x1, T1] = normalise2dpts(x1);
% %                             [x2, T2] = normalise2dpts(x2);
% %                             P = inv(T2')*P*inv(T1);
%                             if(groupModel{gid} == 1)
% %                                 x2tFx1 = zeros(1,length(x1));
% %                                 for n = 1:length(x1)
% %                                     x2tFx1(n) = x2(:,n)'*P*x1(:,n);
% %                                 end
% %                                 Fx1 = P*x1;
% %                                 Ftx2 = P'*x2;
% %                                 % Evaluate distances
% %                                 d =  x2tFx1.^2 ./ ...
% %                                      (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
%                                 d = calFundamentalMatrixErr(P, x1, x2);
%                             elseif(groupModel{gid} == 2)
%                                 d = calHomographyErr(P, x1,x2);
% %                                 % Calculate, in both directions, the transfered points    
% %                                 Hx1    = P*x1;
% %                                 invHx2 = P\x2;
% %                                 % Normalise so that the homogeneous scale parameter for all coordinates
% %                                 % is 1.
% %                                 x1     = hnormalise(x1);
% %                                 x2     = hnormalise(x2);     
% %                                 Hx1    = hnormalise(Hx1);
% %                                 invHx2 = hnormalise(invHx2);
% %                                 d = sqrt(sum((x1-invHx2).^2)  + sum((x2-Hx1).^2)); 
%                             end
%                             nerr = abs(d);
% %                             fthreshold1 = 200;%500;%2;%8e-4;%
%                         end
%                         inlieridx = find(nerr < fthreshold1);
%                         
%                         if(length(inlieridx) > nminclustersize)%ceil(0.1*npt)
% %                             figure; quiver(y(1, :, i), -y(2, : ,i), dy(1,:), dy(2,:), 'b');%quiver(y(1, :, i), -y(2, : ,i), dy(2,:), dy(1,:), 'b');%quiver(y(1, :, i), y(2, : ,i), dy(1,:), dy(2,:), 'b');
% %                             grouppointid{gid} = inlieridx;
% % 
% %                             allpi = grouppointid{gid};  
% %                             hold all; cid = 2;
% %                             plot(y(1, allpi, i), -y(2, allpi, i), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize',10);
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
% %                         hold all; cid = 4;%inlieridx = inliers_local;
% %                         plot(pts(1, inlieridx, 1), -pts(2, inlieridx, 1), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize', 5);
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
% %                 if((nModelid == 1) || (nModelid == 3))
% %                     % Now do a final least squares fit on the data points considered to be inliers.
% %                     x1 = y(:, grouppointid{gj}, i);
% %                     x2 = y(:, grouppointid{gj}, ne);
% %                     groupP{gj} = fundmatrix(x1, x2);                    
% %                 elseif(nModelid == 2)
% %                     groupP{gj} = pp2v_general(groupPbase{gj}(:,1), groupPbase{gj}(:,2), pts, grouppointid{gj});
% %                 end
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
% %     for(ni=1:points)
% %         pgid = find(gpts(:,ni) > 0);
% %         if(length(pgid) > 1)% at the intersection of more than 1 group% grouppointid
% %             groups = findgroup(ni, groups, groupP, groupPbase, pgid, y, i, ne,nModelid,groupModel);
% % %         elseif(isempty(pgid))% Find home for outliers
% % %               for(ng=2:ngroups)
% % %                   if(~isempty(groupP{ng}))
% % %                     pgid = [pgid, ng];
% % %                   end
% % %               end
% % % %             pgid = 2:ntotalgroups;
% % %             groups = findgroup(ni, groups, groupP, groupPbase, validgid, y, i, ne, nModelid,groupModel);
% %         end
% %     end
%     
% %     Displaygroups(y, dy, i, points, grouppointid, colors, 'Final Groups', ntotalgroups);
%     
%     newgid = 1;
%     ntotalgroups = length(validgid);
%     if(ntotalgroups > 0)
%         for(gi =1: length(validgid))
%             grouppointid{gi} = grouppointid{validgid(gi)};
%     %         groupP{gi} = [];
%     %         groupPts{gi} = [];
%     %         grouppointid{gi} = [];
%             grouppointid{validgid(gi)} = [];
%         end
%         grouppointid = grouppointid(~cellfun('isempty', grouppointid));
%         groupP = groupP(~cellfun('isempty', groupP));
%         ng = size(groupP, 2);
%         if(bDisplay)
%             Displaygroups(y, dy, 1, npts, grouppointid, colors, sinfo, ntotalgroups);
%         end
%     else
%         grouppointid = [];
%     end
%     
% %     %% Re-assess the point-group correspondences
% %     %%%%%%%%%%%%%%%%%%%%%%% 
% %     pgid = validgid;
% %     for(ni=1:points)       
% %             groups = findgroup(ni, groups, groupP, groupPbase, pgid, y, i, ne,nModelid,groupModel);
% %     end
% %     
% %     Displaygroups(y, dy, i, points, groups, colors, 'Last Groups');