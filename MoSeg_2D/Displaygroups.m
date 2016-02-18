function []=Displaygroups(y, dy, i, points, grouppointid, colors, stitle, ntotalgroups)

%     addpath(genpath('Utilities'));
    cid = 1;
    figure('name', stitle);% figure;% maximize;
    quiver(y(1, :, i), -y(2, : ,i), dy(1,:), dy(2,:), colors(cid));%title(stitle); 
    ngroups = min(ntotalgroups, length(colors));
    
    [np] = cellfun('size', grouppointid, 2);
    [~, idx] = sort(np, 'descend');
%     grouppointid = grouppointid(~cellfun('sort', grouppointid, 'desend'));
    
    for(gi=1:ngroups) %1)%
        pi = grouppointid{idx(gi)};
        if(~isempty(pi))            
            hold all; plot(y(1, pi, i), -y(2, pi, i), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize', 4);
            cid = cid + 1;
        end
    end    
    
%     for(pi = 1:points)
%         c = round(y(1, pi, i));
%         r = round(y(2, pi, i));
%         cid = groups(r,c);
%         if(cid < length(colors))
%             hold all; plot(y(1, pi, i), -y(2, pi, i), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize',10);
%             %plot(y(1, pi, i), y(2, pi, i), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize',10);
%         end
%     end