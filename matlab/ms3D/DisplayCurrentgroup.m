function []=DisplayCurrentgroup(I1, y, dy, i, points, grouppointid, colors, stitle, ntotalgroups)

%     addpath(genpath('Utilities'));
    cid = 1;
    figure('name', stitle); 
    if(~isempty(I1)) 
        imshow(I1); % figure;% maximize;
        hold all; quiver(y(1, :, i), y(2, : ,i), dy(1,:), dy(2,:), colors(cid));
    else
        quiver(y(1, :, i), -y(2, : ,i), dy(1,:), -dy(2,:), colors(cid));%title(stitle); 
    end
    
    for(gi=1:1)%ngroups)
        pi = grouppointid;
        if(~isempty(pi))            
            hold all; 
            if(~isempty(I1))
                plot(y(1, pi, i), y(2, pi, i), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize', 4);
            else
                plot(y(1, pi, i), -y(2, pi, i), 'rs','LineWidth',2,'MarkerEdgeColor',colors(cid), 'MarkerFaceColor',colors(cid), 'MarkerSize', 4);
            end
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