function [] = ShowMotionGroups(I1, I2, u1f, v1f, u2f, v2f, g, idx, bShow2nd)

        figure(1);%(ni0+100);subplot(2,1,1);
        imshow(I1);
        hold on; 
        colors = 'brgymkw';m = min(size(colors, 2), g{1}{1});%;%cbrgcmybrgcmy%5;%3;%
        for groupIndex=1:m%g{1}{1}
            s =g{idx(groupIndex) + 2}{3}(:,7);%640-480 - 
            plot(u1f(s),v1f(s),'rs','LineWidth',2,'MarkerEdgeColor',colors(groupIndex), 'MarkerFaceColor',colors(groupIndex), 'MarkerSize',15);
        end
        
        if(bShow2nd == 1)
            figure(2);%(ni0+100+1);subplot(2,1,2);
            imshow(I2);
            hold on; 
            for groupIndex=1:m%g{1}{1}
                s =g{idx(groupIndex) + 2}{3}(:,7);%640-480 - 
                plot(u2f(s),v2f(s),'rs','LineWidth',2,'MarkerEdgeColor', colors(groupIndex), 'MarkerFaceColor',colors(groupIndex), 'MarkerSize',15);
            end
        end