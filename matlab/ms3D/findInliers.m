function  [Yaw, Pitch, Roll, R, T, index_pairs] = findInliers(bPrev, nGroup, nPose, I1, I2, bStatic, Td, bTdAdpt, K, cSift, fm, fn, index_pairs)


%         if(cSift == 0)          
            u1f = fm(index_pairs(:, 1), 3);
            v1f = fm(index_pairs(:, 1), 4);
            z1 = fm(index_pairs(:, 1), 7);
            p1 = fm(index_pairs(:, 1), 5:7);        
            u2f = fn(index_pairs(:, 2), 3);
            v2f = fn(index_pairs(:, 2), 4);
            z2 = fn(index_pairs(:, 2), 7);
            p2 = fn(index_pairs(:, 2), 5:7); 
%         elseif(cSift == 1)
%             u1f = fm(index_pairs(2, :), 3);
%             v1f = fm(index_pairs(2, :), 4);
%             u2f = fn(index_pairs(1, :), 3);
%             v2f = fn(index_pairs(1, :), 4);  
%         end
        
%         I1=I1-min(I1(:)) ;
%         I1=I1/double(max(I1(:))) ;
%         I2=I2-min(I2(:)) ;
%         I2=I2/double(max(I2(:))) ;
        
      
        figure; showMatchedFeatures(I1,I2,[u1f,v1f],[u2f,v2f], 'montage');% blend falsecolor
        %plotmatches(I1, I2,[u1f,v1f] - 1,[u2f,v2f] - 1, index_pairs', 'interactive', 1) ;
        legend('mp1','mp2');        


% 
%         p1 = [x1, y1, z1];
%         p2 = [x2, y2, z2];
%         sz = zeros(size(z1,1),1);
% 
%         dp = 100*(p2 -p1);
%         for(j=1:size(p1,1))
%             ndp(j,1) = norm(dp(j,:));
%         end
%         
% %         figure;hold on; plot(x1,y1,'bx'); %imshow(I1); , x2,y2
% %         plot3(x1,y1,z1, 'ro', x2,y2,z2, 'bd', 'MarkerSize',10);% figure;mesh(p2);        

        if(bStatic == 0)
            %% Feature-distance-based segmentation
            [g] = dseg(p1, p2, Td, bTdAdpt);%, bRegionSort);    
        
            %% Re-sort the groups, try to find the static group
            maxduvz = [];
            ncl = g{1}{1};
            for(j=1:ncl)
                mnmx = minmax((g{(j)+2}{3}(:,1:3))');
                delta = mnmx(:,2) - mnmx(:,1);
                maxduvz = [maxduvz delta(1)*delta(2)*delta(3)];
            end
            %% Find the static group
            [~, idstatic] = max(maxduvz);
            % Make this group to be the biggest     
            g{2}{1}(idstatic) = g{2}{1}(idstatic) + 1e5;    
            [B, idx] = sort(g{2}{1}, 'descend');
            B(1) = B(1) - 1e5;
            g{2}{1}(idstatic) = g{2}{1}(idstatic) - 1e5;
            
            bShow2nd = 1;
            ShowMotionGroups(I1, I2, u1f, v1f, u2f, v2f, g, idx, bShow2nd)
        else
            idx = 1;
            g = {{1}, {[size(p1,1)]},{'m', [0, 0, 0; 0, 0, 0],[p1, p2, (1:size(p1,1))']}};
        end

%         if(bFund ==1)
%             %%%%%%%%%%DECOMPOSE USING EPIPOLAR%%%%%%%%%%%%%%%%%%%%%%%%%%
%             F_Results.GT=[0 0 0 0 0 0]; F_Results.T=[]; 
%             F_Results.Match=[]; F_Results.MatchD=[];
%             sigma_noise=0.1;
%             K = [fx, 0, cx; 0, fy, cy; 0, 0, 1];
%             ncl=min(g{1}{1}, 1);
% 
%             for m=1:ncl%2
%                 if(g{2}{1}(idx(m)) < 3)
%                     break;
%                 end  
%                 s =g{idx(m) + 2}{3}(:,7);
%                 [F_R,F_T,Ematrix,inliersF,F_GRIC,r_errorsF]=FDecomp(double([u1f(s),v1f(s)])', double([u2f(s),v2f(s)])',K,sigma_noise);%(g{idx(1)+2}{3}(:,1:2))', (g{idx(1)+2}{3}(:,4:5))'
%                 F_Results.T=[F_Results.T;[F_T ,F_R]];
%             end
%         end    
       
        addpath('GaussNewton');
        addpath('ransac');
        
%         ncl=min(g{1}{1}, nShowgroup); 
                        n1 = [   3,3,3,1,2,2,2,2,2,1,...
                                     2,3,3,1,3,3,1,1,3,1,...
                                     2,1,2,2,2,2,1,1,1,2,...
                                     1,1,3,2,2,2,2,4,3,3,...
                                     2,2,1,2,2,0];
                        n2 = [   2,1,2,2,2,2,5,3,0,0,...
                                     3,2,2,1,2,2,2,1,2,2,...
                                     1,1,3,3,4,3,2,1,1,1,...
                                     1,3,4,4,3,3,1,2,2,1,...
                                     2,6,2,2,3,2];                                 


                   


            if(cSift == 0)
                thr = 2;%3;%1;%7;%4;%10;%0.5;%1e-1;%3;%5;%0.3;%0.01;%6;%4e-4;%5;%4e-6;%
            else
                thr = 2;%4;%7;%1;%1.5;%3;%;3;%5;%3;%1e-4;%4e-6;%1e-5;%
            end
            if(nGroup == 0)
%                 m = 1;
                if((nPose == 22))% || (nPose == 2))
                    m = 2;
%                 elseif((nPose == 3))
%                     m = 4;
                elseif((nPose == 20))
                    m = 3;    
                else
                    m = 1;
                end
            elseif(nGroup == 1)
                if((bPrev == 0) && (nPose == 1))
                    m = n1(nPose);
                else
                    m = 1;
                end
            elseif(nGroup == 2)
                if((bPrev == 0) && (nPose == 1))
                    m = n2(nPose);
                else
                    m = 1;
                end
            end
%             for m=1:ncl
                
%                dp = 10*(g{idx(m)+2}{3}(:,4:6) -g{idx(m)+2}{3}(:,1:3));
%                figure;quiver3(g{idx(m)+2}{3}(:,1), g{idx(m)+2}{3}(:,2), g{idx(m)+2}{3}(:,3), dp(:,1), dp(:,2), dp(:,3));view(30,60);                
      
                
                [rt, inliers] = ransacfit3drt((g{idx(m)+2}{3}(:,1:3)), (g{idx(m)+2}{3}(:,4:6)), thr, K);%0.1
                R = rt(:, 1:3);
                T = rt(:, 4);
%                     s = g{idx(m)+2}{3}(:, 7);
%                     [R, T, inliers] = solveRTRANSAC((g{idx(m)+2}{3}(:,1:3))',...
%                         (g{idx(m)+2}{3}(:,4:6))', [u2f(s), v2f(s)]', K);
                        s = g{idx(m)+2}{3}(:, 7);       
                            
%                 s = g{idx(m)+2}{3}(:, 7);  

%                 if(cSift == 0)

%                     sn = findobj('type', 'figure','name', 'Optflow');
%                     if(~isempty(sn))
%                         h = clf(sn);
%                     else
%                         h = figure('name', 'Optflow');
%                     end

                    figure(3);
                    imshow(I1);hold all;                    
                    quiver(u1f(s), v1f(s), (u2f(s)-u1f(s)), (v2f(s)-v1f(s)), 'b');hold all;
                    quiver(u1f(s(inliers)),v1f(s(inliers)), (u2f(s(inliers))-u1f(s(inliers))), (v2f(s(inliers))-v1f(s(inliers))), 'r');%figure; showMatchedFeatures(I1,I2,matched_pts1,matched_pts2, 'blend');%montage
                    legend('mp1','mp2');
%                     set(0,'current', h);
%                     pause(1);
%                     if(size(inliers, 2) < 10)
%                         pause;
%                     end
%                 elseif(cSift == 1)         
        %         figure; showMatchedFeatures(I1,I2,matched_pts1',matched_pts2', 'montage');% blend falsecolor
        %         legend('mp1','mp2');
%                 end                    
%                 hold all;
%                 for(i=1:size(inliers,2))
%                     text(g{idx(m)+2}{3}(inliers(i),1), g{idx(m)+2}{3}(inliers(i),2), ...
%                         g{idx(m)+2}{3}(inliers(i),3), ['\leftarrow (', ...
%                         num2str(g{idx(m) + 2}{3}(inliers(i),7)), ')'], 'FontSize',50);
%                 end
%                 s =g{idx(m) + 2}{3}(inliers,7)
%                 [F_R,F_T,Ematrix,inliersF,F_GRIC,r_errorsF]=FDecomp(double([u1f(s),v1f(s)])', double([u2f(s),v2f(s)])',K,sigma_noise);
                R = R';
                [Roll,Pitch,Yaw]=InvRotMatrixYPR22(R');
%                 [Yaw,Pitch,Roll]=InvRotMatrixYPR22(R');
                T = -R*T;   
                
                index_pairs = index_pairs(s(inliers), :);
