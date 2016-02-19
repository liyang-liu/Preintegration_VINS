function [g] = dseg(p1,p2, Td, bTdAdpt)%, bRegionSort)
%% p1,p2: nx3, two corresponding 3D points in two frames

 np = size(p1, 1);
        
%         %Pre-sort
%         if(bRegionSort)
%             [p1 idx]=sortrows(p1);
%             p2 = p2(idx,:);
%         end

    %% Segmentation

    % 1. Pick up the first point into the first group
        k = 1;
        g = {{1}, {[1]},{'', [0, 0, 0; 0, 0, 0],[p1(k,:), p2(k,:), k]}};%{{GroupNum},{PtNof eachGp}, {Mature,PlaneCoef, grouppointSet}}
        k = k+1;

tic
        while(k <= np)
            pk1 = p1(k,:);
            pk2 = p2(k,:);
            for m=1:g{1}{1}% Iterate through available groups
                gms=g{2}{1}(m);%Each group has a number of points aligned in rows
                if(gms < 3)
                    for n=1:gms
                        if(bTdAdpt == 1)     
                            % Get adaptive threshold
                            Td = max(g{m+2}{3}(n,3), g{m+2}{3}(n,6))/65.;%100.;%110.;
%                             Td = max([g{m+2}{3}(n,3), g{m+2}{3}(n,6), pk1(3),pk2(3)])/100.;%
                        end
                        pd = abs(norm(pk1-g{m+2}{3}(n,1:3)) - norm(pk2-g{m+2}{3}(n,4:6)));%pd = abs(sum(abs(pk1-g{m+1}{4}(n,1:3)), 'double') - sum(abs(pk2-g{m+1}{4}(n,4:6)),'double'));
                        if(pd >Td)
                            n = 0;
                            break;
                        end
                    end
                    if(n == gms)% Pass all the tests, Belongs to this group
                        g{2}{1}(m) = g{2}{1}(m) + 1;% Increas the point number
                        g{m+2}{3} = [g{m+2}{3};pk1, pk2, k];% 
                        if(gms == 2)% Now is 3, get the plane function, *degenerate situation---line
                            nplane = cross((g{m+2}{3}(1,1:3) - g{m+2}{3}(2,1:3)), (g{m+2}{3}(3,1:3) - g{m+2}{3}(2,1:3)));                        
                            if(norm(nplane) ~= 0)%nplane ~= zeros(1,3))%Degenerate situation: 3p on a line
                                nplane = nplane/norm(nplane);
                                g{m+2}{2}(1,:) = nplane;% Remember the plane
                                nplane = cross((g{m+2}{3}(1,4:6) - g{m+2}{3}(2,4:6)), (g{m+2}{3}(3,4:6) - g{m+2}{3}(2,4:6)));	
                                nplane = nplane/norm(nplane);
                                g{m+2}{2}(2,:) = nplane;% Remember the plane'
                                g{m+2}{1} = 'm';
                            end
                     
                        end
                        break;%Have found a group, do not need to iterate through the other groups 
                    end
                elseif(g{m+2}{1} == 'm')% Mature group% >=3, a mature group, the first 3 amount to be the basic components of this group
                    for n=1:3
                        pd = abs(norm(pk1-g{m+2}{3}(n,1:3)) - norm(pk2-g{m+2}{3}(n,4:6)));%pd = abs(sum(abs(pk1-g{m+1}{4}(n,1:3)), 'double') - sum(abs(pk2-g{m+1}{4}(n,4:6)),'double'));
                        if(pd >Td)
                            n = 0;
                            break;
                        end
                    end%
                    if(n == 3)% Passed the first three tests                	
                        pd1 = (g{m+2}{2}(1,:)) * (pk1 - g{m+2}{3}(2,1:3))'/norm(pk1 - g{m+2}{3}(2,1:3));
                        pd2 = (g{m+2}{2}(2,:)) * (pk2 - g{m+2}{3}(2,4:6))'/norm(pk2 - g{m+2}{3}(2,4:6));
                        if(abs(pd1 - pd2) <= Td)%(pd1*pd2 >= 0)%On the same side of the plane Belongs to this group                 
                            g{2}{1}(m) = g{2}{1}(m) + 1;% Increas the point number
                            
                            g{m+2}{3} = [g{m+2}{3}; pk1, pk2, k];

                            break;%Have found a group, do not need to iterate through the other groups 
                        else
                            n = 0;
                        end
                    end%
                else% Inmature group but points >=3 are on a single line
                   for n=1:2
                       pd = abs(norm(pk1-g{m+2}{3}(n,1:3)) - norm(pk2-g{m+2}{3}(n,4:6)));%pd = abs(sum(abs(pk1-g{m+1}{4}(n,1:3)), 'double') - sum(abs(pk2-g{m+1}{4}(n,4:6)),'double'));
                        if(pd >Td)
                            n = 0;
                            break;%Not belong to this group
                        end
                    end%
                    if(n == 2)% Passed the first two tests                    
                        g{2}{1}(m) = g{2}{1}(m) + 1;% Increas the point number
                        g{m+2}{3} = [pk1, pk2, k;g{m+2}{3}];

                        nplane = cross((g{m+2}{3}(1,1:3) - g{m+2}{3}(2,1:3)), (g{m+2}{3}(3,1:3) - g{m+2}{3}(2,1:3)));                  
                        if(norm(nplane) ~= 0)%nplane ~= zeros(1,3))%This point has made the group mature and become one of the basic components 
                            nplane = nplane/norm(nplane);
                            g{m+2}{2}(1,:) = nplane;% Remember the plane
                            nplane = cross((g{m+2}{3}(1,4:6) - g{m+2}{3}(2,4:6)), (g{m+2}{3}(3,4:6) - g{m+2}{3}(2,4:6)));	
                            nplane = nplane/norm(nplane);
                            g{m+2}{2}(2,:) = nplane;% Remember the plane'                        
                            g{m+2}{1} = 'm';	
                        else%Still degenerate, but belongs to this group

                        end
                        break;
                    end                
                end
            end
            if(n == 0)%Found no group, set a new one
                g{1}{1} = g{1}{1} +1;
                m = size(g{2}{1}, 2);
                g{2}{1}(m+1) = 1;
                g{g{1}{1} + 2} = {'', [0, 0, 0; 0, 0, 0], [pk1, pk2, k]};          
            end
          
            k = k + 1;

        end
toc

       
   


        
