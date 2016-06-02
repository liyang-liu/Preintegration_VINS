function groups = findgroup(ni, groups, groupP, groupPbase, pgid,y, i, ne, nModelid,groupModel)
    nerr = [];
    for(ng=1:length(pgid))
        P = groupP{pgid(ng)};
        if((nModelid == 1) || (nModelid == 3))
            x1 = y(:, ni, i);
            x2 = y(:, ni, ne);  
            if(groupModel{pgid(ng)} == 1)
%             x2tFx1 = x2'*P*x1;
%             Fx1 = P*x1;
%             Ftx2 = P'*x2;
%             % Evaluate distances
%             d =  x2tFx1.^2 ./ ...
%                  (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
                d = calFundamentalMatrixErr(P, x1, x2);
            elseif(groupModel{pgid(ng)} == 2)
                d = calHomographyErr(P, x1,x2);
            end
            nerr(ng) = abs(d); 
        elseif(nModelid == 2)
    %         p0 = y(1:2, grouppointid{pgid(ng)}(1), i);
    %         p1 = y(1:2, grouppointid{pgid(ng)}(1), ne);
            duvi = y(1:2, ni, i) - groupPbase{pgid(ng)}(:,1);
            duvi1 = y(1:2, ni, ne) - groupPbase{pgid(ng)}(:,2);                
            nerr(ng) = norm(P*duvi - duvi1);
        end
    end
    [~, ci] = min(nerr);
    c = round(y(1, ni, i));
    r = round(y(2, ni, i));
    groups(r,c) = pgid(ci);