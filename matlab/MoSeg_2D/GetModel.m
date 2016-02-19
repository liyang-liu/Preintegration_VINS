function [P, groupModel] = GetModel(nModelid, y, i, ne, grouppointid, groupModel, gid, pts, p0, p1) 
    
    if((nModelid == 1) || (nModelid == 3))
%         p0 = []; p1 = [];
        x1 = y(:, grouppointid{gid}, i);
        x2 = y(:, grouppointid{gid}, ne);
        if(nModelid == 3)
            [P] = affinefundmatrix(x1, x2);
            groupModel{gid} = 1;
        else
            [P, A] = fundmatrix_A(x1, x2);
            t = null(A);
            if(~isempty(t))
                groupModel{gid} = 1;% Fundamental Matrix 
            else
                P = homography2d(x1, x2);
                groupModel{gid} = 2;% homography                           
            end
        end
    elseif(nModelid == 2)
        allpi = grouppointid{gid};
%         p0 = pts(:, allpi(1), 1);
%         p1 = pts(:, allpi(1), 2);
        P = pp2v_general(p0, p1, pts, allpi);
    end