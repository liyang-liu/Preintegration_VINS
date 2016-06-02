function [inlierIds, Ft] = fnGetFundamentalMatrix_8p(sX1, sX2, X1, X2)


    Ft = []; inlierIds = [];
    % Parameters
    t = 1e-3;%1e-2;%0.001;
    % First normalize the points
    [X1t,T1] = normalize2Dpoints(X1);
    [X2t,T2] = normalize2Dpoints(X2);
    
    % Compute the fundamental matrix
    Ft = eightpoint(sX1,sX2);
    % If it can be computed...
    if(sum(sum(isnan(Ft))) == 0)  
        [inlierIds] = fnGetInliers5F(X1t, X2t, Ft, t);
    else
        Ft = [];
        return;
    end        

    if(size(inlierIds,2) > 8)
%         % Reestimate F based on the inliers
%         Ft = eightpoint(X1(:,inlierIds),X2(:,inlierIds));     
%         % If it can be computed...
%         if(sum(sum(isnan(Ft))) == 0)  
%             [inlierIds] = fnGetInliers5F(X1t, X2t, Ft, t);
%         end    
    else
        Ft = []; inlierIds = [];
        return;
    end
        
