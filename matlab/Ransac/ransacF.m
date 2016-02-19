% ransacF - RANSAC algorithm for the estimation of the fundamental matrix
%
%
% Computes the fundamental matrix given two sets of corresponding points. Based on
% the 8-pint algorithm described by Zisserman in p282.
%
%
% Input  - A -> 3xn set of homogeneous points in image A
%        - B -> 3xn set of homogeneous points in image B
%
% Output - 3x3 fundamental matrix F
%
%
%
% Author: Isaac Esteban
% IAS, University of Amsterdam
% TNO Defense, Security and Safety
% iesteban@science.uva.nl
% isaac.esteban@tno.nl

function [F, bestmodel] = ransacF(X1,X2)

    % Parameters
    t = 1e-3;%0.001;
    N = 10000;
    
    % Best scores
    bestscore = 0;
    bestmodel = [];
    
    % List of points
    inliers = [];   
    counter = 1;
    
    % First normalize the points
    [X1t,T1] = normalize2Dpoints(X1);
    [X2t,T2] = normalize2Dpoints(X2);
    
    for i =1:N
        
        
        % Select 8 random points
        rindex = ceil(rand(8,1)*size(X1,2));
        
        % Compute the fundamental matrix
        Ft = eightpoint(X1(:,rindex),X2(:,rindex));
                     
        % If it can be computed...
        if(sum(sum(isnan(Ft))) == 0)
            
            X2tFX1 = zeros(1,size(X1t,2));
            for n = 1:size(X1t,2)
                X2tFX1(n) = X2t(:,n)'*Ft*X1t(:,n);
            end

            FX1 = Ft*X1t;
            FtX2 = Ft'*X2t;     

            % Evaluate distances
            d =  X2tFX1.^2 ./ (FX1(1,:).^2 + FX1(2,:).^2 + FtX2(1,:).^2 + FtX2(2,:).^2);
            %min(d) 
            inliersL = find(abs(d) < t);     % Indices of inlying points
            
            %size(inliersL)
            %pause
            
            % Check this model (size of the inliers set)
            if(length(inliersL)>bestscore)
                bestscore = length(inliersL);
                bestmodel = inliersL;
            end;
            
%             % Find which inliers are new
%             for j=1:length(inliersL)
%                 if(sum(inliers == inliersL(j))==0)
%                     inliers(counter) = inliersL(j);
%                     counter = counter+1;
%                 end;
%             end;
            
            
        end;
        
    end;
    
    fprintf('N. Inliers:  %d\n', size(X1(:,bestmodel),2));
    fprintf('N. Outliers: %d\n', size(X1,2)-size(X1(:,bestmodel),2));

        
    % Reestimate F based on the inliers of the best model only 

    F = eightpoint(X1(:,bestmodel),X2(:,bestmodel));