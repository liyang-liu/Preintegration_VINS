% normalize2Dpoints - Normalization of 2D image points for the DLT
%                     algorithm
%
% Normalizes the points so that the origin is at the centroind of the
% points and the average distance to the origin is sqr(2)
%
% Input  - 2xn matrix of image points
%
% Output - 3xn matrix of normalized points (including w=1)
%        - Transformation matrix
%
%
%
% Author: Isaac Esteban
% IAS, University of Amsterdam
% TNO Defense, Security and Safety
% iesteban@science.uva.nl
% isaac.esteban@tno.nl


function [X, T] = normalize2Dpoints(X1)

    if(size(X1,1)==2)
        Xt = ones(3,size(X1,2));
        Xt(1:2,:) = X1;
        X1 = Xt;
    end;

    X1n = X1;
    
    % Normalize with the centroid
    [X1n(1:2,:),c1] = normalize2DpointsCentroid(X1(1:2,:));
    
    % Compute the mean distance
    meandistance1 = mean(sqrt(X1n(1,:).^2+X1n(2,:).^2));

    % Compute the scale factor
    scale1 = sqrt(2)/meandistance1;

    % Transformation matrix
    T = [scale1, 0,      -scale1*c1(1);
            0,    scale1, -scale1*c1(2);
            0,    0,       1];

    % Recalculate points
    X = T*X1;
