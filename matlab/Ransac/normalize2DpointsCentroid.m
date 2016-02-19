% normalize2DpointsCentroid - Normalization of 2D points so that the
% centroid is at the origin
%
% Normalizes the points so that the origin is at the centroind of the
% points
%
% Input  - 2xn matrix of image points
%
% Output - 2xn matrix of normalized points 
%        - Transformation matrix
%
%
%
% Author: Isaac Esteban
% IAS, University of Amsterdam
% TNO Defense, Security and Safety
% iesteban@science.uva.nl
% isaac.esteban@tno.nl

function [X,T] = normalize2DpointsCentroid(X)

    % Calculate centroid
    c = mean(X(1:2,:)')';
    
    % Adjust points so that the origin is at the centroid
    X(1,:) = X(1,:)-c(1);
    X(2,:) = X(2,:)-c(2);
    
    T = c;