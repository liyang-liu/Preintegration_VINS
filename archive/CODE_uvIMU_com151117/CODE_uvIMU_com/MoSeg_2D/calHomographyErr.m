function d = calHomographyErr(P, x1,x2)

    % Calculate, in both directions, the transfered points    
    Hx1    = P*x1;
    invHx2 = P\x2;
    % Normalise so that the homogeneous scale parameter for all coordinates
    % is 1.
    x1     = hnormalise(x1);
    x2     = hnormalise(x2);     
    Hx1    = hnormalise(Hx1);
    invHx2 = hnormalise(invHx2);
    d = sqrt(sum((x1-invHx2).^2)  + sum((x2-Hx1).^2)); 