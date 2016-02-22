function [p3d] = fnTrianguFeatures(K, Rci, Tci, Rci1, Tci1, comfeatures)


    P1 = K*Rci*[eye(3),Tci];%important
    P2 = K*Rci1*[eye(3),Tci1];%important

    [X] = Func_ComputeX(comfeatures(1:2, :), comfeatures(3:4, :), P1, P2);
    
    p3d = X./repmat(X(4, :), 4, 1);
    p3d = (p3d(1:3, :))';

