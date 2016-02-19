function d = calFundamentalMatrixErr(P, x1,x2)

    x2tFx1 = zeros(1,size(x1,2));
    for n = 1:size(x1,2)%length(x1)
        x2tFx1(n) = x2(:,n)'*P*x1(:,n);
    end
    Fx1 = P*x1;
    Ftx2 = P'*x2;
    % Evaluate distances
    d =  x2tFx1.^2 ./ ...
         (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);