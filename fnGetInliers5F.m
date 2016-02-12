function [inlierIds] = fnGetInliers5F(X1t, X2t, Ft, fThreshold)


           
            X2tFX1 = zeros(1,size(X1t,2));
            for n = 1:size(X1t,2)
                X2tFX1(n) = X2t(:,n)'*Ft*X1t(:,n);
            end

            FX1 = Ft*X1t;
            FtX2 = Ft'*X2t;     

            % Evaluate distances
            d =  X2tFX1.^2 ./ (FX1(1,:).^2 + FX1(2,:).^2 + FtX2(1,:).^2 + FtX2(2,:).^2);
             
            inlierIds = find(abs(d) < fThreshold);     % Indices of inlying points