function [inliers, R, T] = UpdateModelinliers(p3d1, p3d2, inliers, np, nminp, fthreshold)

        [rt] = Trans3D([p3d1(:, inliers); p3d2(:, inliers)]');
        if(isempty(rt))
            inliers = [];
            R = [];
            T = [];
            return;
        end
        R = rt(:, 1:3);
        T = rt(:, 4);
        x2tox1 = R' * p3d2 - repmat(R'  * T, 1, np);
        x2tx1 = zeros(1, np);
        for n = 1:np
            x2tx1(n) = norm(x2tox1(:, n) - p3d1(:, n));
        end
        inliers = find(x2tx1 < fthreshold);
        if(length(inliers) < nminp)
%             inliers = find(x2tx1 < 6e-3);
%             if(length(inliers) < nminp)
                R = [];
                T = [];
%             end
        end
end