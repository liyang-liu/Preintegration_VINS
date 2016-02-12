function [e] = fnUVDErr_C1U(K, x, Zobs, nPoses, nPts, nIMUrate)

global InertialDelta_options

f = K(1,1); cx0 = K(1,3); cy0 = K(2,3);
if(InertialDelta_options.bPreInt == 1)
    idx = (nPoses-1)*6;
else
    idx = (nPoses-1)*nIMUrate*6;
end
p3d0 = reshape(x((idx+1):(idx+3*nPts), 1), 3, []);
e = Zobs;
if(bPreInt == 1)
    idx = ((nPoses-1)*6+nPts*3+3*nPoses+3);%(nPoses-1)*6+nPts*3+3*nPoses + 9
else
    idx = (nPoses-1)*nIMUrate*6+3*nPts+3*((nPoses-1)*nIMUrate+1)+3;
end
alpha = x(idx+1);beta = x(idx + 2); gamma = x(idx + 3);
Ru2c = fnR5ABG(alpha, beta, gamma);
Tu2c = x((idx+4):(idx+6));
% Reprojection at each pose
for pid=1:nPoses
    if(pid > 1)
        if(bPreInt == 1)
            idx = (pid-2)*6;
        else
            idx = (pid-1)*nIMUrate*6-6;
        end
        alpha = x(1+idx); beta = x(2+idx); gamma = x(3+idx); 
        Tu = x((4+idx):(idx+6), 1);
        Ru = fnR5ABG(alpha, beta, gamma);%Rx(alpha) * fRy (beta) * fRz(gamma);
    else % Pose 1 is special
        Tu = zeros(3,1); 
        Ru = eye(3);
    end
    Rc = Ru2c * Ru;
    Tc = Tu + (Ru)' * Tu2c;
    p3d1 = Rc * (p3d0 - repmat(Tc, 1, nPts));
    u1 = f * p3d1(1, :) ./ p3d1(3, :) + repmat(cx0, 1, nPts);
    v1 = f * p3d1(2, :) ./ p3d1(3, :) + repmat(cy0, 1, nPts);
%     d1 = p3d1(3, :);
%     et = [u1;v1;d1];
    et = [u1;v1];
    et = et(:);
    e((nPts*2*(pid-1)+1):(nPts*2*pid), 1) = et - Zobs((nPts*2*(pid-1)+1):(nPts*2*pid), 1);    
%     e((N*3*(pid-1)+1):(N*3*pid), 1) = et - e((N*3*(pid-1)+1):(N*3*pid), 1);
end