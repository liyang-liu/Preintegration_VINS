function [R,T,EMatrix] = FuncComputeRT(inlier,Calibtation)
%% Compute R and T from the inlier.
%%

K = inv(Calibtation);

[nRowNum,nColNum] = size(inlier);

for i=1:nColNum;
    corr1(1,i) = inlier(1,i);
    corr1(2,i) = inlier(2,i);
    corr1(3,i) = 1;
    corr2(1,i) = inlier(3,i);
    corr2(2,i) = inlier(4,i);
    corr2(3,i) = 1;
end;

norcorr1 = K*corr1;
norcorr2 = K*corr2;

norcorr = cat(1,norcorr1(1:2,:),norcorr2(1:2,:));

[NorMatrixx1,NorMatrixx2,NormalizedCorx1,NormalizedCorx2] = FuncCorNormalize(norcorr1,norcorr2);

[EMatrix] = FuncComputeFMatrix(NormalizedCorx1,NormalizedCorx2,NorMatrixx1,NorMatrixx2);

[P1,P2,R,T,rot_axis,rot_angle,g] = torr_linear_EtoPX22(EMatrix,norcorr',Calibtation,1);
% [P1,P2,R,T,rot_axis,rot_angle,g] = torr_linear_EtoPX(EMatrix,inlier',Calibtation,1);
