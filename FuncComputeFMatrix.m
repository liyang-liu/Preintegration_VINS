function [FMatrix] = FuncComputeFMatrix(NormalizedCorx1,NormalizedCorx2,NorMatrixx1,NorMatrixx2)
% Computer the Fundamental Matrix from the Find the normalized correspondences 
% using the 8 points algorithm. And then do the denormalization.

[nRowNum,nColNum] = size(NormalizedCorx1);

%%
% Make the linear equations matrix A from the normalized correspondences.
for i=1:nColNum;
    A(i,1) = NormalizedCorx2(1,i)*NormalizedCorx1(1,i);
    A(i,2) = NormalizedCorx2(1,i)*NormalizedCorx1(2,i);
    A(i,3) = NormalizedCorx2(1,i);
    A(i,4) = NormalizedCorx2(2,i)*NormalizedCorx1(1,i);
    A(i,5) = NormalizedCorx2(2,i)*NormalizedCorx1(2,i);
    A(i,6) = NormalizedCorx2(2,i);
    A(i,7) = NormalizedCorx1(1,i);
    A(i,8) = NormalizedCorx1(2,i);
    A(i,9) = 1;
end;

%%
% Get the least-squares solution for Fmatrix1 is the singular vector corresponding to the smallest singular value of A, 
% that is, the last column of V in the SVD A = UDVT.Get the Normalizing transformation Matrices of the correspondences.
[U,D,V] = svd(A);
for i=1:3;
    for j=1:3;
        FMatrix1(i,j) = V(3*(i-1)+j,9);
    end;
end;

%%
% Constraint enforcement.
if det(FMatrix1)==0;
	FMatrix2=FMatrix1;
else
    [U1,D1,V1] = svd(FMatrix1);
    D1(3,3)=0;
    FMatrix2=U1*D1*V1';
end;

%%
% Denormalization.
FMatrix = NorMatrixx2'*FMatrix2*NorMatrixx1;
