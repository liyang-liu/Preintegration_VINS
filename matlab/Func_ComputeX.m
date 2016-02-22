function [X] = FuncComputeX(OriginalCorx1,OriginalCorx2,P1,P2)
% Computer the 3d space point X from the correspondences and asuming P
% matrices using the Linear trangulation methods.

[nRowNum,nColNum] = size(OriginalCorx1);

for i=1:nColNum;
%%
% Make the linear equations matrix A from the correspondences and asuming P matrices.

A(1,:) = OriginalCorx1(1,i)*P1(3,:)-P1(1,:);
A(2,:) = OriginalCorx1(2,i)*P1(3,:)-P1(2,:);
A(3,:) = OriginalCorx2(1,i)*P2(3,:)-P2(1,:);
A(4,:) = OriginalCorx2(2,i)*P2(3,:)-P2(2,:);

%%
% Get the Linear solution for 3d space point X is the singular vector corresponding to the smallest singular value of A, 
% that is, the last column of V in the SVD A = UDVT.
[U,D,V] = svd(A);
X(:,i) = V(:,4);

end;