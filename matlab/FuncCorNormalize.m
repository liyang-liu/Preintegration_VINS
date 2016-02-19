function [NorMatrixx1,NorMatrixx2,NormalizedCorx1,NormalizedCorx2] = FuncCorNormalize(OriginalCorx1,OriginalCorx2)
% Find the Normalizing transformation Matrices to normaliz the correspondences

%[nRowNum,nColNum] = size(OriginalCor);

%%
% Get the mean value and the standard deviation of the correspondences
Mx1 = mean(OriginalCorx1');
Mx2 = mean(OriginalCorx2');
Sx1 = std(OriginalCorx1',1);
Sx2 = std(OriginalCorx2',1);

%%
% Get the Normalizing transformation Matrices of the correspondences
NorMatrixx1=[1/Sx1(1),0,-Mx1(1)/Sx1(1);0,1/Sx1(2),-Mx1(2)/Sx1(2);0,0,1];
NorMatrixx2=[1/Sx2(1),0,-Mx2(1)/Sx2(1);0,1/Sx2(2),-Mx2(2)/Sx2(2);0,0,1];

%%
% Get the Normalized correspondences 
NormalizedCorx1=NorMatrixx1*OriginalCorx1;
NormalizedCorx2=NorMatrixx2*OriginalCorx2;
