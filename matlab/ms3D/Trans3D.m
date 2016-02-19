function [rt] = Trans3D(x)
%
%  transform from two point sets a and b
%  Dissa -- modified by Shoudong, 2009, August
%
%[tc,rc,failed,c,d] = Trans3D(a,b)
% using the SVD (Singular Value Decomposition) algorithm by Arun et.al 
% paper: Least-squares fitting of two 3-D points sets, IEEE Transactions on
% Pattern Analysis and Machine Intelligence, Vol PAMI-9, No. 5, Sept 1987,
% page 698-700.

% size of sets a
a = x(:, 1:3);
b = x(:, 4:6);
nsize = size(a,1);
% averge of a
p = mean(a)';
% average of b
pp = mean(b)';
% shift sets a
q = a - ones(nsize,1)*p';
% shift sets b
qq = b - ones(nsize,1)*pp';
% do SVD 
h = q'*qq;
[u,s,v] = svd(h);
x = v*u';
% get the rotation matrix
rc = x;
failed = 0;

%%% if det(x) = -1, then the algorithm fail, you get a reflection instead
%%% of a rotation

if det(x) < -0.99
    det_x = det(x);
   % disp(' it is a reflection')
    s=s;
   % pause
    diag_s = diag(s);
    [min_s ind] = min(diag_s);
% if the minimal singular value is zero, it is possible to get the rotation
    if min_s<1e-3 
        min_singular_value = min_s;
        %disp(' try to get a solution')       
        v(:,ind)= -v(:,ind);
        rc = v*u';
    else
        % the noise is too large, least square method is not useful
        %disp(' the noise is too large,  need to remove outliers, reduce the number of samples in RANSAC')
        failed = 1;
    end

end
% get the translation
tc = pp - rc*p;
%tc = rc'*tc;%Added by wyb

%
% c is a transformed to location b
% d is b trans to location a
%
c = (rc*a')' + ones(nsize,1)*tc';
d = (rc'*b')' - ones(nsize,1)*(rc'*tc)';

if failed == 0
% % check the transformation accuracy
check_1 = b-c;
check_2 = a-d;
end

if failed == 1
   warning(' algorithm failed');%disp
   rt = [];
else
   rt = [rc,tc];   
end









