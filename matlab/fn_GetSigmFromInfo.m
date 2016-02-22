function Sigma0 = fnGetSigmFromInfo(Info)

[m,n] = size(Info);

% 1. 
tv = Info \ speye(m);

% % 2. 
% R = chol(Info);
% em = speye(m);
% y = R' \ em;
% tv = R \ y;
% 
Sigma0 = diag(tv);

% % 3. 
% Sigma0 = zeros(m,1);
% for(i=1:m)
%     em = zeros(n,1);
%     em(i) = 1;    
%     tv = Info \ em;
%     Sigma0(i) = tv(i);
% end
    