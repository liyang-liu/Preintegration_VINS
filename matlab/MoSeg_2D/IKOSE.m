function [h] = IKOSE(normr, K)
    normr = sort(normr);%, 'descend'
    E = 1;%2.5;%
    sKJ = 1;
    nrJ = length(normr);
    k= K/nrJ;
%     nmk = norminv(0.5*(1+k), 0, 1);
%     sKJ = normr(K)/nmk;
    sKJ0 = sKJ - 1;    
    while((sKJ0 ~= sKJ) && (k < 1))        
        sKJ0 = sKJ;
        nmk = norminv(0.5*(1+k), 0, 1);
        sKJ = normr(K)/nmk;        
        idxr = find(normr < (E * sKJ));
        nrJ = length(idxr);
        k= K/nrJ; 
    end
    
    h = sKJ;
    