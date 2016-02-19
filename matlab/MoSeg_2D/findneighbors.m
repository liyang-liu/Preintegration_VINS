function [nf, pos, nvalues,dr] = findneighbors(img, imsk, ci, cj,dr, nnb)
%%


% nf: the number of found points
% pos: the (u,v)s of the found points
% nvalues: feature flow values of the corresponding points
% dr: the final dr


[mr, mc] = size(imsk);
% dr = 1;
nf = 0;
%nnb = 1;%50;%20;%100;%
% Increase dr to ensure that we can find at least nnb neigubouring points
while(nf < nnb)
    % Adjust the bound to ensure 2*dr range and not out of the image
    if((cj < dr) || (cj == dr))
        hl = 1;
        hr = hl + 2*dr;
        if(hr > mc)
            hr = mc;
        end        
    else    
        if((cj+dr > mc))
            hr = mc;
            hl = mc - 2*dr;
        else
            hl = cj - dr;
            hr = cj + dr;
        end
        if(hl < 1)
            hl = 1;
        end        
    end

    if((ci < dr) || (ci == dr))
        vl = 1;
        vr = vl + 2*dr;
        if(vr > mr)
            vr = mr;
        end
    else    
        if((ci+dr > mr))
            vr = mr;
            vl = mr - 2*dr;
        else
            vl = ci - dr;
            vr = ci + dr;
        end
        if(vl < 1)
            vl = 1;
        end
    end
    
    % Count available points in this area
    nf = sum(sum(imsk(vl:vr, hl:hr)))-1;
    dr = dr + 1;
end

nvalues = []; pos = [];
for(i = vl:vr)
    for(j= hl:hr)
        if((imsk(i,j) ~= 0) && ((i ~= ci) || (j ~= cj)))
            nvalues = [nvalues, img(i,j)];
            pos = [pos; j, i];
        end
    end
end
