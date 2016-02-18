function [nerr] = Calpp2vErr_general(p0, p1, P, y, epsf)
    epsf = 1e-6;
%     p0 = mean(y(1:2, allpi, 1), 2);
%     p1 = mean(y(1:2, allpi, 2), 2);length(allpi)              
    duvi = y(1:2, :, 1) - repmat(p0, 1, size(y, 2));
    duvi1 = y(1:2, :, 2) - repmat(p1, 1, size(y, 2));
    Mduvi1 = P*duvi;
    err = P*duvi - duvi1;
    nerr = zeros(1,size(y, 2));
    for(t = 1:size(y, 2))
%         nerr(t) = norm(err(:, t));
        nerr(t) = norm(duvi1(:,t) - Mduvi1(:,t));%/(.5*(norm(duvi1(:,t)) + norm(Mduvi1(:,t))) + epsf); 
%         nerr(t) = norm(duvi1(:,t) - Mduvi1(:,t))/(.5*(norm(duvi1(:,t)) + norm(Mduvi1(:,t))) + epsf); 
    end   

end