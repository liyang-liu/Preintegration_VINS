function P = pp2v_general(p0, p1, y, pid)    
%     p0 = mean(y(1:2, pid, 1), 2);
%     p1 = mean(y(1:2, pid, 2), 2);
    duvi = y(1:2, pid, 1) - repmat(p0, 1, (length(pid)));
    duvi1 = y(1:2, pid, 2) - repmat(p1, 1, (length(pid)));
%     i = 1;
%     allpi = pid;
%     p0 = allpi(1);%  y(1:2, p0, i+1) - 1 - 1
%     duvi = y(1:2, allpi(1:end), i) - repmat(y(1:2, p0, i), 1, (length(allpi)));
%     duvi1 = y(1:2, allpi(1:end), i+1) - repmat(y(1:2, p0, i+1), 1, (length(allpi)));
    P = leastqr(duvi', duvi1');
    P = P';
end