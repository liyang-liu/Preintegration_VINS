function [Zobs, nPoses, nPts, PosFt_mat] = fnRdFeatureFromPBAfile(sFileFullPath)

fid = fopen(sFileFullPath, 'r');
tl = fgetl(fid);

PosFt_u = zeros(200, 200);
PosFt_v = zeros(200, 200);
PosFt_mat = zeros(200, 200);
idft = 0;
while(ischar(tl))
    idft = idft + 1;
    nFtTimes = int32(str2num(tl(1)));
    for(k=1:nFtTimes)
        ftrpts = sscanf(tl(3:end), '%d %f %f');
        PosFt_mat(idft, ftrpts(:,1)+1) = 1;
        PosFt_u(idft, ftrpts(:,1)+1) = ftrpts(:,2);
        PosFt_v(idft, ftrpts(:,1)+1) = ftrpts(:,3);
    end
end
fclose(fid);

nPts = max(PosFt_u(:, 1));
nPoses = max(PosFt_u(:, 2));
idz = 1;
for(pid=1:nPoses)
   idf = find(PosFt_mat(:, pid) == 1);
   nlen = length(idf) * 2;
   tv = [osFt_u(idf, 2), PosFt_v(idf, 2)]';
   Zobs(idz:(idz + nlen - 1)) = tv(:);
end
