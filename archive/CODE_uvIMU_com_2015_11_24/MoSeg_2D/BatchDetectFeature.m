function BatchDetectFeature(imgDir, info2Dir)

addpath(genpath('sift'));
imgfiles = dir(imgDir);
imgfiles = imgfiles(3:end);
nimg = length(imgfiles);

for(i = 1:1:nimg)%10
    k = strfind(imgfiles(i).name, '.');
    s = fullfile(info2Dir, [imgfiles(i).name(1:(k(end)-1)) '.mat']);
    if(exist(s, 'file'))
        continue;
    end
    
    img = imread(fullfile(imgDir, imgfiles(i).name));
    if(size(img, 3) == 3)
        img = rgb2gray(img);
    end
    [vp, f] = sift(img);%, 'threshold', 0.02);%, 'Verbosity', 1);
    [~, I]=unique(vp(1,:),'first');
    vp=vp(:,I);
    f=f(:,I);
    f = f';
    uv = vp(1:2, :)';
    uv = 1 + uv;
%     u = 1 + uv(:,1);
%     v = 1 + uv(:,2);    
    imginfo.featuredescritor = f;
    imginfo.fuv = uv;
    imginfo.imgname = imgfiles(i).name;
%     k = strfind(imgfiles(i).name, '.');
%     s = fullfile(info2Dir, [imgfiles(i).name(1:(k(end)-1)) '.mat']);
    save(s, 'imginfo');
end