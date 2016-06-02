function [] = fnBldRptFeatureListFiles(RptFidSet, nPoses, imgdir, newimgdir)

%imgdir = ['.' filesep 'Whole170R' filesep];
%nPoses = 170;

for(pid=1:nPoses)
    load(sprintf('%sImage%d.mat', imgdir, pid)); 
    [c, idorg, idrpt] = intersect(Image(2:end, 1),RptFidSet);
    Image = [Image(1, :); Image(idorg+1,:)]; % Only repeated features are selected
    Image(2:end, 1) = idrpt; % Update their ids
    save(sprintf('%sImage%d.mat', newimgdir, pid), 'Image');    
end
