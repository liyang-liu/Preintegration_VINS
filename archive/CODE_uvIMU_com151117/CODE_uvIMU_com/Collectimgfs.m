function [currentfeatures,Ig1] = Collectimgfs(sfrgb1, sfd1, nPose, dfactor, K, ...
                            fMthreshold, nSlevel, cSift, cDataset, dmin, dmax)

    %% Detect features
    str=sfrgb1;
    Ig1 = imreadbw(str);
%     Ig1 = rgb2gray(I1);
%     
%     Ig1=Ig1-min(Ig1(:)) ;
%     Ig1=double(Ig1)/double(max(Ig1(:))) ;
%     Ig1=imsmooth(Ig1, .1) ;    

    if(cSift == 0)
        pts1 = detectSURFFeatures(Ig1, 'MetricThreshold', fMthreshold, 'NumScaleLevels', nSlevel);%500.);%%detectMSERFeatures
%         pts1 = pts1.selectStrongest(300);%200
        [f1, vp1] = extractFeatures(Ig1, pts1);    
    elseif(cSift == 1)
        [vp1, f1, ~,dogss1] = sift(Ig1);%, 'threshold', 0.02);%, 'Verbosity', 1);
        %% Figures for debug
        figure(11) ; clf ; plotss(dogss1) ; colormap gray ;
        drawnow ;
        figure(12) ; clf ;
        imagesc(Ig1) ; colormap gray ;
        hold on ;
        h=plotsiftframe( vp1 ) ; set(h,'LineWidth',2,'Color','g') ;
        h=plotsiftframe( vp1 ) ; set(h,'LineWidth',1,'Color','k') ;
        %%
%         [~, I]=unique(vp1(1,:),'first');
%         vp1=vp1(:,I);
%         f1=f1(:,I);
     
    end
    % Read depth
    str=sfd1;%sprintf('../segDataset/data%d/depth%04d.png', ndg, ni0);%'.//depth%04d.png', ni0);%Cr = 98mm, Hl=50mm
    d1 = imread(str);
    if((cDataset == 2))%(cDataset == 1) || 
        d1 = bitor(bitshift(d1,-3), bitshift(d1,3-16));
    end
    d1 = double(d1)/dfactor;
    
    [r,c]=size(d1);    
    if(cSift == 0)
        uv = vp1.Location;        
%         u1f = uv(:,1);
%         v1f = uv(:,2);
%         u1 = round(u1f);
%         v1 = round(v1f);

    else
        f1 = f1';
        uv = 1 + (vp1(1:2, :))'; % From C to Matlab, index 0->1       
%         u1f = uv(:,1);
%         v1f = uv(:,2);   
%         u1 = round(uv(:,1));
%         v1 = round(uv(:,2));     
    end
    u1f = uv(:,1);
    v1f = uv(:,2);
    u1 = round(u1f);
    v1 = round(v1f);   
%             figure; imshow(I1);
%             hold on; 
%             plot(u1f,v1f,'rs','LineWidth',2,'MarkerEdgeColor', 'r', 'MarkerFaceColor','r', 'MarkerSize',15);
    
    nz = ((u1-1)*r+v1);%int32
    z1 = d1(nz);
    
    idx = find(z1 <dmin);%Get rid of zero point
    z1(idx) = [];
    u1f(idx) = [];
    v1f(idx) = [];
%     u1(idx) = [];
%     v1(idx) = [];
    
    f1(idx, :) = [];   
    
    idx = find(z1 > dmax);%Get rid of zero point
    z1(idx) = [];
    u1f(idx) = [];
    v1f(idx) = [];
%     u1(idx) = [];
%     v1(idx) = [];double(u1), double(v1),     
    f1(idx, :) = [];  
    
    nf = size(f1, 1);
    
    fx = K(1,1);
    cx = K(1,3);
    fy = K(2,2);
    cy = K(2,3);
    
    x1 = (double(u1f) - cx) .*z1/fx;
    y1 = (double(v1f) - cy) .* z1/fy;
    p1 = [x1, y1, z1];
    
    currentfeatures = [ones(nf,1)*nPose, (1:nf)', u1f, v1f, p1, f1]; %
        