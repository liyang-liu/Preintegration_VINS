function [fdes, fp, Ig1] = DetectMonoFeatures(sfrgb1, fMthreshold, nSlevel, cSift)

    %% Detect features
    str=sfrgb1;
    Ig1 = imreadbw(str); 
    Ig1=Ig1-min(Ig1(:)) ;
    Ig1=Ig1/max(Ig1(:)) ;


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
    end
   
    if(cSift == 0)
        uv = vp1.Location;        
    else
        f1 = f1';
        uv = 0 + (vp1(1:2, :))'; %0 From C to Matlab, index 0->1, but not true?! 
 
    end

    
    fp = uv;
    fdes = f1;