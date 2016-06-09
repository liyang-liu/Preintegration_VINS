function [ RptFidSet, RptFeatureObs, PBAFeature ] = fn_GetUniqueFeatures( FeatureObs, nPoses, fMaxDistance )
    global PreIntegration_options Data_config
    
    if(PreIntegration_options.bMalaga == 1)
        
        load([ Data_config.DATA_DIR 'PBAFeature.mat']);
        %RptFidSet = find(FeatureObs(:, nObsId_FeatureObs) >= min(nPoseNew, nMinObsTimes));
        %RptFidSet = intersect(RptFidSet, find(abs(PBAFeature(:,3)) < fMaxDistance));
        %RptFeatureObs = FeatureObs(RptFidSet, :);
        RptFidSet = find( [FeatureObs(:).nObs] > 1);
        RptFidSet = RptFidSet(:);
        RptFidSet = intersect( RptFidSet, find( abs( PBAFeature(:,3)) < fMaxDistance ) );
        RptFeatureObs = FeatureObs(RptFidSet);
        
    elseif(PreIntegration_options.bDinuka == 1)
        
        RptFidSet = find( [FeatureObs(:).nObs] >= min(nPoses, PreIntegration_options.nMinObsTimes));
        RptFidSet = RptFidSet(:);
        RptFeatureObs = FeatureObs(RptFidSet);
        
    end
