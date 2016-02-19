function [matches] = GetUniqueMatch(matches)

    [UniXY, Index]=unique(matches(:,2),'first');%unique(index_pairs,'rows','first');
    DupIndex=setdiff(1:size(matches,1), Index);
    matches(DupIndex,:)=[];
    
end