function [nodeTarget, narrowBand] = getMinLocalNB(narrowBand, propMap, closed, Tinit, Treach, Tmap, L, offset, ratio)
minT = inf;
pointer = 1;
    for k = 1:size(narrowBand,1)
        if((~isempty(Treach))&&(~isempty(L)))
            interpGlobalT = interpolatePoint((narrowBand(k,1:2)-offset)/ratio + [1 1],Tmap);
            H = max(0,(interpGlobalT-Treach)/(Tinit-Treach)*L);
        else
            H = 0;
        end
        hCost = propMap(narrowBand(k,2),narrowBand(k,1))+H;
        if (hCost < minT) && (closed(narrowBand(k,2),narrowBand(k,1))==0)
            minT = hCost;
            nodeTarget = [narrowBand(k,1),narrowBand(k,2)];
            pointer = k;
        end
    end
    narrowBand(pointer,:) = [];
end