function [repairedPath,localProp_conv] = repairPathConservative(path, minIndex, maxIndex, riskMap, offset, ratio)
    
    localCmap = 1+riskMap;
    localCmap(riskMap == 1) = inf;
    catchPos = path(maxIndex,:);
    roverPos = path(minIndex,1:2);
    localCatchPos = round((catchPos - [1 1])*ratio + offset);
    localRoverPos = round((roverPos - [1 1])*ratio + offset);
    [localProp_conv] = computeTmapHeur(localCmap,localRoverPos,localCatchPos,1);
    repairedPath = getPathGDM(localProp_conv,localCatchPos,localRoverPos,0.4);
    repairedPath(:,1) = (repairedPath(:,1)-offset(1))/ratio + 1;
    repairedPath(:,2) = (repairedPath(:,2)-offset(2))/ratio + 1;

end