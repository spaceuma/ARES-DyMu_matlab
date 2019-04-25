function [trajectory,localPropagations,minIndex,maxIndex] = evaluatePath(trajectory, ratio, offset, riskMap, riskDistance, Tmap, Cmap,d)
    minIndex = 1;
    maxIndex = 1;
    isBlocked = false;
    k = 1;
    n = 0; %number of local Propagations
    localPropagations = [];
    while(k <= length(trajectory))
        i = round(ratio*(trajectory(k,1)-1)+offset(1));
        j = round(ratio*(trajectory(k,2)-1)+offset(2));
        if ((trajectory(k,3) == 0)&&(riskMap(floor(j),floor(i)) > 0))
            if(~isBlocked)
                isBlocked = true;
                if (k>minIndex)
                    minIndex = k;
                end
            else
                if (k>maxIndex)
                    maxIndex = k;
                end
            end
        else
            if ((trajectory(k,3) == 1)&&(riskMap(floor(j),floor(i)) == 1))
                kk = k;
                while(trajectory(kk,3) == 1)
                    kk = kk-1;
                end
                if(~isBlocked)
                    isBlocked = true;
                    if (k>minIndex)
                        minIndex = kk;
                    end
                else
                    if (k>maxIndex)
                        maxIndex = kk;
                    end
                end
            else
                if ((trajectory(k,3) == 0)&&(isBlocked))
                    if (k>maxIndex)
                        maxIndex = k;
                    end
                    [trajectory,localMap] = repairPath(trajectory, minIndex, maxIndex, riskMap, riskDistance, Tmap, Cmap, offset, ratio,d);
                    n = n+1;
                    localPropagations(:,:,n) = localMap;
                    isBlocked = false;
                    k = minIndex;
                end
            end
        end
        k = k+1;
    end
    if (isBlocked)
        maxIndex = size(trajectory,1);
        [trajectory,localMap] = repairPath(trajectory, minIndex, maxIndex, riskMap, riskDistance, Tmap, Cmap, offset, ratio,d);
        if (~isempty(localMap))
            n = n+1;
            localPropagations(:,:,n) = localMap;
        end
    end
end