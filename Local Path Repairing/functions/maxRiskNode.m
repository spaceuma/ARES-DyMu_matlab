function [nodeTarget,obstacleList] = maxRiskNode(obstacleList,riskMap)
    
%     if (localExpandableObstacles.empty())
%         return NULL;
    index = 1;
    maxRisk = 0;
    for k = 1:size(obstacleList,1)
        if(riskMap(obstacleList(k,2),obstacleList(k,1)) == 1)
            index = k;
            break;
        else
            if (riskMap(obstacleList(k,2),obstacleList(k,1))>maxRisk)
                index = k;
                maxRisk = riskMap(obstacleList(k,2),obstacleList(k,1));
            end
        end
    end
    nodeTarget = obstacleList(index,:);
    obstacleList(index,:) = [];
end