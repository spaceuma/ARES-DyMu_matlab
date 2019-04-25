function [travMap,nodeTarget,iterations, eikonalIterations] = computeLocalPropagation(riskMap, init, Tinit, Tcatch, Tmap, Cmap, L, offset, ratio)


closed = zeros(size(riskMap));
closed(riskMap == 1) = 1;
travMap = ones(size(riskMap))*inf;
travMap(init(2),init(1)) = 0;
closed(init(2),init(1)) = 1;
iterations = 1;
eikonalIterations = 0;
nextWaypoint = [1 1];

narrowBand = [];

levelSetFound = 0;

[travMap, narrowBand,levelSetFound,closed,eikonalIterations,nextWaypoint] = updateLocalNode(init(1:2), Cmap, Tmap, riskMap, travMap, narrowBand, closed, Tinit, Tcatch, L, levelSetFound, offset, ratio,eikonalIterations,nextWaypoint);

while(true)
%while(closed(roverPos(2),roverPos(1))==0)
    [nodeTarget, narrowBand] = getMinLocalNB(narrowBand, travMap, closed, Tinit, Tcatch, Tmap, L, offset, ratio);
    closed(nodeTarget(2),nodeTarget(1)) = 1;  %nodeTarget -> closed
    [travMap, narrowBand,levelSetFound,closed,eikonalIterations,nextWaypoint] = updateLocalNode(nodeTarget, Cmap, Tmap, riskMap, travMap, narrowBand, closed, Tinit, Tcatch, L, levelSetFound, offset, ratio,eikonalIterations,nextWaypoint);
    
    % Border expansion
    iterations = iterations + 1;   
    if(levelSetFound)
        if(closed(nextWaypoint(2),nextWaypoint(1)) == 1)
            iterations = iterations + length(narrowBand);
            break;
        end
    end
%     surf(travMap),view(2),xlim([0,size(riskMap,2)]),ylim([0,size(riskMap,1)]),drawnow
    
end
end

