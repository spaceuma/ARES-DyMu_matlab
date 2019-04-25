function [localProp_swe,globalPath,Treference,repairedPath] = ...
    repairPathSweeping(trajectory, minIndex, maxIndex, riskMap, Tmap,...
                       Cmap, offset, ratio)
                   
    goal = trajectory(end,:);      
    Treference = interpolatePoint(trajectory(maxIndex,1:2),Tmap);
    Tinit = interpolatePoint(trajectory(minIndex,1:2),Tmap);
    
    L = 0; %Length of path between indexed waypoints
    for i = (minIndex+1):maxIndex
        L = L + norm(trajectory(i,1:2)-trajectory(i-1,1:2));
    end
    
    lastWaypoint = [round((trajectory(minIndex,1:2) - [1 1])*ratio + offset) 0]; 
    tic
            [localProp_swe,nextWaypoint,localIterations, eikonalIterations] =...
                computeLocalPropagation(riskMap, lastWaypoint, Tinit,...
                Treference, Tmap, Cmap, L, offset, ratio);
    toc
    disp('New Repairing')
    disp('  Number of local iterations is: ')
    disp(localIterations)
    disp('  Number of eikonal iterations is: ')
    disp(eikonalIterations)
    localPath = getPathGDM(localProp_swe,nextWaypoint(1:2),lastWaypoint(1:2),0.4);
    repairedPath = flip(localPath);
    repairedPath(:,1) = (repairedPath(:,1)-offset(1))/ratio + 1;
    repairedPath(:,2) = (repairedPath(:,2)-offset(2))/ratio + 1;
    repairedPath(1,:) = []; %First element is redundant
    nextGlobalWaypoint = (nextWaypoint-offset)/ratio+[1 1];
    globalPath = getPathGDM(Tmap,nextGlobalWaypoint(1:2),goal(1:2),0.4);
end