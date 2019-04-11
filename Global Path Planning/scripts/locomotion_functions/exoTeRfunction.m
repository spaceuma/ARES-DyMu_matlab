%% ARES Path Planning %%

function [costMap,locMap,soilMap] = exoTeRfunction(elevationMap, soilMap)

    costMap = ones(size(elevationMap))*inf;
    locMap = zeros(size(elevationMap));
    
    for j = 2:size(soilMap,1)-1
        for i = 2:size(soilMap,2)-1
            switch(soilMap(j,i))
                case 0 % Obstacle
                    costMap(j,i) = inf;
                    locMap(j,i) = 0; % No Locomotion
                case 1 % Soil #1
                    costMap(j,i) = 0.088;
                    if(costMap(j,i)>0.236)
                        locMap(j,i) = 2;
                        costMap(j,i) = 0.236;
                    else
                        locMap(j,i) = 1;
                    end
                case 2 % Soil #2
                    costMap(j,i) = 0.236;
                    locMap(j,i) = 2; % Wheel-walking
                case 3
                    costMap(j,i) = 0.472;
                    locMap(j,i) = 1;
            end
        end
    end
end