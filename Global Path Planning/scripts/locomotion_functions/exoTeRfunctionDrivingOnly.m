%% ARES Path Planning %%

function [costMap,locMap] = exoTeRfunctionDrivingOnly(elevationMap, soilMap)

    costMap = ones(size(elevationMap))*inf;
    locMap = zeros(size(elevationMap));
    
    for j = 1:size(soilMap,1)
        for i = 1:size(soilMap,2)
            switch(soilMap(j,i))
                case 0 % Obstacle
                    costMap(j,i) = inf;
                    locMap(j,i) = 0; % No Locomotion
                case 1 % Soil #1
                      costMap(j,i) = 0.088;
                      locMap(j,i) = 1; % Normal Driving
                case 2 % Soil #2
                    costMap(j,i) = 1.074;
                    locMap(j,i) = 0; % No Locomotion
                case 3
                    costMap(j,i) = 2.148;
                    locMap(j,i) = 1;
            end
        end
    end
end