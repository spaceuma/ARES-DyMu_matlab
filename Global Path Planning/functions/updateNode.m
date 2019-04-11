%% _______________________________ARES__________________________________ %%
%                Autonomous Routing on Extreme Terrains                   %
%             University of Malaga - European Space Agency                %
%                   Author: J. Ricardo Sanchez Ibañez                     %
%                       E-mail: ricardosan@uma.es                         %
%                             Supervisors:                                %
%                     Carlos J. Pérez del Pulgar (UMA)                    %
%                          Martin Azkarate (ESA)                          %
%-------------------------------------------------------------------------%
% Description:                                                            %
% Update of Total Cost values of nodeTarget neighbours                    %
%-------------------------------------------------------------------------%

function [totalCostMap, narrowBand] = updateNode(nodeTarget, costMap, totalCostMap, narrowBand, closed,start,minC)    
    for i = 1:4
            switch(i)
                case 1
                    nodeChild = nodeTarget + [0 -1];
                case 2
                    nodeChild = nodeTarget + [0 1];
                case 3
                    nodeChild = nodeTarget + [-1 0];
                case 4
                    nodeChild = nodeTarget + [1 0];
            end   
            if ~((nodeChild(1)<2)||(nodeChild(2)<2)||(nodeChild(1)>size(costMap,2)-1)||(nodeChild(2)>size(costMap,1)-1))
                obstacleNeighbors = 0;
                if (costMap(nodeChild(2),nodeChild(1)+1) == inf)
                    obstacleNeighbors = obstacleNeighbors + 1;
                end
                if (costMap(nodeChild(2),nodeChild(1)-1) == inf)
                    obstacleNeighbors = obstacleNeighbors + 1;
                end
                if (costMap(nodeChild(2)+1,nodeChild(1)) == inf)
                    obstacleNeighbors = obstacleNeighbors + 1;
                end
                if (costMap(nodeChild(2)-1,nodeChild(1)) == inf)
                    obstacleNeighbors = obstacleNeighbors + 1;
                end
                
                if (obstacleNeighbors > 1)
                    closed(nodeChild(2),nodeChild(1)) = 1;
                else
                    if ((~closed(nodeChild(2),nodeChild(1))))%&&(workMap(nodeChild(2),nodeChild(1))==1))
                        if (nodeChild(2)==1)
                            T1 = totalCostMap(nodeChild(2)+1,nodeChild(1));
                        else
                            if (nodeChild(2)==size(costMap,1))
                                T1 = totalCostMap(nodeChild(2)-1,nodeChild(1));
                            else
                                T1 = min(totalCostMap(nodeChild(2)-1,nodeChild(1)),totalCostMap(nodeChild(2)+1,nodeChild(1)));
                            end
                        end
                        if (nodeChild(1)==1)
                            T2 = totalCostMap(nodeChild(2),nodeChild(1)+1);
                        else
                            if (nodeChild(1)==size(costMap,2))
                                T2 = totalCostMap(nodeChild(2),nodeChild(1)-1);
                            else
                                T2 = min(totalCostMap(nodeChild(2),nodeChild(1)-1),totalCostMap(nodeChild(2),nodeChild(1)+1));
                            end
                        end    
                        T = propagationFunctionGlobal(T1,T2,costMap(nodeChild(2),nodeChild(1)));
                        if(totalCostMap(nodeChild(2),nodeChild(1))==inf)
                            totalCostMap(nodeChild(2),nodeChild(1)) = T;
                            narrowBand = [narrowBand ; nodeChild T];
                        else
                            if (T < totalCostMap(nodeChild(2),nodeChild(1)))
                                totalCostMap(nodeChild(2),nodeChild(1)) = T;
                                k = 1;
                                while ((narrowBand(k,1) ~= nodeChild(1)) || (narrowBand(k,2) ~= nodeChild(2)))
                                    k = k+1;
                                end
                                narrowBand(k,3) = T;
                            end
                        end                     
                    end 
                end
            end
    end   % Finished looking for children nodes  
end