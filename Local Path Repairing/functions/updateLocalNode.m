function [travMap, narrowBand,levelSetFound,closed,eikonalIterations,nextWaypoint] = updateLocalNode(nodeTarget, Cmap, Tmap, riskMap, travMap, narrowBand, closed, Tinit, Tcatch, L, levelSetFound, offset, ratio,eikonalIterations,nextWaypoint)  
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
            if ~((nodeChild(1)<1)||(nodeChild(2)<1)||(nodeChild(1)>size(riskMap,2))||(nodeChild(2)>size(riskMap,1)))
%                 if(Cmap(round(nodeChild(2)/ratio),round(nodeChild(1)/ratio))==inf)
%                     closed(nodeChild(2),nodeChild(1)) = 1;
%                 else
                    % CHECK if it is a bridge!
%                     if(isBridge(nodeChild, riskMap))
%                         closed(nodeChild(2),nodeChild(1)) = 1;
%                     end
                    if (~closed(nodeChild(2),nodeChild(1)))
                        if (nodeChild(2)==1)
                            T1 = travMap(nodeChild(2)+1,nodeChild(1));
                        else
                            if (nodeChild(2)==size(riskMap,1))
                                T1 = travMap(nodeChild(2)-1,nodeChild(1));
                            else
                                T1 = min(travMap(nodeChild(2)-1,nodeChild(1)),travMap(nodeChild(2)+1,nodeChild(1)));
                            end
                        end
                        if (nodeChild(1)==1)
                            T2 = travMap(nodeChild(2),nodeChild(1)+1);
                        else
                            if (nodeChild(1)==size(riskMap,2))
                                T2 = travMap(nodeChild(2),nodeChild(1)-1);
                            else
                                T2 = min(travMap(nodeChild(2),nodeChild(1)-1),travMap(nodeChild(2),nodeChild(1)+1));
                            end
                        end
                        globalNodeChild = (nodeChild-offset)/ratio + [1 1];
                        interpGlobalT = interpolatePoint(globalNodeChild,Tmap);
    %                     C = max(0,(interpGlobalCost-wCatch)/wCatch) + riskMap(nodeChild(2),nodeChild(1)) + 0.1;
                        if ((interpGlobalT-Tcatch <= 0)&&(riskMap(nodeChild(2),nodeChild(1)) == 0)&&(levelSetFound == 0))
                            levelSetFound = 1;
                            nextWaypoint = nodeChild;
                        end
                        C = (1 + riskMap(nodeChild(2),nodeChild(1)))/ratio;
                        T = propagationFunctionLocal(T1,T2,C);
                        eikonalIterations = eikonalIterations + 1;
                        if(travMap(nodeChild(2),nodeChild(1))==inf)
                            travMap(nodeChild(2),nodeChild(1)) = T;
                            narrowBand = [narrowBand;nodeChild];
                        else
                            if (T < travMap(nodeChild(2),nodeChild(1)))
                                travMap(nodeChild(2),nodeChild(1)) = T;
                            end
                        end    
                    end  
%                 end
            end
    end   % Finished looking for children nodes 
end