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
% Get the node with lower value of Total Cost (using heuristic) from      %
% Narrow Band                                                             %
%-------------------------------------------------------------------------%

function [nodeTarget, narrowBand] = getMinNBHeur(narrowBand, workMap, closed, start, minC)
minT = inf;
pointer = 1;
    for k = 1:size(narrowBand,1)     %Ahora vamos a buscar el nodo abierto con menor coste
        if((~isempty(start))&&(~isempty(minC)))
            H = norm(start-[narrowBand(k,1),narrowBand(k,2)])*minC;
        else
            H = 0;
        end
        hCost = workMap(narrowBand(k,2),narrowBand(k,1))+H;
        if (hCost < minT) && (closed(narrowBand(k,2),narrowBand(k,1))==0)
            minT = hCost;
            nodeTarget = [narrowBand(k,1),narrowBand(k,2)];
            pointer = k;
        end
    end
    narrowBand(pointer,:) = [];
end