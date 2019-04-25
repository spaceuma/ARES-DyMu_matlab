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
% Function to compute T values using Fast Marching propagation            %
% Input:                                                                  %
% - Cmap = Matrix of C values (cost values)                               %
% - goal = coordinates of the point from which the wave is expanded       %
% - start = (optional) current position of vehicle                        %
%     -> if not empty, propagation stops at vehicle position              %
% - H = (optional) heuristic value, usually the least value of C in Cmap  %
%     -> if not empty, heuristic Fast Marching is used                    %
%-------------------------------------------------------------------------%


function [Tmap,iterations,Tstart] = computeTmapHeur(Cmap,goal,start,H)

    closed = zeros(size(Cmap));
    closed(Cmap == inf) = 1;
    Tmap = ones(size(Cmap))*inf;
    Tmap(goal(2),goal(1)) = 0;
    closed(goal(2),goal(1)) = 1;
    iterations = 1;
    Tstart = 0;

    narrowBand = [];

    [Tmap, narrowBand] = updateNode(goal, Cmap, Tmap, narrowBand, closed);
    
    while(~isempty(narrowBand))
        [nodeTarget, narrowBand] = getMinNBHeur(narrowBand, Tmap, closed, start, H);
        closed(nodeTarget(2),nodeTarget(1)) = 1;
        [Tmap, narrowBand] = updateNode(nodeTarget, Cmap, Tmap, narrowBand, closed);
        iterations = iterations + 1;
        if ((~isempty(start))&&(closed(start(2),start(1)) == 1))
            Tstart = Tmap(start(2),start(1));
            break;
        end
    end
end

