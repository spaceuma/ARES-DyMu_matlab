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
%-------------------------------------------------------------------------%

function [Tmap,iterations] = computeTmap(Cmap,goal)
  % 'closed' indicates whether the node is closed (1) or not
    closed = zeros(size(Cmap));
    closed(Cmap == inf) = 1;
    N = sum(closed(:) == 0);
    Tmap = ones(size(Cmap))*inf;
    narrowBand = [];
    Tmap(goal(2),goal(1)) = 0;
    closed(goal(2),goal(1)) = 1;
    iterations = 1;

    [Tmap, narrowBand] = updateNode(goal, Cmap, Tmap, narrowBand, closed);
    % Main Loop
    while(~isempty(narrowBand))
        [nodeTarget, narrowBand] = getMinNB(narrowBand);
        closed(nodeTarget(2),nodeTarget(1)) = 1;
        [Tmap, narrowBand] = updateNode(nodeTarget, Cmap, Tmap,...
                                        narrowBand, closed);  
        iterations = iterations + 1;
        disp('% Completed: ')
        disp(100*iterations/N)
    end
end

