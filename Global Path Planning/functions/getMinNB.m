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
% Get the node with lower value of Total Cost from Narrow Band            %
%-------------------------------------------------------------------------%

function [nodeTarget, narrowBand] = getMinNB(narrowBand)
    [~,pointer] = min(narrowBand,[],1);
    nodeTarget = narrowBand(pointer(3),1:2);
    narrowBand(pointer(3),:) = [];
end