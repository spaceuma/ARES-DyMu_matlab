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
% Path Extraction using Gradient Descent Method                           %
%-------------------------------------------------------------------------%

function [gamma,traversedCost,Gx,Gy] = getPathGDM(varargin)
%GETPATHGDM Path Extraction using Gradient Descent Method
%   A path is extracted given the previously computed "totalCostMap", from
%   the waypoint "initWaypoint" to "endWaypoint"; "tau" defines the
%   separation between the generated path waypoints, as a percentage of the
%   map resolution. "Gx" and "Gy" are the gradient maps of
%   "totalCostMap" in the X and Y axes respectively, which reduce the
%   algorithm computational cost if given.
%   USAGE:
%   [gamma,traversedCost,Gx,Gy] = getPathGDM(totalCostMap,initWaypoint,...
%                                            endWaypoint,tau);
%   [gamma,traversedCost,Gx,Gy] = getPathGDM(totalCostMap,initWaypoint,...
%                                            endWaypoint,tau, Gx, Gy);
    switch nargin
        case 4
            totalCostMap = varargin{1};
            initWaypoint = varargin{2};
            endWaypoint = varargin{3};
            tau = varargin{4};
            [Gx,Gy] = calculateGradient(totalCostMap);
        case 6
            totalCostMap = varargin{1};
            initWaypoint = varargin{2};
            endWaypoint = varargin{3};
            tau = varargin{4};
            Gx = varargin{5};
            Gy = varargin{6};
        otherwise
            cprintf('err','Wrong number of inputs provided. Usage:\n')
            cprintf('err','    getPathGDM(totalCostMap,initWaypoint,endWaypoint,tau);\n')
            cprintf('err','    getPathGDM(totalCostMap,initWaypoint,endWaypoint,tau,Gx,Gy);\n')
            error('Wrong number of inputs');
    end
    
    Geval = @(G,x)[interp2(1:size(totalCostMap,1),1:size(totalCostMap,2),Gx,x(1),x(2)), ...
                 interp2(1:size(totalCostMap,1),1:size(totalCostMap,2),Gy,x(1),x(2)) ]; 
    gamma = initWaypoint;
    traversedCost = 0;
    for k=1:15000/tau
        dx = interpolatePoint(gamma(end,:),Gx);
        dy = interpolatePoint(gamma(end,:),Gy);
        if (isnan(dx))||(isnan(dy))||... %In case it is degenerated
            isnan(interpolatePoint(gamma(end,:)-[dx dy],totalCostMap))
            nearN(1) = round(gamma(end,1));
            nearN(2) = round(gamma(end,2));
            while totalCostMap(nearN(2),nearN(1))==inf
                gamma(end,:) = [];
                nearN(1) = round(gamma(end,1));
                nearN(2) = round(gamma(end,2));
            end
            if (~isempty(gamma))
                while (norm(gamma(end,:) - nearN) < 1)
                    gamma(end,:) = [];
                    if (isempty(gamma))
                        break;
                    end
                end
            end
            gamma(end+1,:) = nearN;
            currentT = totalCostMap(nearN(2),nearN(1));
            for i = 1:4
                switch(i)
                    case 1
                        nodeChild = nearN + [0 -1];
                    case 2
                        nodeChild = nearN + [0 1];
                    case 3
                        nodeChild = nearN + [-1 0];
                    case 4
                        nodeChild = nearN + [1 0];
                    case 5
                        nodeChild = nearN + [-1 -1];
                    case 6
                        nodeChild = nearN + [1 1];
                    case 7
                        nodeChild = nearN + [-1 1];
                    case 8
                        nodeChild = nearN + [1 -1];
                end
                if (totalCostMap(nodeChild(2),nodeChild(1)) < currentT)
                    currentT = totalCostMap(nodeChild(2),nodeChild(1));
                    dx = (-nodeChild(1)+nearN(1))/tau;
                    dy = (-nodeChild(2)+nearN(2))/tau;
                end
            end
        end
        if (norm([dx dy]) < 0.01)
            dnx = dx/sqrt(dx^2+dy^2);
            dny = dy/sqrt(dx^2+dy^2);
            gamma(end+1,:) = gamma(end,:) - tau*[dnx,...
                                                 dny];
        else
            gamma(end+1,:) = gamma(end,:) - tau*[dx,...
                                                 dy];
        end     
        if norm(gamma(end,:)-endWaypoint)<1.5
            break;
        end
    end
    gamma(end+1,:) = endWaypoint;
end

