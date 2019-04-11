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
% Eikonal equation resolution                                             %
%-------------------------------------------------------------------------%


function [T] = propagationFunctionGlobal( Tx, Ty, C)
    if Tx == inf
        T = Ty + C;
    else
        if Ty == inf
            T = Tx + C;
        else
            if C < abs(Tx-Ty)
                T = min(Tx,Ty) + C;
            else
                T = (Tx+Ty+sqrt(2*C^2-(Tx-Ty)^2))/2;
            end
        end
    end
end

