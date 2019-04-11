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
% Interpolation of a point within matrix map                              %
%-------------------------------------------------------------------------%

function I = interpolatePoint(point,map)
    i = fix(point(1));
    j = fix(point(2));
    a = point(1) - i;
    b = point(2) - j;
    
    if i == size(map,2)
        if j == size(map,1)
            I = map(j,i);
        else
            I = b*map(j+1,i) + (1-b)*map(j,i);
        end
    else
        if j == size(map,1)
            I = a*map(j,i+1) + (1-a)*map(j,i);
        else
            a00 = map(j,i);
            a10 = map(j,i+1) - map(j,i);
            a01 = map(j+1,i) - map(j,i);
            a11 = map(j+1,i+1) + map(j,i) - map(j,i+1) - map(j+1,i);
            if(a == 0)
                if (b == 0)
                    I = a00;
                else
                    I = a00 + a01*b;
                end
            else
                if (b == 0)
                    I = a00 + a10*a;
                else
                    I = a00 + a10*a + a01*b + a11*a*b;
                end
            end 
        end
    end 
end