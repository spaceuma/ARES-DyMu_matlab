function [ Gnx, Gny ] = calculateGradient(cost)
    Gx = cost;
    Gy = cost;
    Gnx = cost;
    Gny = cost;
    for i = 1:size(cost,2)
        for j = 1:size(cost,1)
            if j == 1
                Gy(1,i) = cost(2,i)-cost(1,i);
            else
                if j == size(cost,1)
                    Gy(j,i) = cost(j,i)-cost(j-1,i);
                else
                    if (cost(j+1,i) == Inf)
                        if (cost(j-1,i) == Inf)
                            Gy(j,i) = 0;
                        else
                            Gy(j,i) = cost(j,i)-cost(j-1,i);
                        end
%                         Gy(j,i) = 0;
                    else
                        if (cost(j-1,i) == Inf)
                            Gy(j,i) = cost(j+1,i)-cost(j,i);
%                             Gy(j,i) = 0;
                        else
                            Gy(j,i) = (cost(j+1,i)-cost(j-1,i))/2;
                        end
                    end
                end
            end
            if i == 1
                Gx(j,1) = cost(j,2)-cost(j,1);
            else
                if i == size(cost,2)
                    Gx(j,i) = cost(j,i)-cost(j,i-1);
                else
                    if (cost(j,i+1) == Inf)
                        if (cost(j,i-1) == Inf)
                            Gx(j,i) = 0;
                        else
                            Gx(j,i) = cost(j,i)-cost(j,i-1);
                        end
%                         Gx(j,i) = 0;
                    else
                        if (cost(j,i-1) == Inf)
                            Gx(j,i) = cost(j,i+1)-cost(j,i);
%                             Gx(j,i) = 0;
                        else
                            Gx(j,i) = (cost(j,i+1)-cost(j,i-1))/2;
                        end
                    end
                end
            end
            Gnx(j,i) = Gx(j,i)/sqrt(Gx(j,i)^2+Gy(j,i)^2);
            Gny(j,i) = Gy(j,i)/sqrt(Gx(j,i)^2+Gy(j,i)^2);
        end
    end
end

