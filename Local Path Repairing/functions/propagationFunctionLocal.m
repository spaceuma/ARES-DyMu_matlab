function W = propagationFunctionLocal( Wx, Wy, fs)

%     switch(soil)
%         case 0 
%             fs = 1000;
%         case 1
%             fs = 1/max([0.1-0.1/15*slope*180/pi,2/30-2/30/20*slope*180/pi,0.01]);
%         case 2
%             fs = 1/0.05;
%     end
    if Wx == inf
        W = Wy + fs;
    else
        if Wy == inf
            W = Wx + fs;
        else
            if fs < abs(Wx-Wy)
                W = min(Wx,Wy) + fs;
            else
                W = (Wx+Wy+sqrt(2*fs^2-(Wx-Wy)^2))/2;
            end
        end
    end
end

