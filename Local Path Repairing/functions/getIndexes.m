% Function that returns the indexes to certain waypoints in the paht
% minIndex -> points to the waypoint that triggers the local repairing
% maxIndex -> points to waypoint 'reference', which is placed after the
%             obstacle and risky areas.


function [minIndex,maxIndex] = getIndexes(trajectory,riskMap,offset,ratio)
    minIndex = 1;
    maxIndex = 1;
    isBlocked = false;
    k = 1;
    while(k <= length(trajectory))&&(maxIndex == 1)
        i = round(ratio*(trajectory(k,1)-1)+offset(1));
        j = round(ratio*(trajectory(k,2)-1)+offset(2));
        if riskMap(j,i) > 0
            if(~isBlocked)
                isBlocked = true;
                minIndex = k;
            end
        else
        	if (isBlocked)
                if (k>maxIndex)
                	maxIndex = k;
                end
        	end
        end
        k = k+1;
    end
end