function riskMap = expandRisk(obstacleList,riskMap,ratio,offset, riskDistance)
    iterations = 0;
    eikonaliterations = 0;
    while(~isempty(obstacleList))
        [nodeTarget,obstacleList] = maxRiskNode(obstacleList,riskMap);
        iterations = iterations + 1;
        for i=1:4
            switch(i)
                case 1
                    nodeChild = nodeTarget + [0 -1];
                case 2
                    nodeChild = nodeTarget + [0 1];
                case 3
                    nodeChild = nodeTarget + [-1 0];
                case 4
                    nodeChild = nodeTarget + [1 0];
            end 
            if((nodeChild(1)>0)&&(nodeChild(2)>0)&&(nodeChild(1)<=size(riskMap,2))&&(nodeChild(2)<=size(riskMap,1))&&(riskMap(nodeChild(2),nodeChild(1)) < 1))
                if (nodeChild(2)==1)
                    Ry = riskMap(nodeChild(2)+1,nodeChild(1));
                else if (nodeChild(2)== size(riskMap,1))
                        Ry = riskMap(nodeChild(2)-1,nodeChild(1));
                    else
                        Ry = max(riskMap(nodeChild(2)-1,nodeChild(1)),riskMap(nodeChild(2)+1,nodeChild(1)));
                    end
                end
                
                if(nodeChild(1) == 1)
                    Rx = riskMap(nodeChild(2),nodeChild(1)+1);
                else if (nodeChild(1)== size(riskMap,2))
                        Rx = riskMap(nodeChild(2),nodeChild(1)-1);
                    else
                        Rx = max(riskMap(nodeChild(2),nodeChild(1)-1),riskMap(nodeChild(2),nodeChild(1)+1));
                    end
                end
                
                Sx = 1-Rx;
                Sy = 1-Ry;
                
                globalPos = round((nodeChild-offset)/ratio+[1 1]);
                
                C = 1/(ratio*riskDistance);
                
                if (abs(Sx-Sy) < C)
                    S = (Sx+Sy+sqrt(2*C^2 - (Sx-Sy)^2))/2;
                else
                    S = min(Sx,Sy) + C;
                end
                R = max(1-S,0);
                if(R > riskMap(nodeChild(2),nodeChild(1)))
                    riskMap(nodeChild(2),nodeChild(1)) = R;
                    obstacleList = [obstacleList; nodeChild];
                end
                eikonaliterations = eikonaliterations + 1;
            end
        end
    end
end