%% Global-Local Path Planner for Robotics and Autonomous Systems
% ARES: Autonomous Routing on Extreme Surfaces
% University of Málaga, European Space Agency
% Demo for showing functioning of the local path repairing


% Libraries paths are added
    addpath(genpath('../functions'));
    addpath(genpath('../../Global Path Planning/functions'));

% A global layer with cost 1 for all nodes is created:
    Cmap = ones(100,100);

% Going from (2,2) to (50,50)
    start = [2 2];
    goal = [50 50];

% Total Cost on Global Layer is computed
    [Tmap,iterations] = computeTmap(Cmap,goal);
    
% Get global path
    path = getPathGDM(Tmap,start,goal,0.4);

% Ratio Global Res / Local Res
    ratio = 10;

% Local map offset from global map
    offset = [5.5 5.5]; 
    
% Distance from triggerer waypoint in global units
    d = 0.5;
    
% Local Cost Map initialization   
    localCostMap = ones(size(Cmap)*ratio)*inf;
    
% Local Risk Map
    riskMap = zeros(size(localCostMap));
  % Global Coordinates of obstacle center points
    globalObstacle = [39.8 40; 41 38.5; 40.5 41.5];
  % Obstacle areas definition
    for k = 1:size(globalObstacle,1)
        localObstacle = round(10*(globalObstacle-[1 1]) + offset);
        riskMap(localObstacle(k,2),localObstacle(k,1)) = 1;
    end
    h = fspecial('disk',5);
    riskMap = riskMap + imfilter(riskMap,h,0);
    riskMap(riskMap > 0) = 1;
    obstacleList = [];
    for j = 2:size(riskMap,1)-1
        for i = 2:size(riskMap,2)-1
            if (riskMap(j,i) == 1)
                obstacleList = [obstacleList; [i j]];
            end
        end
    end
  % Risk Expansion
    riskMap = expandRisk(obstacleList,riskMap,ratio,offset,d);
    
  % Get indexes pointing to triggerer and reference waypoints
    [minIndex,maxIndex] = getIndexes(path,riskMap,offset,ratio);
    
  % Make minIndex point to a waypoint from which propagation is started
    i = minIndex;
    while (i>1)
        if (norm(path(i,1:2)-path(minIndex,1:2)) > d)
            minIndex = i;
            break;
        end
        i = i-1;
    end
    
  % Execute repairing using sweeping approach
    [localProp_swe,globalPath1,Treference,repairedPath1] = ...
        repairPathSweeping(path, minIndex, maxIndex, riskMap, Tmap, ...
        Cmap, offset, ratio);

  % Execute repairing using conservative approach
    [repairedPath,localProp_conv] = repairPathConservative(path,...
        minIndex, maxIndex, riskMap, offset, ratio);
    
  
    
% Figures----------------------------------------

    restPath = repairedPath(maxIndex:end,:);
    [xLocal,yLocal] = meshgrid(0.5:0.1:100.4); 

figure(1)
    ax = gca;
    daspect([0.5 0.5 0.1]);
    hold on
        contour(Tmap./20,.5:0.02:1,'LineWidth',1,'LineColor','b');
        for k = 1:size(globalObstacle,1)
            rectangle('Position',[globalObstacle(k,1)-1 globalObstacle(k,2)-1 2 2],'Curvature',[1,1],'FaceColor',[1 1 0],'LineStyle','none');
            rectangle('Position',[globalObstacle(k,1)-0.5 globalObstacle(k,2)-0.5 1 1],'Curvature',[1,1],'FaceColor',[1 0.5 0],'LineStyle','none');
            rectangle('Position',[globalObstacle(k,1)-0.25 globalObstacle(k,2)-0.25 .5 .5],'Curvature',[1,1],'FaceColor','r','LineStyle','none');
        end
        initialPath = plot(path(:,1),path(:,2),'-*b','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','b',...
    'MarkerFaceColor','b');
    hold off
    
    hold on
        rover = [-1 -1; -1 1; 1 1; 1 -1]*0.35;
        alpha = pi/4;
        xrueda1 = 38 + rover*[cos(alpha) -sin(alpha)]';
        yrueda1 = 38 + rover*[sin(alpha) cos(alpha)]';
        fill(xrueda1,yrueda1,'green');
        scatter(NaN,NaN,'filled','o','MarkerFaceColor','r');
        scatter(NaN,NaN,'filled','o','MarkerFaceColor',[1 .5 0]);
        scatter(NaN,NaN,'filled','o','MarkerFaceColor',[1 1 0]);
    hold off
    
    l = legend([newline 'Global' newline 'Propagation'], [newline 'Path' newline],...
        [newline 'Rover'], [newline 'Detected' newline 'Obstacles'], [newline 'Dilated' newline 'Obstacle' newline 'Area'],...
        [newline 'Risky Area']);
    l.FontSize = 9;
    l.Interpreter = 'latex';
    ax.XGrid = 'on';
    ax.YGrid = 'on';
    ax.XTick = 0.5:1:99.5;
    ax.YTick = 0.5:1:99.5;
    ax.XMinorTick = 'on'; ax.YMinorTick = 'on';
    ax.XMinorGrid = 'on'; ax.YMinorGrid = 'on';
    ax.XAxis.MinorTickValues = 0.5:0.1:99.5; 
    ax.YAxis.MinorTickValues = 0.5:0.1:99.5;
    ax.MinorGridLineStyle = '-';
    ax.MinorGridColor = 'k';
    ax.MinorGridAlpha = 0;
    ax.GridColor = 'k';
%     ax.GridAlpha = 0.8;
    ax.XTickLabel = {};
    ax.YTickLabel = {};
    xlim([37 43]);
    ylim([37 43]);

figure(3)
    ax = gca;
    rMap = riskMap;
    riskPlot = surf(xLocal,yLocal,riskMap,'EdgeAlpha',.15);
    hold on
        plot3(path(:,1),path(:,2),ones(size(path,1),1),'-*c','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','c',...
    'MarkerFaceColor','c');
    hold off
    colormap(flipud(hot))
    view(2)
    xlim([37 43]);
    ylim([37 43]);
    
    ax.XTickLabel = {};
    ax.YTickLabel = {};
    daspect([0.5 0.5 0.1]);
    c = colorbar;
    c.Label.String = 'Risk Index';
    c.Label.Interpreter = 'latex';
    c.Location = 'west';
    c.Ticks = [0 1];
    ax.XMinorTick = 'on'; ax.YMinorTick = 'on';
    ax.XMinorGrid = 'on'; ax.YMinorGrid = 'on';
    ax.XAxis.MinorTickValues = 0.5:0.1:99.5; 
    ax.YAxis.MinorTickValues = 0.5:0.1:99.5;
    ax.MinorGridAlpha = 0;
    ax.GridAlpha = 0;
    grid off
    
figure(2)
    ax = gca;
    riskPlot = surf(xLocal+0.05,yLocal+0.05,-riskMap,'EdgeAlpha',.15,'HandleVisibility','off');
    colormap hot
    
    hold on
        contour(Tmap./20,.5:0.02:1,'LineWidth',1,'LineColor','b');
        localPath = repairedPath;
        contour(xLocal+0.05,yLocal+0.05,localProp_conv./100,40,'LineColor','g')
        contour(xLocal+0.05,yLocal+0.05,localProp_swe./100,40,'LineColor','m')
        surf(xLocal+0.05,yLocal+0.05,localProp_conv./100,'FaceColor','g','FaceAlpha',.15,'EdgeAlpha',0,'HandleVisibility','off');
        surf(xLocal+0.05,yLocal+0.05,localProp_swe./100,'FaceColor','m','FaceAlpha',.15,'EdgeAlpha',0,'HandleVisibility','off');
        initialPath = plot(path(:,1),path(:,2),'-*c','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','c',...
    'MarkerFaceColor','c');
    hold off
    hold on
        plot3(localPath(:,1),localPath(:,2),ones(size(localPath,1),1),'-*g','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
        plot3(repairedPath1(:,1),repairedPath1(:,2),ones(size(repairedPath1,1),1),'-*m','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','m',...
    'MarkerFaceColor','m');
        plot(restPath(:,1),restPath(:,2),'-*b','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','b',...
    'MarkerFaceColor','b');
        plot(globalPath1(:,1),globalPath1(:,2),'-*','Color',[1 .5 0],'LineWidth',2,'MarkerSize',5,'MarkerEdgeColor',[1 .5 0],...
    'MarkerFaceColor',[1 .5 0]);
    hold off
    view(2)
    daspect([1 1 1]);
    caxis([-1 0])
    hold on
        contour(Tmap,[Treference Treference],'--','LineWidth',1,'LineColor',[1 .5 0]);
    hold off
    ax.XTickLabel = {};
    ax.YTickLabel = {};
    xlim([37 43]);
    ylim([37 43]);
    l = legend([newline 'T Global' newline 'Propagation'],...
        [newline '$\Psi_1$ Local' newline 'Propagation'],...
        [newline '$\Psi_2$ Local' newline 'Propagation'],...
        [newline 'Path To Be' newline 'Repaired'],...
        [newline '\#1 Local' newline 'Waypoints'],...
        [newline '\#2 Local' newline 'Waypoints'],...
        [newline 'Old Global' newline 'Waypoints'],...
        [newline 'New Global' newline 'Waypoints'],...
        [newline '$T_{reach}$']);
    l.FontSize = 9;
    l.Interpreter = 'latex';
    
    
