%% Global Path Planning Simulation for EAAI journal
% ARES: Autonomous Routing on Extreme Surfaces
% University of Malaga, European Space Agency
% Contact info: J. Ricardo Sanchez-Ibanez (ricardosan@uma.es)

addpath(genpath('../functions'));
addpath('locomotion_functions');
addpath('decos_data');

load('decosData.mat');

% Cost Function according to vehicle
% Cmap = costMap = Indicates min. cost required to traverse such node
% Lmap = locomotionMap = Indicates best locomotion to execute
[Cmap1,Lmap] = exoTeRfunction(elevationMap, soilMap);
Cmap2 = exoTeRfunctionDrivingOnly(elevationMap, soilMap);




% Fast Marching Propagation
% - Input data:
%    - start = node where vehicle is
%    - goal = node to reach (node from which FM expansion is executed)
    goal = [88, 60];
    tic
    [Tmap1,iterations1] = computeTmap(Cmap1,goal);
    toc
    [Tmap2,iterations2] = computeTmap(Cmap2,goal);

% FM*
% [Tmap,iterations,Tstart] = calculateTmap(Kmap,goal,start,minC);

  disp('Number of global iterations in case #1 is: ')
    disp(iterations1)
  disp('Number of global iterations in case #2 is: ')
    disp(iterations2)

% Path extraction using Gradient Descent Method
    
    start = [60 110; 40,100; 90,96; 57,56; 110,56];
    
figure(1)
    ax = gca;
    Tmap1scaled = Tmap1*1000/3600;
    [c,h] = contour(Tmap1scaled,0:0.05:3.7); daspect([0.5 0.5 0.1]);
    colormap jet
    for i = 1:size(start,1)
        hold on
            path = getPathGDM(Tmap1scaled,start(i,:),goal,0.4);
            pathWW = path;
            for j = size(pathWW,1):-1:1
                if (interpolatePoint(pathWW(j,:),soilMap) == 1)
                    pathWW(j,:) = [];
                end
            end
            pD = plot(path(:,1),path(:,2),'-b','LineWidth',4);
            if (~isempty(pathWW))
                pWW = plot(pathWW(:,1),pathWW(:,2),'-g','LineWidth',4);
            end
            pStart = plot3(start(i,1),start(i,2),10,'m*','MarkerSize',6,'LineWidth',2);
            t = text(start(i,1),start(i,2),10,['T = ' num2str(Tmap1scaled(start(i,2),start(i,1)),2) 'Wh'],'Interpreter','latex','FontSize',12,'Color','k','EdgeColor','k','BackgroundColor','w');       
        hold off
    end
    hold on
        pGoal = plot3(goal(1),goal(2),10,'r*','MarkerSize',6,'LineWidth',2);
    hold off
    l = legend([h,pD,pWW,pStart,pGoal], 'FM propagation','Driving','Wheel-walking','Initial Position','Goal Position');
    l.Interpreter = 'latex';
    l.FontSize = 12;
    xlim([35 115]), ylim([35 115])
    ax.XTickLabel = {};
    ax.YTickLabel = {};
    grid minor
    c = colorbar;
    c.Label.String = 'Total Cost [Wh]';
    c.Label.Interpreter = 'latex';
    c.Label.FontSize = 12;
    c.TickLabelInterpreter  = 'latex';
    c.Location = 'southoutside';
    
figure(2)
    ax = gca;
    Tmap2scaled = Tmap2*1000/3600;
    [c,h] = contour(Tmap2scaled,0:0.05:3.7); daspect([0.5 0.5 0.1]);
    colormap jet
    for i = 1:size(start,1)
        hold on
            path = getPathGDM(Tmap2scaled,start(i,:),goal,0.4);
            pD = plot(path(:,1),path(:,2),'-b','LineWidth',4);
            pStart = plot3(start(i,1),start(i,2),10,'m*','MarkerSize',6,'LineWidth',2);
            t = text(start(i,1),start(i,2),10,['T = ' num2str(Tmap2scaled(start(i,2),start(i,1)),2) 'Wh'],'Interpreter','latex','FontSize',12,'Color','k','EdgeColor','k','BackgroundColor','w');       

        hold off
    end
    hold on
        pGoal = plot3(goal(1),goal(2),10,'r*','MarkerSize',6,'LineWidth',2);
%         text(goal(1),goal(2)-10,10,'Goal','Interpreter','latex','FontSize',16,'Color','k')
    hold off
    l = legend([h,pD,pStart,pGoal], 'FM propagation','Driving','Initial Position','Goal Position');
    l.Interpreter = 'latex';
    l.FontSize = 12;
    xlim([35 115]), ylim([35 115])
    ax.XTickLabel = {};
    ax.YTickLabel = {};
    grid minor
    c = colorbar;
    c.Label.String = 'Total Cost [Wh]';
    c.Label.Interpreter = 'latex';
    c.Label.FontSize = 12;
    c.TickLabelInterpreter  = 'latex';
    c.Location = 'southoutside';
 
figure(3)
    ax = gca;
    obstacleArea = soilMap; obstacleArea(obstacleArea ~= 0) = inf;
    drivingArea = soilMap; drivingArea(drivingArea ~= 1) = inf;
    wwArea = soilMap; wwArea(wwArea ~= 2) = inf;
    nearArea = soilMap; nearArea(nearArea ~= 3) = inf;
    hold on
        s0 = surf(obstacleArea,'EdgeColor','none'); daspect([1 1 1]), view(2);
        s1 = surf(drivingArea,'EdgeColor','none'); daspect([1 1 1]), view(2);
        s2 = surf(wwArea,'EdgeColor','none'); daspect([1 1 1]), view(2);
        s3 = surf(nearArea,'EdgeColor','none'); daspect([1 1 1]), view(2);
        surf(soilMap,'EdgeColor','none'); daspect([1 1 1]), view(2);
    hold off
    xlim([35 115]), ylim([35 115])
    ax.XTickLabel = {};
    ax.YTickLabel = {};
    grid minor
    colormap summer
    l = legend([s0 s1 s2 s3],'Obstacles',['Compact' newline 'Soil'],['Loose' newline 'Soil'],['Near' newline 'Obstacle']);
    l.Location = 'southoutside';
    l.Interpreter = 'latex';
    l.FontSize = 12;
    l.Orientation = 'horizontal';
    
