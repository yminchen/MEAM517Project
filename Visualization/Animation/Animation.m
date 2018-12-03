function Animation(T,X,tFinal,F_SAVEVID,videoFileName)

% settings
slowmotionMultiplier = 1;
fignum = 1;         % figure number
param = yumingParameters();
sysParam = param.sysParam;

ter_i = param.ter_i;
target_pos = param.target_pos;

%% Preprocessing
% position plot window size
h=figure(fignum); clf
set(h,'Position',[0 0 1000 400]);

%% Animation

% step through each time increment and plot results
if F_SAVEVID
    vidObj = VideoWriter(videoFileName);
    vidObj.FrameRate = length(T)/(tFinal-0)/slowmotionMultiplier;
    open(vidObj);
    F(length(T)).cdata = []; F(length(T)).colormap = []; % preallocate
end

%%%%% Prepare figure %%%%
figure(fignum); clf; 
%%%%% Plot physical world %%%%
hold on
axis equal; 
% boarder of the world
boarderR = max(X(:,1))+1;
boarderL = min(X(:,1))-2;
boarderT = max(X(:,2))+0.5;
axis([boarderL boarderR -0.2 boarderT])
% axis([boarderL 2 -0.1 2])

% Plot ground
edge = Terrain_edge(ter_i);
groundL = floor(boarderL);
groundR = ceil(boarderR);
for i = 1:size(edge,1)+1
    if i == 1
        fill([groundL edge(i,1) edge(i,1) groundL],[Terrain(groundL,ter_i) Terrain(edge(i,1),ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
    elseif i == size(edge,1)+1
        fill([edge(i-1,1) groundR groundR edge(i-1,1)],[Terrain(edge(i-1,1),ter_i) Terrain(groundR,ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
    else
        fill([edge(i-1,1) edge(i,1) edge(i,1) edge(i-1,1)],[Terrain(edge(i-1,1),ter_i) Terrain(edge(i,1),ter_i) -0.1 -0.1],[0 0 0],'EdgeColor',[0 0 0]);
    end
end
% Plot target position
scatter(target_pos,Terrain(target_pos,ter_i),50,'MarkerEdgeColor','b',...
          'MarkerFaceColor','g',...
          'LineWidth',1);
% Time text
text(0.95*boarderL,0.2*boarderT,'elapsed time:','color','k');
% Animation title and label
title('CLF-QP')
xlabel(' (m)')
ylabel(' (m)')
% Skeleton plot =============================
posB = [0 0];
posH = [0 0];
CoGTR = [0 0]; % CoG of right thigh
posKR = [0 0];
CoGSR = [0 0]; % CoG of right shank
posFR = [0 0];
CoGTL = [0 0]; % CoG of left thigh
posKL = [0 0];
CoGSL = [0 0]; % CoG of left shank
posFL = [0 0];
% size of the plot
CoGBodySize = 120-((120-65)/(7-3))*((boarderR-boarderL)-3); 
            % 120 when world width = 3m. 65 when world width = 7m. 
if CoGBodySize<=0.1
    CoGBodySize = 0.1;
end
CoGThighSize = 50-((50-25)/(7-3))*((boarderR-boarderL)-3);  
            % 50 when world width = 3m. 25 when world width = 7m.  
if CoGThighSize<=0.1
    CoGThighSize = 0.1;
end
LinkWidth = 3-((3-2)/(7-3))*((boarderR-boarderL)-3);  
            % 3 when world width = 3m. 2 when world width = 7m.
if LinkWidth<=0.1
    LinkWidth = 0.1;
end

% (Plot left leg first so that right leg can overlap the left.)
% Plot CoG of body
a0 = scatter(posB(1), posB(2),CoGBodySize,'MarkerEdgeColor',[0 0 0],...
          'MarkerFaceColor',[0 0 0],...
          'LineWidth',1.5);
% Plot body link
a1 = plot([posH(1) posB(1)],[posH(2) posB(2)],'k','LineWidth',LinkWidth);
% Plot CoG of left thigh
a6 = scatter(CoGTL(1), CoGTL(2),CoGThighSize,'MarkerEdgeColor',[1 0 0],...
          'MarkerFaceColor',[1 0 0],...
          'LineWidth',1.5);
% Plot left thigh
a7 = plot([posH(1) posKL(1)],[posH(2) posKL(2)],'r','LineWidth',LinkWidth);
% Plot CoG of left shank
a8 = scatter(CoGSL(1), CoGSL(2),CoGThighSize,'MarkerEdgeColor',[1 0 0],...
          'MarkerFaceColor',[1 0 0],...
          'LineWidth',1.5);
% Plot left shin
a9 = plot([posFL(1) posKL(1)],[posFL(2) posKL(2)],'r','LineWidth',LinkWidth);
% Plot CoG of right thigh
a2 = scatter(CoGTR(1), CoGTR(2),CoGThighSize,'MarkerEdgeColor',[0 0 1],...
          'MarkerFaceColor',[0 0 1],...
          'LineWidth',1.5);
% Plot right thigh
a3 = plot([posH(1) posKR(1)],[posH(2) posKR(2)],'b','LineWidth',LinkWidth);
% Plot CoG of right shank
a4 = scatter(CoGSR(1), CoGSR(2),CoGThighSize,'MarkerEdgeColor',[0 0 1],...
          'MarkerFaceColor',[0 0 1],...
          'LineWidth',1.5);
% Plot right shin
a5 = plot([posFR(1) posKR(1)],[posFR(2) posKR(2)],'b','LineWidth',LinkWidth);

% Display time on plot
a10 = text(0.95*boarderL,0.1*boarderT,['' ...
    num2str(T(1),'%1.1f') ' sec'],'color','k');
      
% end of skeleton plot =======================

% This for loop would show animation as well as store the animation in F().
for ti=1:length(T)            

    % plot robot
    posB = posBody(X(ti,1:7)',sysParam);
    posH = posHip(X(ti,1:7)',sysParam);
    CoGTR = CoGThighR(X(ti,1:7)',sysParam);
    posKR = posKneeR(X(ti,1:7)',sysParam);
    CoGSR = CoGShankR(X(ti,1:7)',sysParam);
    posFR = posFootR(X(ti,1:7)',sysParam);
    CoGTL = CoGThighL(X(ti,1:7)',sysParam);
    posKL = posKneeL(X(ti,1:7)',sysParam);
    CoGSL = CoGShankL(X(ti,1:7)',sysParam);
    posFL = posFootL(X(ti,1:7)',sysParam);
        
    % Plot CoG of body
    set(a0,'XData',posB(1),'YData',posB(2));
    % Plot body link
    set(a1,'XData',[posH(1) posB(1)],'YData',[posH(2) posB(2)]);
    % Plot CoG of right thigh
    set(a2,'XData',CoGTR(1),'YData',CoGTR(2));
    % Plot right thigh
    set(a3,'XData',[posH(1) posKR(1)],'YData',[posH(2) posKR(2)]);
    % Plot CoG of right shank
    set(a4,'XData',CoGSR(1),'YData',CoGSR(2));
    % Plot right shin
    set(a5,'XData',[posFR(1) posKR(1)],'YData',[posFR(2) posKR(2)]);
    % Plot CoG of left thigh
    set(a6,'XData',CoGTL(1),'YData',CoGTL(2));
    % Plot left thigh
    set(a7,'XData',[posH(1) posKL(1)],'YData',[posH(2) posKL(2)]);
    % Plot CoG of left shank
    set(a8,'XData',CoGSL(1),'YData',CoGSL(2));
    % Plot left shin
    set(a9,'XData',[posFL(1) posKL(1)],'YData',[posFL(2) posKL(2)]);
    
    % Display time on plot
    set(a10,'String',[num2str(T(ti),'%1.1f') ' sec']);
    
    % Save frames for animaiton
    F(ti) = getframe(h,[0 0 1000 400]); 
    if F_SAVEVID
        writeVideo(vidObj,F(ti));
    end
    
end

if F_SAVEVID
    close(vidObj);
end







