function plot_path(paths, mapClass, SFCs)
% PLOT_PATH Visualize a path and sfc's ellipses
%   PLOT_PATH(path mapClass, SFCs) creates a figure showing a path through
%   the environment.  
%   SFCs provide functions for drawing ellipses: drawEllipsoids();

persistent map decomps init;
if isempty(init)
    map = mapClass;
    decomps = SFCs;
    init = 1;
end

figure('Name','Animation');

hold on;
for i = 1:size(map.blocks,1)
    block = map.blocks(i, :);
    drawBlock(block);
end

path = paths{1};

% draw Inflated obstacle
for i = 1:size(map.blocks,1)
    margin = map.margin;
    block = map.blocks(i, :);
    block(1,1:3) = block(1,1:3) - margin;
    block(1,4:6) = block(1,4:6) + margin;
    block(7) = max(150, block(7));  block(8) = max(150, block(8));   block(9) = max(150, block(9));     %% set color
    drawBlock(block);
end

if size(path,1) > 0
    % pcshow(path, [0,1,0],'MarkerSize', 0.1);
    % The lower left corner position, width and height. 260 here is exactly 7cm
%     set(gcf,'position', [500 500 260 220]);
%     set(gcf,'position', [500 500 400 400]);   % big pic
%     set(gcf,'position', [500 500 300 300]);   % small pic
    set(gcf,'color','w');
    set(gca,'color','w');
    
    %% Display the unit of x y z
%     xlabel('x length(m)');
%     ylabel('y length(m)');
%     zlabel('z length(m)');
    
end

% Set the axis range
axis_add = 2*map.res(1);
axis([map.boundary.ld(1)-axis_add, map.boundary.ru(1)+axis_add,   ... 
      map.boundary.ld(2)-axis_add, map.boundary.ru(2)+axis_add,   ...
      map.boundary.ld(3)-axis_add, map.boundary.ru(3)+axis_add])

hold on
for j = 1 : size(paths, 2)
    if j == 1
        % JPS's result
%         plot3(paths{j}(:,1), paths{j}(:,2), paths{j}(:,3),'-*','color','k','LineWidth',3);
    elseif j == 2
        % JPS() + simple path() result: 
%         plot3(paths{j}(:,1), paths{j}(:,2), paths{j}(:,3),'-*','color','#A2142F','LineWidth',3);
        plot3(paths{j}(:,1), paths{j}(:,2), paths{j}(:,3),'-*','color','k','LineWidth',3);
    end
end
hold off

set(gca,'DataAspectRatio',[0.1 0.1 0.1]);

% drawEllipsoid = true;
if (exist('drawEllipsoid') && (drawEllipsoid == true))
    decomps{1}.drawEllipsoids();
%     decomps{2}.drawEllipsoids();
end

grid on

view(30, 10);
hold off;

end

function drawBlock(block)
    
    x = [ones(4,1) * block(1); ones(4,1) * block(4)];
    y = [ones(2,1) * block(5); ones(2,1) * block(2); ones(2,1) * block(5); ones(2,1) * block(2)];
    z = [block(3);block(6);block(3);block(6);block(3);block(6);block(3);block(6)];


    vert = [x, y, z];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    c = block(7:9)/255;
    patch('Vertices',vert,'Faces',fac,...
          'FaceVertexCData',hsv(6),'FaceColor',c,'FaceAlpha',.2);

    
    x = [ones(4,1) * block(1); ones(4,1) * block(4)];
    y = [block(2);block(5);block(2);block(5);block(2);block(5);block(2);block(5)];
    z = [ones(2,1) * block(3); ones(2,1) * block(6); ones(2,1) * block(3); ones(2,1) * block(6)];

    vert = [x, y, z];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    c = block(7:9)/255;
    patch('Vertices',vert,'Faces',fac,...
          'FaceVertexCData',hsv(6),'FaceColor',c,'FaceAlpha',.2);
end