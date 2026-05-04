% PLOTTING OPTIONAL WALL RESULTS

data = load('simrun1.mat');
dataStore = data.dataStore;

mapdata = load('PracticeMap2026.mat');

fixedWalls = mapdata.map;
optWalls = mapdata.optWalls;

% Get wall belief if available
if isfield(dataStore, 'wallBelief') && ~isempty(dataStore.wallBelief)
    wallState = dataStore.wallBelief.state;
else
    wallState = zeros(size(optWalls,1),1);
end

figure;
hold on;
grid on;
axis equal;

% Plot fixed walls in solid black
for i = 1:size(fixedWalls,1)
    plot([fixedWalls(i,1), fixedWalls(i,3)], ...
         [fixedWalls(i,2), fixedWalls(i,4)], ...
         'k-', 'LineWidth', 2);
end

% Plot optional walls in dashed black first
for i = 1:size(optWalls,1)
    plot([optWalls(i,1), optWalls(i,3)], ...
         [optWalls(i,2), optWalls(i,4)], ...
         'k--', 'LineWidth', 1.5);
end

% Plot detected-present optional walls in red
for i = 1:size(optWalls,1)
    if wallState(i) == 1
        plot([optWalls(i,1), optWalls(i,3)], ...
             [optWalls(i,2), optWalls(i,4)], ...
             'r-', 'LineWidth', 3);
    end
end

% Plot detected-absent optional walls in green
for i = 1:size(optWalls,1)
    if wallState(i) == -1
        plot([optWalls(i,1), optWalls(i,3)], ...
             [optWalls(i,2), optWalls(i,4)], ...
             'g-', 'LineWidth', 3);
    end
end

% Plot robot trajectory
if isfield(dataStore, 'truthPose') && ~isempty(dataStore.truthPose)
    x = dataStore.truthPose(:,2);
    y = dataStore.truthPose(:,3);

    plot(x, y, 'b-', 'LineWidth', 1.5);
    plot(x(1), y(1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot(x(end), y(end), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
end

% V = dataStore.rrtNodes;
% parent = dataStore.rrtParent;
% path = dataStore.rrtPath;
% plot RRT path
if ~isempty(dataStore.rrtPath)
    plot(dataStore.rrtPath(:,1), dataStore.rrtPath(:,2), 'k-', 'LineWidth', 2);
end

if isfield(dataStore, 'rrtNodes') && ~isempty(dataStore.rrtNodes)

    for k = 1:length(dataStore.rrtNodes)
        V = dataStore.rrtNodes{k};
        parent = dataStore.rrtParent{k};

        for i = 2:size(V,1)
            p = parent(i);
            plot([V(i,1), V(p,1)], ...
                 [V(i,2), V(p,2)], ...
                 'Color', [0.7 0.7 1], 'LineWidth', 0.5);
        end
    end
end

xlabel('x position [m]');
ylabel('y position [m]');
title('Optional Wall Detection Results');

legend('Fixed walls', ...
       'Optional walls', ...
       'Detected optional walls', ...
       'Robot trajectory', ...
       'Start', ...
       'End');

