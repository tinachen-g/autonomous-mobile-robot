function [path, V, parent] = buildRRT(map, mapBoundary, start, goal, robot_radius)
% BUILD RRT POINT: Builds an RRT for a point robot and returns a 
% collision-free path
%
% INPUTS:
%   map_file    text file containing polygon obstacles
%   map_bounds  [xmin ymin xmax ymax]
%   start       [x y]
%   goal        [x y]
%
% OUTPUT:
%   path        nx2 list of waypoints from start to goal

% parameters
step_size = 0.2;
max_nodes = 10000;
goal_tol = 0.5;
tol = 1e-9;

% load map
mapfile = dlmread(map);

% convert map rows into obstacle cell array
obstacles = {};
for i = 1:size(mapfile,1)
    vertices = [];
    for j = 1:2:size(mapfile,2)
        vert_x = mapfile(i,j);
        vert_y = mapfile(i,j+1);

        if vert_x == 0 && vert_y == 0
            break;
        end

        vertices = [vertices; vert_x vert_y];
    end
    obstacles{end+1} = vertices;
end

% build obstacle edge list
edges = [];
for k = 1:length(obstacles)
    vertices = obstacles{k};
    for n = 1:size(vertices,1)
        p = mod(n,size(vertices,1)) + 1;
        x1 = vertices(n,1);
        y1 = vertices(n,2);
        x2 = vertices(p,1);
        y2 = vertices(p,2);
        edges = [edges; x1 y1 x2 y2];
    end
end

% initialize tree
V = start;
parent = 0;
goal_found = false;
goal_node = [];

for k = 1:max_nodes
    if mod(k,500) == 0
    disp(['RRT iteration: ', num2str(k)])
    end
    % sample a random point in bounds
    x_rand = mapBoundary(1) + rand*(mapBoundary(3)-mapBoundary(1));
    y_rand = mapBoundary(2) + rand*(mapBoundary(4)-mapBoundary(2));
    q_rand = [x_rand y_rand];

    % find node closest to q rand
    min_dist = inf;
    min_node = 1;
    % loop through nodes
    for i = 1:size(V,1)
        % find distance between the random point and nodes
        d = norm(q_rand-V(i,:));
        % check if node is closest and store if true
        if d < min_dist
            min_dist = d;
            min_node = i;
        end
    end
    q_near = V(min_node,:);

    % create a new point toward the random point
    dist_new = q_rand-q_near;
    len = norm(dist_new);
    if len < tol
        continue;
    end
    dist_new = dist_new/len;
    q_new = q_near+step_size*dist_new;

    % check if new point and the nearby point intersect with an obstacle
    if q_new(1) < mapBoundary(1)+robot_radius|| q_new(1) > mapBoundary(3)... 
            -robot_radius|| q_new(2) < mapBoundary(2)+robot_radius...
            || q_new(2) > mapBoundary(4)-robot_radius
        continue;
    end

    collision = false;
    % check line against all obstacle edges
    num_check = 10;
    
    for t = linspace(0,1,num_check)
        p = q_near + t*(q_new-q_near);
    
        % check point stays inside reduced map bounds
        if p(1) < mapBoundary(1)+robot_radius || ...
           p(1) > mapBoundary(3)-robot_radius || ...
           p(2) < mapBoundary(2)+robot_radius || ...
           p(2) > mapBoundary(4)-robot_radius
            collision = true;
            break;
        end
    
        for u = 1:length(obstacles)
            vertices = obstacles{u};
            % check if robot center is inside obstacle
            [in, on] = inpolygon(p(1), p(2), vertices(:,1), vertices(:,2));
            if in || on
                collision = true;
                break;
            end
    
            % check distance from robot center to each obstacle edge
            for n = 1:size(vertices,1)
                p_next = mod(n,size(vertices,1)) + 1;
    
                x1 = vertices(n,1);
                y1 = vertices(n,2);
                x2 = vertices(p_next,1);
                y2 = vertices(p_next,2);
    
                edge = [x2-x1, y2-y1];
                pt = [p(1)-x1, p(2)-y1];
    
                edge_len_sq = edge(1)^2 + edge(2)^2;
                proj = (pt(1)*edge(1) + pt(2)*edge(2)) / edge_len_sq;
                proj = max(0,min(1,proj));
    
                closest_x = x1 + proj*edge(1);
                closest_y = y1 + proj*edge(2);
    
                dist_edge = norm([p(1)-closest_x, p(2)-closest_y]);
    
                if dist_edge < robot_radius
                    collision = true;
                    break;
                end
            end
    
            if collision==true
                break;
            end
        end
    
        if collision==true
            break;
        end
    end

    if collision==true
        continue;
    end

    % add new node to the tree
    V = [V; q_new];
    parent = [parent; min_node];
    new_node = size(V,1);

    % check if goal is visible from q_new
    goal_visible = true;
    num_check = 15;
    
    for t = linspace(0,1,num_check)
        p = q_new + t*(goal-q_new);
    
        % check point stays inside reduced map bounds
        if p(1) < mapBoundary(1)+robot_radius || ...
           p(1) > mapBoundary(3)-robot_radius || ...
           p(2) < mapBoundary(2)+robot_radius || ...
           p(2) > mapBoundary(4)-robot_radius
            goal_visible = false;
            break;
        end
    
        for u = 1:length(obstacles)
            vertices = obstacles{u};
    
            [in, on] = inpolygon(p(1), p(2), vertices(:,1), vertices(:,2));
            if in || on
                goal_visible = false;
                break;
            end
    
            for n = 1:size(vertices,1)
                p_next = mod(n,size(vertices,1)) + 1;
    
                x1 = vertices(n,1);
                y1 = vertices(n,2);
                x2 = vertices(p_next,1);
                y2 = vertices(p_next,2);
    
                edge = [x2-x1, y2-y1];
                pt = [p(1)-x1, p(2)-y1];
    
                edge_len_sq = edge(1)^2 + edge(2)^2;
                proj = (pt(1)*edge(1) + pt(2)*edge(2)) / edge_len_sq;
                proj = max(0,min(1,proj));
    
                closest_x = x1 + proj*edge(1);
                closest_y = y1 + proj*edge(2);
    
                dist_edge = norm([p(1)-closest_x, p(2)-closest_y]);
    
                if dist_edge < robot_radius
                    goal_visible = false;
                    break;
                end
            end
    
            if goal_visible==false
                break;
            end
        end
    
        if goal_visible==false
            break;
        end
    end
    
    if goal_visible==true
        V = [V; goal];
        parent = [parent; new_node];
        goal_node = size(V,1);
        goal_found = true;
        break;
    end

end

% check if any goal was found
if goal_found==false
    path = [];
    %disp('No path found');
    return;
end

path = [];
idx = goal_node;
while idx ~= 0
    path = [V(idx,:); path];
    idx = parent(idx);
end

end