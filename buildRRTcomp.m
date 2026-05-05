function [path, V, parent] = buildRRTcomp(compMapFile, mapBoundary, start, goal, robot_radius, wallState)
% BUILDRRT: Builds an RRT using the competition map format.
%
% Competition map format:
%   compMap.mat contains:
%       map       = known walls [x1 y1 x2 y2]
%       optWalls  = optional walls [x1 y1 x2 y2]
%
% INPUTS:
%   compMapFile    string, usually 'compMap.mat'
%   mapBoundary    [xmin ymin xmax ymax]
%   start          [x y]
%   goal           [x y]
%   robot_radius   robot radius in meters
%   wallState      optional wall state:
%                    1 = present
%                    0 = unknown
%                   -1 = absent
%
% OUTPUTS:
%   path           nx2 waypoint path
%   V              RRT nodes
%   parent         parent list

% ---------------- PARAMETERS ----------------
step_size = 0.2;
max_nodes = 10000;
goal_tol = 0.5;
tol = 1e-9;

% ---------------- LOAD COMPETITION MAP ----------------
S = load(compMapFile);

fixedWalls = S.map;        % known walls
optWalls = S.optWalls;     % optional walls

% If wallState is not given, assume all optional walls are unknown
if nargin < 6 || isempty(wallState)
    wallState = zeros(size(optWalls,1),1);
end

% For safe planning:
% include optional walls that are present or unknown
% exclude only walls that are known absent
activeOptWalls = optWalls(wallState >= 0, :);

% Combined wall list used for planning
walls = [fixedWalls; activeOptWalls];

% Physical wall thickness is about 0.1 m
wall_thickness = 0.10;
clearance = robot_radius + wall_thickness/2;

% ---------------- INITIALIZE TREE ----------------
V = start;
parent = 0;

goal_found = false;
goal_node = [];

% ---------------- RRT LOOP ----------------
for k = 1:max_nodes

    if mod(k,500) == 0
        disp(['RRT iteration: ', num2str(k)])
    end

    % Occasionally sample the goal directly
    if rand < 0.10
        q_rand = goal;
    else
        x_rand = mapBoundary(1) + rand*(mapBoundary(3)-mapBoundary(1));
        y_rand = mapBoundary(2) + rand*(mapBoundary(4)-mapBoundary(2));
        q_rand = [x_rand y_rand];
    end

    % Find nearest node
    min_dist = inf;
    min_node = 1;

    for i = 1:size(V,1)
        d = norm(q_rand - V(i,:));
        if d < min_dist
            min_dist = d;
            min_node = i;
        end
    end

    q_near = V(min_node,:);

    % Step from q_near toward q_rand
    direction = q_rand - q_near;
    len = norm(direction);

    if len < tol
        continue;
    end

    direction = direction / len;
    q_new = q_near + step_size * direction;

    % Check collision from q_near to q_new
    if edgeCollision(q_near, q_new, walls, mapBoundary, clearance)
        continue;
    end

    % Add new node
    V = [V; q_new];
    parent = [parent; min_node];
    new_node = size(V,1);

    % Check if goal can be connected directly
    if norm(q_new - goal) < goal_tol
        if ~edgeCollision(q_new, goal, walls, mapBoundary, clearance)
            V = [V; goal];
            parent = [parent; new_node];

            goal_node = size(V,1);
            goal_found = true;
            break;
        end
    end
end

% ---------------- BUILD PATH ----------------
if goal_found == false
    path = [];
    return;
end

path = [];
idx = goal_node;

while idx ~= 0
    path = [V(idx,:); path];
    idx = parent(idx);
end

end

% ============================================================
% Helper function: checks if a path segment collides with walls
% ============================================================
function collision = edgeCollision(q1, q2, walls, mapBoundary, clearance)

collision = false;
num_check = 20;

for t = linspace(0,1,num_check)

    p = q1 + t*(q2 - q1);

    % Check map bounds
    if p(1) < mapBoundary(1) + clearance || ...
       p(1) > mapBoundary(3) - clearance || ...
       p(2) < mapBoundary(2) + clearance || ...
       p(2) > mapBoundary(4) - clearance
        collision = true;
        return;
    end

    % Check distance to every wall segment
    for i = 1:size(walls,1)

        a = walls(i,1:2);
        b = walls(i,3:4);

        d = pointToSegmentDistance(p, a, b);

        if d < clearance
            collision = true;
            return;
        end
    end
end

end

% ============================================================
% Helper function: distance from point to line segment
% ============================================================
function d = pointToSegmentDistance(p, a, b)

ab = b - a;
ap = p - a;

if norm(ab) < 1e-9
    d = norm(p - a);
    return;
end

s = dot(ap, ab) / dot(ab, ab);
s = max(0, min(1, s));

closest = a + s*ab;
d = norm(p - closest);

end