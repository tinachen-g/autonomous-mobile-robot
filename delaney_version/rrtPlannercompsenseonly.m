function [dataStore] = rrtPlannercompsenseonly(Robot, maxTime)
% RRTPLANNERCOMPSENSEONLY
% Visits waypoints using RRT while assuming all optional walls exist.
% Still senses optional walls and stores wallBelief, but does NOT update map
% or replan because of wall detections.

if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

global dataStore;

dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'rrtPath', [], ...
                   'rrtNodes', [], ...
                   'rrtParent', [], ...
                   'wallBelief', [], ...
                   'visitedWaypoints', zeros(0,2));

noRobotCount = 0;
SetFwdVelAngVelCreate(Robot, 0, 0);

% ---------------- SETTINGS ----------------
mapfile = 'PracticeMap2026.mat';
S = load(mapfile);

robot_radius = 0.2;

outer = S.map(1:4,:);
pts = [outer(:,1:2); outer(:,3:4)];

xmin = min(pts(:,1));
xmax = max(pts(:,1));
ymin = min(pts(:,2));
ymax = max(pts(:,2));

mapBoundary = [xmin ymin xmax ymax];

if isfield(S,'ECwaypoints')
    allWaypoints = [S.waypoints; S.ECwaypoints];
else
    allWaypoints = S.waypoints;
end

visited = false(size(allWaypoints,1),1);

closeEnough = 0.15;
epsilon = 0.1;

path = [];
gotopt = 1;
pathBuilt = false;

currentGoalIdx = [];
currentGoal = [];
needPlan = true;

% Assume all optional walls exist for planning
numOptWalls = size(S.optWalls,1);
wallStateForPlanning = ones(numOptWalls,1);

% Wall sensing still gets stored, but does not affect planning
wallBelief.presentScore = zeros(numOptWalls,1);
wallBelief.absentScore = zeros(numOptWalls,1);
wallBelief.state = zeros(numOptWalls,1);

sensorOrigin = [0 0.08];
numDepthRays = 9;
angles = linspace(deg2rad(27), -deg2rad(27), numDepthRays);

tic
while toc < maxTime

    % Read sensors
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

    if isempty(dataStore.truthPose)
        SetFwdVelAngVelCreate(Robot, 0, 0);
        continue;
    end

    robotXY = dataStore.truthPose(end,2:3);

    % Check if current waypoint reached
    if ~isempty(currentGoalIdx)
        distToGoal = norm(robotXY - currentGoal);

        if distToGoal <= 0.2
            disp(['Visited waypoint ', num2str(currentGoalIdx)]);

            visited(currentGoalIdx) = true;
            dataStore.visitedWaypoints = allWaypoints(visited,:);

            try
                BeepCreate(Robot);
            catch
                disp('Beep skipped');
            end

            currentGoalIdx = [];
            currentGoal = [];
            path = [];
            gotopt = 1;
            pathBuilt = false;
            needPlan = true;
        end
    end

    % Choose nearest unvisited waypoint
    if isempty(currentGoalIdx)

        unvisitedIdx = find(~visited);

        if isempty(unvisitedIdx)
            disp('All waypoints visited');
            cmdV = 0;
            cmdW = 0;
        else
            dists = vecnorm(allWaypoints(unvisitedIdx,:) - robotXY, 2, 2);
            [~, bestLocalIdx] = min(dists);

            currentGoalIdx = unvisitedIdx(bestLocalIdx);
            currentGoal = allWaypoints(currentGoalIdx,:);

            disp(['New goal is waypoint ', num2str(currentGoalIdx)]);

            needPlan = true;
        end
    end

    % Build RRT only when choosing a new waypoint
    if needPlan && ~isempty(currentGoalIdx)

        start = robotXY;

        [path, V, parent] = buildRRTcomp(mapfile, ...
            mapBoundary, start, currentGoal, robot_radius, wallStateForPlanning);

        needPlan = false;

        if isempty(path)
            disp(['Planner failed for waypoint ', num2str(currentGoalIdx)]);

            cmdV = 0;
            cmdW = 0;
            pathBuilt = false;

            % Skip unreachable waypoint for now
            visited(currentGoalIdx) = true;
            dataStore.visitedWaypoints = allWaypoints(visited,:);

            currentGoalIdx = [];
            currentGoal = [];
            needPlan = true;

        else
            disp(['Planner found path to waypoint ', num2str(currentGoalIdx)]);

            pathBuilt = true;
            gotopt = 2;

            dataStore.rrtPath = path;
            dataStore.rrtNodes = V;
            dataStore.rrtParent = parent;
        end
    end

    % Follow path
    if pathBuilt
        [cmdV, cmdW, gotopt] = visitWaypoints(path, gotopt, closeEnough, epsilon);

        if gotopt > size(path,1)
            cmdV = 0;
            cmdW = 0;

            pathBuilt = false;
            needPlan = true;
        end
    else
        cmdV = 0;
        cmdW = 0;
    end

    % Sense optional walls and store data, but DO NOT replan from it
    if ~isempty(dataStore.rsdepth)
        pose_est = dataStore.truthPose(end,2:4);
        depth_meas = dataStore.rsdepth(end,3:end);

        wallBelief = detectOptionalWalls(wallBelief, pose_est, depth_meas, ...
            mapfile, sensorOrigin, angles);

        dataStore.wallBelief = wallBelief;
    end

    % Limit commands
    wheel2center = 0.13;
    maxV = 0.2;
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);

    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end

SetFwdVelAngVelCreate(Robot, 0, 0);

end