function[dataStore] = rrtPlannercompwithmapupdatenew(Robot, maxTime)
% RRTPLANNER: builds an RRT path once, then follows it with visitWaypoints
%
% INPUTS:
%   Robot       robot object / simulator object
%   maxTime     max runtime in seconds
%
% OUTPUT:
%   dataStore   logged sensor data

% Set unspecified inputs
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

% initialize datalog struct
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [],...
                   'rrtPath', [], ...
                   'rrtNodes', {}, ...
                   'rrtParent', {}, ...
                   'wallBelief', [], ...
                   'visitedWaypoints', []);

noRobotCount = 0;

% stop robot initially
SetFwdVelAngVelCreate(Robot, 0, 0);

% variables
mapfile = 'PracticeMap2026.mat';
S = load(mapfile);
robot_radius = 0.2;
% map boundary
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
dataStore.visitedWaypoints = [];

closeEnough = 0.1;
epsilon = 0.1;

path = [];
gotopt = 1;
pathBuilt = false;

currentGoalIdx = [];
currentGoal = [];
needReplan = true;

numOptWalls = size(S.optWalls,1);

wallBelief.presentScore = zeros(numOptWalls,1);
wallBelief.absentScore = zeros(numOptWalls,1);
wallBelief.state = zeros(numOptWalls,1);
lastWallState = wallBelief.state;
mapChangedSinceLastPlan = false;

sensorOrigin = [0 0.08];
numDepthRays = 9;
angles = linspace(deg2rad(27), -deg2rad(27), numDepthRays);

tic
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);
    
    % wait until truthPose exists
    if isempty(dataStore.truthPose)
        SetFwdVelAngVelCreate(Robot, 0, 0);
        continue;
    end
    
    % RRT planning function calling

    robotXY = dataStore.truthPose(end,2:3);

    % check if current waypoint was reached
    if ~isempty(currentGoalIdx)
        distToGoal = norm(robotXY - currentGoal);
        if distToGoal <= 0.2
            disp(['Visited waypoint ', num2str(currentGoalIdx)]);
            visited(currentGoalIdx) = true;
            dataStore.visitedWaypoints = allWaypoints(visited,:);
    
            % beep when waypoint is reached
            try
                BeepCreate(Robot);
            catch
                disp('Beep skipped');
            end
    
            % force a new goal and new plan
            currentGoalIdx = [];
            currentGoal = [];
            path = [];
            pathBuilt = false;
            gotopt = 1;
            needReplan = true;
        end
    end
    
    % Choose a new waypoint if there is no active goal
    if isempty(currentGoalIdx)
    
        unvisitedIdx = find(~visited);
    
        if isempty(unvisitedIdx)
            disp('All waypoints visited');
            cmdV = 0;
            cmdW = 0;
        else
            % choose nearest unvisited waypoint
            dists = vecnorm(allWaypoints(unvisitedIdx,:) - robotXY, 2, 2);
            [~, bestLocalIdx] = min(dists);
    
            currentGoalIdx = unvisitedIdx(bestLocalIdx);
            currentGoal = allWaypoints(currentGoalIdx,:);
    
            disp(['New goal is waypoint ', num2str(currentGoalIdx)]);
    
            needReplan = true;
        end
    end

    % build/rebuild RRT if needed
    if needReplan && ~isempty(currentGoalIdx)
    
        start = robotXY;
    
        % updated planning map
        planningMap = makePlanningMap(S, wallBelief);
    
        tempMap.map = planningMap;
        tempMap.optWalls = zeros(0,4);
        save('currentPlanningMap.mat','-struct','tempMap');
    
        [path, V, parent] = buildRRTcomp('currentPlanningMap.mat', ...
            mapBoundary, start, currentGoal, robot_radius);
    
        needReplan = false;
    
        if isempty(path)
            disp(['Planner failed for waypoint ', num2str(currentGoalIdx)]);
    
            cmdV = 0;
            cmdW = 0;
            pathBuilt = false;
    
            % skip this waypoint for now
            visited(currentGoalIdx) = true;
            currentGoalIdx = [];
            currentGoal = [];
            needReplan = true;
    
        else
            disp(['Planner found path to waypoint ', num2str(currentGoalIdx)]);
    
            pathBuilt = true;
            gotopt = 2;
    
            dataStore.rrtPath = path;
            dataStore.rrtNodes{end+1} = V;
            dataStore.rrtParent{end+1} = parent;
        end
    end
    
    % Follow current path
    if pathBuilt
        [cmdV, cmdW, gotopt] = visitWaypoints(path, gotopt, closeEnough, epsilon);
    
        if gotopt > size(path,1)
            cmdV = 0;
            cmdW = 0;
    
            pathBuilt = false;
    
            % Only replan here, after finishing the current path/waypoint segment
            if mapChangedSinceLastPlan
                disp('Replanning now because map changed during previous path');
                needReplan = true;
                mapChangedSinceLastPlan = false;
            else
                needReplan = true;
            end
        end
    else
        cmdV = 0;
        cmdW = 0;
    end

    % sense walls while moving
    pose_est = dataStore.truthPose(end,2:4);
    depth_meas = dataStore.rsdepth(end,3:end);

    wallBelief = detectOptionalWalls(wallBelief, pose_est, depth_meas, ...
    mapfile, sensorOrigin, angles);

    dataStore.wallBelief = wallBelief;

    if any(wallBelief.state ~= lastWallState)
        disp('Wall belief changed, will replan after current waypoint');
        mapChangedSinceLastPlan = true;
        lastWallState = wallBelief.state;
    end

    % limit commands
    wheel2center = 0.13;
    maxV = 0.2;
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2center);
    
    % if localization is lost, stop robot
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end
end

% stop before exiting
SetFwdVelAngVelCreate(Robot, 0, 0);

end

function planningMap = makePlanningMap(S, wallBelief)
% MAKEPLANNINGMAP
% Fixed walls are always included.
% Optional walls are included only if detected present.

fixedWalls = S.map;

if isempty(S.optWalls)
    presentOptWalls = zeros(0,4);
else
    % Include optional walls that are present or still unknown.
    % Exclude only walls that are confidently absent.
    presentOptWalls = S.optWalls(wallBelief.state >= 0, :);
end

planningMap = [fixedWalls; presentOptWalls];

end