
function [cmdV, cmdW, gotopt] = visitWaypoints(waypoints, gotopt, closeEnough, epsilon)

global dataStore

% gets x and y coordinates of waypoints and robot
next_waypoint = waypoints(gotopt,:);
x_current_waypoint = next_waypoint(1);
y_current_waypoint = next_waypoint(2);

x_current_robot = dataStore.truthPose(end, 2);
y_current_robot = dataStore.truthPose(end, 3);

% calculates distance from robot to waypoint
x_dist = x_current_waypoint-x_current_robot;
y_dist = y_current_waypoint-y_current_robot;

distance = sqrt((x_dist)^2+(y_dist)^2);

% determines if waypoint is close and updates current waypoint and distance
if distance < closeEnough
    gotopt = gotopt + 1;
    if gotopt > size(waypoints,1)
        cmdV = 0;
        cmdW = 0;
        return;
    end

    next_waypoint = waypoints(gotopt,:);
    x_current_waypoint = next_waypoint(1);
    y_current_waypoint = next_waypoint(2);
    x_dist = x_current_waypoint-x_current_robot;
    y_dist = y_current_waypoint-y_current_robot;
    x_current_robot = dataStore.truthPose(end, 2);
    y_current_robot = dataStore.truthPose(end, 3);
end

% provide inputs to feedbackLin function
speed = 1;
cmdVx = speed*x_dist;
cmdVy = speed*y_dist;
theta = dataStore.truthPose(end, 4);

[cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, theta, epsilon);