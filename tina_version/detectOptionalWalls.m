function [wallBelief] = detectOptionalWalls(wallBelief, pose_est, depth_meas, ...
    mapfile, sensorOrigin, angles)
% DETECT OPTIONAL WALLS: updates belief for each optional wall using measured depth data
% 
% INPUTS:
%   wallBelief                    previous wall state
%   mapfile                      file for map
%   pose_est                      pose from localization estimate
%   sensorOrigin                  sensor offset [x y]
%   angles                        angular fov of depth sensor
%   depth_meas                    measured depth scan
% 
% OUTPUTS:
%   wallBelief.presentScore       updates when optional wall is sensed
%   wallBelief.absentScore        updates when optional wall is not sensed
%   wallBelief.state              1 = wall present, 0 = unknown, -1 = wall absent

% load map walls from file
compmap = load(mapfile);
fixedwalls = compmap.map; % [n x 4]
optwalls = compmap.optWalls; % [n x 4]
numoptwalls = size(optwalls,1);

% initialize beliefs for optional wall
if isempty(wallBelief)
    wallBelief.presentScore = zeros(numoptwalls,1);
    wallBelief.absentScore = zeros(numoptwalls,1);
    wallBelief.state = zeros(numoptwalls,1);
end

% loop through optional walls and compare present and absent to sensor measurement
for i = 1:numoptwalls
    % build maps with and without optional walls for comparison
    fixedmap = fixedwalls;
    optionalmap = [fixedwalls; optwalls(i,:)];

    % predicted depth measurements
    depthfixed = depthPredict(pose_est, fixedmap, sensorOrigin, angles);
    depthoptional = depthPredict(pose_est, optionalmap, sensorOrigin, angles);
    depth_meas = depth_meas(:);
    depthfixed = depthfixed(:);
    depthoptional = depthoptional(:);

    N = min(length(depth_meas), length(depthfixed));
    depth_meas = depth_meas(1:N);
    depthfixed = depthfixed(1:N);
    depthoptional = depthoptional(1:N);

    % filter depth measurements
    % can tune depth measurement range for accuracy with testing
    validmeas = ~isnan(depth_meas) & depth_meas > 0.1 & depth_meas < 2.4;
    % can tune error threshold between predicted depths
    predictmeas = validmeas & abs(depthoptional - depthfixed) > 0.1;
    if nnz(predictmeas) < 2
        continue
    end

    predictmeas = predictmeas(:);

    N = min(length(depth_meas), length(depthfixed));
    depth_meas = depth_meas(1:N);
    depthfixed = depthfixed(1:N);
    predictmeas = predictmeas(1:N);

    % finds error between measurement depths and fixed/optional wall depths
    fixederror = mean(abs(depth_meas(predictmeas)-depthfixed(predictmeas)));
    optionalerror = mean(abs(depth_meas(predictmeas)-depthoptional(predictmeas)));

    % check if error is within threshold and update wall belief
    threshold = 0.1;
    if optionalerror + threshold < fixederror
        wallBelief.presentScore(i) = wallBelief.presentScore(i) + 1;
    elseif fixederror + threshold < optionalerror
        wallBelief.absentScore(i) = wallBelief.absentScore(i) + 1;
    end

    % once a confident score is reached, update wall state
    if wallBelief.presentScore(i) >= 3
        wallBelief.state(i) = 1;
    elseif wallBelief.absentScore(i) >= 3
        wallBelief.state(i) = -1;
    else
        wallBelief.state(i) = 0;
    end
end