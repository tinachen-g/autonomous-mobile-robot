function[dataStore] = finalCompetition(Robot,maxTime, offset_x, offset_y)
%
%   Cornell University
%   MAE 5180/ECE 5772: Autonomous Mobile Robots
% -------------------------------------------------------------------------

if nargin < 1
    disp('error: tcp/ip port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end
if nargin < 3
    offset_x = 0;
    offset_y = 0;
end

try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

% -------------------------------------------------------------------------
% map / waypoint load
% -------------------------------------------------------------------------
MAPFILE = 'PracticeMap2026.mat';
data = load(MAPFILE);
map = data.map;
optWalls = data.optWalls;
waypoints = data.waypoints;
ECwaypoints = data.ECwaypoints;
beaconLoc = data.beaconLoc;

nMandatory = size(waypoints,1);
allWaypoints = [waypoints; ECwaypoints];

global dataStore;
dataStore = struct('truthPose', [], ...
                   'odometry',  [], ...
                   'rsdepth',   [], ...
                   'bump',      [], ...
                   'beacon',    [], ...
                   'visitedWaypoints', [], ...
                   'rrtPath', [], ...
                   'rrtNodes', [], ...
                   'rrtParent', [], ...
                   'wallBelief', []);

% -------------------------------------------------------------------------
% robot geometry
% -------------------------------------------------------------------------
noRobotCount = 0;
wheel2Center = 0.13;
maxV = 0.13;
% apply camera offsets to sensor origin
sensorOrigin = [0.13 + offset_x, offset_y];

% -------------------------------------------------------------------------
% spin parameters
% -------------------------------------------------------------------------
SPIN_RATE = 0.4;
SPIN_DURATION = (2*pi / SPIN_RATE) * 1.5;

% -------------------------------------------------------------------------
% realsense parameters
% -------------------------------------------------------------------------
RS_FOV = 58 * pi/180;
NUM_RAYS = 9;
rsAngles = linspace(-RS_FOV/2, RS_FOV/2, NUM_RAYS)';

% -------------------------------------------------------------------------
% particle filter parameters
% -------------------------------------------------------------------------
N_PARTICLES = 600;
SIGMA_D = 0.1;
SIGMA_A = 0.2;
SIGMA_DEPTH = 0.12; % depth likelihood std dev (m)
SIGMA_BEACON = 0.06; % bearing likelihood std dev (rad)
SIGMA_RANGE = 0.15; % range-only beacon std dev (m)

% -------------------------------------------------------------------------
% convergence parameters
% -------------------------------------------------------------------------
CONV_MASS_THR = 0.60;
CONV_WP_RADIUS = 0.30;

% -------------------------------------------------------------------------
% beacon-guided seeding parameters
% -------------------------------------------------------------------------
SEED_FEASIBLE_FRAC = 0.85;
BEACON_SEED_RANGE = 2.0;

% -------------------------------------------------------------------------
% depth-signature parameters
% -------------------------------------------------------------------------
N_SIG_ORI = 72;
SIG_WEIGHT = 0.6;

ROBOT_RADIUS = 0.30; % inflation radius for obstacles


% -------------------------------------------------------------------------
% state machine
% -------------------------------------------------------------------------
phase = 'SPIN';
phaseStart = 0;

spinBeacons = [];
bestDepthScan = [];

particles = [];
dr_pose = [0, 0, 0];
pose_est = [];

wpVisited = false(size(allWaypoints,1), 1);
navPath = [];
navPathIdx = 1;
currentGoalIdx = [];
gotopt = 1;
closeEnough = 0.18;
epsilon = 0.6;

wallBelief = struct();
wallBelief.score = zeros(size(optWalls,1), 1);
wallBelief.state = zeros(size(optWalls,1), 1);
postConvergence = false;

predict_func = @(poses,u) motionModel(poses, u, SIGMA_D, SIGMA_A);

accPointCloud = zeros(0,2);
scanHistory = struct('time',{},'pose',{},'scan',{});
poseGraph = zeros(0,3);

% =========================================================================
% precompute depth signatures for each mandatory waypoint
% =========================================================================
fprintf('\n=== precomputing depth signatures (%d wps, %d orientations) ===\n', ...
    nMandatory, N_SIG_ORI);
wpDepthSigs   = cell(nMandatory, 1);
wpSigAngles   = linspace(0, 2*pi*(1-1/N_SIG_ORI), N_SIG_ORI)';
for wi = 1:nMandatory
    sigs = zeros(N_SIG_ORI, NUM_RAYS);
    for oi = 1:N_SIG_ORI
        probe = [waypoints(wi,1), waypoints(wi,2), wpSigAngles(oi)];
        d = depthPredict(probe, map, sensorOrigin, rsAngles);
        sigs(oi,:) = d(:)';
    end
    wpDepthSigs{wi} = sigs;
end
fprintf('=== signatures done ===\n\n');

% =========================================================================
% beacon-waypoint feasibility
% =========================================================================
BEACON_VIS_RANGE = BEACON_SEED_RANGE;
fprintf('=== beacon-waypoint visibility (range=%.1fm) ===\n', BEACON_VIS_RANGE);
nBeacons = size(beaconLoc,1);
wpBeaconVis = false(nMandatory, nBeacons);

for wi = 1:nMandatory
    wx = waypoints(wi,1); wy = waypoints(wi,2);
    visible = [];
    for bi = 1:nBeacons
        bx = beaconLoc(bi,2); by = beaconLoc(bi,3);
        if sqrt((bx-wx)^2+(by-wy)^2) > BEACON_VIS_RANGE; continue; end
        blocked = false;
        for mi = 1:size(map,1)
            [isect,~,~] = intersectPoint(wx,wy,bx,by, ...
                map(mi,1),map(mi,2),map(mi,3),map(mi,4));
            if isect; blocked = true; break; end
        end
        if ~blocked
            wpBeaconVis(wi,bi) = true;
            visible(end+1) = beaconLoc(bi,1); %#ok<AGROW>
        end
    end
    if isempty(visible)
        fprintf('  wp%d (%.2f,%.2f): no beacons visible\n', wi, wx, wy);
    else
        fprintf('  wp%d (%.2f,%.2f): beacons %s\n', wi, wx, wy, mat2str(visible));
    end
end
fprintf('===================================================\n\n');


% =========================================================================
% main loop
% =========================================================================
SetFwdVelAngVelCreate(Robot, 0, 0);
tic
while toc < maxTime

    % ---- sensor read ----
    [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);

    % ---- odometry ----
    deltaD = 0; deltaA = 0;
    try
        deltaD = DistanceSensorRoomba(CreatePort);
        deltaA = AngleSensorRoomba(CreatePort);
        dataStore.odometry = [dataStore.odometry; toc deltaD deltaA];
    catch
        disp('[odom] error retrieving odometry.');
    end

    dr_pose(1) = dr_pose(1) + deltaD*cos(dr_pose(3));
    dr_pose(2) = dr_pose(2) + deltaD*sin(dr_pose(3));
    dr_pose(3) = mod(dr_pose(3)+deltaA+pi, 2*pi)-pi;
    u = [deltaD; deltaA];

    % ---- beacon observations ----
    currBeaconObs = [];
    try
        tags = RealSenseTag(Robot);
        if ~isempty(tags)
            for ti = 1:size(tags,1)
                tagID = tags(ti,2);
                tagZ = tags(ti,3);
                tagX = tags(ti,4);
                dist_m = sqrt(tagZ^2 + tagX^2);
                localBearing = atan2(tagX, tagZ);
                currBeaconObs(end+1,:) = [tagID, localBearing, dist_m]; %#ok<AGROW>
            end
        end
    catch
        % no beacon available this tick
    end
    if ~isempty(currBeaconObs)
        spinBeacons = [spinBeacons; currBeaconObs]; %#ok<AGROW>
    end

    % ---- depth / bump ----
    latestDepth = [];
    if ~isempty(dataStore.rsdepth)
        row = dataStore.rsdepth(end,:);
        if numel(row) >= 3
            latestDepth = row(3:end)';
        end
    end

    if postConvergence && ~isempty(pose_est) && ~isempty(latestDepth)
        oldWallState = wallBelief.state;
    
        wallBelief = updateWallBelief(wallBelief, optWalls, pose_est, ...
            latestDepth, rsAngles, sensorOrigin, map);
    
        dataStore.wallBelief = wallBelief;
    
        if any(wallBelief.state ~= oldWallState)
            fprintf('[wall] belief changed: %s\n', mat2str(wallBelief.state'));
        end
    end

    bumpTriggered = ~isempty(dataStore.bump) && any(dataStore.bump(end,2:end));

    if ~isempty(latestDepth)
        vc = sum(isfinite(latestDepth) & latestDepth>0.05 & latestDepth<2.8);
        bc = 0;
        if ~isempty(bestDepthScan)
            bc = sum(isfinite(bestDepthScan) & bestDepthScan>0.05 & bestDepthScan<2.8);
        end
        if vc > bc; bestDepthScan = latestDepth; end
    end

    % =========================================================================
    % particle filter update
    % =========================================================================
    if strcmp(phase,'LOCALIZE_POS') && ~isempty(particles)
        z.depth       = latestDepth;
        z.beacons     = currBeaconObs;
        z.spinBeacons = spinBeacons;
        z.wpDepthSigs = wpDepthSigs;
        z.waypoints   = waypoints;
        lh = @(p,zz) measurementLikelihood5(p, zz, map, sensorOrigin, rsAngles, ...
            beaconLoc, SIGMA_DEPTH, SIGMA_BEACON, SIGMA_RANGE, SIG_WEIGHT);
        [particles, best_particle, ~, ~] = PF2(particles, u, z, predict_func, lh);
        pose_est = best_particle;
    end


    % ---- accumulate point cloud (only after convergence) ----
    if postConvergence && ~isempty(pose_est) && ~isempty(latestDepth)
        pts = depthToWorldPoints(pose_est, latestDepth, rsAngles, sensorOrigin);
        accPointCloud = [accPointCloud; pts]; %#ok<AGROW>
        if size(accPointCloud,1) > 8000
            accPointCloud = accPointCloud(end-7999:end,:);
        end
    end

    % =========================================================================
    % state machine
    % =========================================================================
    switch phase

        case 'SPIN'
            elapsed = toc - phaseStart;
            if elapsed < SPIN_DURATION
                [cmdV, cmdW] = limitCmds(0, SPIN_RATE, maxV, wheel2Center);
                SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
            else
                SetFwdVelAngVelCreate(Robot, 0, 0);

                if isempty(spinBeacons)
                    fprintf('[spin] done. no beacons seen — uniform wp seed.\n');
                    particles = initParticlesAtWaypoints(waypoints, N_PARTICLES);
                else
                    seenTagIDs = unique(spinBeacons(:,1));
                    fprintf('[spin] done. beacons seen: %s\n', mat2str(seenTagIDs'));
                    particles = initParticlesBeaconGuided( ...
                        waypoints, beaconLoc, wpBeaconVis, ...
                        seenTagIDs, N_PARTICLES, SEED_FEASIBLE_FRAC);
                end

                fprintf('[spin->loc_pos] %d particles (full 3-dof)\n', N_PARTICLES);
                phase = 'LOCALIZE_POS';
                phaseStart = toc;
            end

        case 'LOCALIZE_POS'
            elapsed = toc - phaseStart;
            if ~isempty(particles)
                [convReached, bestWPidx, bestMass] = checkConvergence( ...
                    particles, waypoints, CONV_WP_RADIUS, CONV_MASS_THR);

                posStd = std(particles(:,1:2), 0, 1);
                oriStd = std(particles(:,3));

                for wpi = 1:nMandatory
                    inR = sqrt((particles(:,1)-waypoints(wpi,1)).^2 + ...
                               (particles(:,2)-waypoints(wpi,2)).^2) < CONV_WP_RADIUS;
                    fprintf('  wp%d mass=%.3f\n', wpi, sum(particles(inR,4)));
                end

                if convReached || elapsed > 25.0
                    if elapsed > 25.0 && ~convReached
                        fprintf('[loc_pos] timeout — using highest-mass wp.\n');
                    else
                        fprintf('[loc_pos] converged (mass=%.2f at wp%d).\n', ...
                            bestMass, bestWPidx);
                    end
                    
                    % snap xy to nearest mandatory wp; keep estimated heading
                    dists = sqrt((waypoints(:,1)-pose_est(1)).^2 + ...
                                 (waypoints(:,2)-pose_est(2)).^2);
                    [snapD, startWaypoint] = min(dists);
                    pose_est(1) = waypoints(startWaypoint,1);
                    pose_est(2) = waypoints(startWaypoint,2);

                    % refine heading via best-ncc signature match
                    if ~isempty(latestDepth) && ~isempty(wpDepthSigs{startWaypoint})
                        sigs = wpDepthSigs{startWaypoint};
                        nccVals = zeros(N_SIG_ORI, 1);
                        for oi = 1:N_SIG_ORI
                            nccVals(oi) = scanNCC(latestDepth(:), sigs(oi,:)');
                        end
                        [~, bestOriIdx] = max(nccVals);

                        % CHANGED TO ADD 45 DEG
                        pose_est(3) = wpSigAngles(bestOriIdx) + pi/4;
                        fprintf('[loc_pos] heading snapped via ncc to %.3f rad (%.1f deg)\n', ...
                            pose_est(3), pose_est(3)*180/pi);
                    end
                    fprintf('[loc_pos] snap to wp%d (%.2f,%.2f) d=%.2fm  theta=%.3frad\n', ...
                        startWaypoint, pose_est(1), pose_est(2), snapD, pose_est(3));

                    particles = initParticlesAroundPose(pose_est, N_PARTICLES, 0.04, 0.06);
                    dr_pose = pose_est;
                    postConvergence = true;

                    % log first scan for loop closure baseline
                    if ~isempty(latestDepth)
                        entry.time = toc; entry.pose = pose_est; entry.scan = latestDepth;
                        scanHistory(end+1) = entry; %#ok<AGROW>
                        poseGraph(end+1,:) = pose_est; %#ok<AGROW>
                    end

                    wpVisited(startWaypoint) = true;
                    
                    % log visited waypoint to datastore
                    dataStore.visitedWaypoints = [dataStore.visitedWaypoints; ...
                        toc, startWaypoint, pose_est];
                    
                    BeepCreate(Robot);
                    phase = 'PLANNING';
                    phaseStart = toc;
                end
            end

        case 'NAVIGATE'
            % check for bump and back up if detected
            if bumpTriggered
                fprintf('[nav] bump detected! backing up...\n');
                SetFwdVelAngVelCreate(Robot, -0.1, 0);
                pause(0.5);
                SetFwdVelAngVelCreate(Robot, 0, 0);
                continue;
            end
            
            % moving the robot
            if isempty(navPath) || isempty(dr_pose)
                SetFwdVelAngVelCreate(Robot, 0, 0);
                phase = 'PLANNING';
                break;
            end
        
            [cmdV, cmdW, gotopt] = followPathWithPose(navPath, gotopt, dr_pose, closeEnough, epsilon);
        
            if gotopt > size(navPath,1)
                cmdV = 0;
                cmdW = 0;
        
                if ~isempty(currentGoalIdx)
                    wpVisited(currentGoalIdx) = true;
                    % log using dr_pose for consistency
                    dataStore.visitedWaypoints = [dataStore.visitedWaypoints; ...
                        toc, currentGoalIdx, dr_pose];
        
                    BeepCreate(Robot);
                    fprintf('[nav] visited waypoint %d\n', currentGoalIdx);
                end
        
                navPath = [];
                navPathIdx = 1;
                gotopt = 1;
                currentGoalIdx = [];
        
                phase = 'PLANNING';
                phaseStart = toc;
            end
        
            [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);


        case 'PLANNING'
            if ~isempty(dr_pose)
                unvisitedMand = find(~wpVisited(1:nMandatory));
                unvisitedEC = find(~wpVisited(nMandatory+1:end)) + nMandatory;
        
                % mandatory first, EC second
                if ~isempty(unvisitedMand)
                    candidates = unvisitedMand;
                elseif ~isempty(unvisitedEC)
                    candidates = unvisitedEC;
                else
                    phase = 'DONE';
                    break;
                end
        
                robotXY = dr_pose(1:2);
        
                % choose nearest unvisited waypoint
                dists = vecnorm(allWaypoints(candidates,:) - robotXY, 2, 2);
                [~, bestLocalIdx] = min(dists);
        
                currentGoalIdx = candidates(bestLocalIdx);
                currentGoal = allWaypoints(currentGoalIdx,:);
        
                fprintf('[rrt plan] new goal wp%d at (%.2f, %.2f)\n', ...
                    currentGoalIdx, currentGoal(1), currentGoal(2));
        
                % map boundary from known map walls
                outer = map(1:4,:);
                pts = [outer(:,1:2); outer(:,3:4)];
                mapBoundary = [min(pts(:,1)) min(pts(:,2)) max(pts(:,1)) max(pts(:,2))];
        
                % assume optional walls exist unless known absent
                wallState = zeros(size(optWalls,1),1);
        
                [navPath, V, parent] = buildRRTcomp(MAPFILE, mapBoundary, ...
                    robotXY, currentGoal, ROBOT_RADIUS, wallState);
        
                if isempty(navPath)
                    fprintf('[rrt plan] failed for wp%d, skipping\n', currentGoalIdx);
        
                    wpVisited(currentGoalIdx) = true;
                    currentGoalIdx = [];
                    navPath = [];
                    gotopt = 1;
        
                    phase = 'PLANNING';
                else
                    fprintf('[rrt plan] path to wp%d has %d points\n', ...
                        currentGoalIdx, size(navPath,1));
        
                    dataStore.rrtPath = navPath;
                    dataStore.rrtNodes = V;
                    dataStore.rrtParent = parent;
        
                    gotopt = min(2, size(navPath,1));
                    navPathIdx = gotopt;
        
                    phase = 'NAVIGATE';
                    phaseStart = toc;
                end
            end

        case 'DONE'
            SetFwdVelAngVelCreate(Robot, 0, 0);
            plotFinalMap(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
                allWaypoints, wpVisited, wallBelief, pose_est, accPointCloud);
            break;

    end

    % ---- live plots ----
    displayPose = pose_est;
    if strcmp(phase, 'NAVIGATE')
        displayPose = dr_pose;
    end
    updateLivePlot(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
        allWaypoints, wpVisited, particles, displayPose, navPath, navPathIdx, phase, toc, postConvergence);

    % updateDepthViz(latestDepth, rsAngles, pose_est, accPointCloud, map, phase, toc);

    if noRobotCount >= 5
        SetFwdVelAngVelCreate(Robot, 0, 0);
        break;
    end

end

SetFwdVelAngVelCreate(Robot, 0, 0);
fprintf('[end] t=%.1fs\n', toc);
fprintf('[end] visited waypoints:\n');
for i = 1:size(dataStore.visitedWaypoints,1)
    fprintf('  t=%.1fs  wp%d  pose=(%.3f, %.3f, %.3f)\n', ...
        dataStore.visitedWaypoints(i,1), dataStore.visitedWaypoints(i,2), ...
        dataStore.visitedWaypoints(i,3:5));
end
end


% =========================================================================
% measurement likelihood
% =========================================================================
function w = measurementLikelihood5(pose, z, map, sensorOrigin, rsAngles, ...
    beaconLoc, sigma_depth, sigma_beacon, sigma_range, sigWeight)

w = 1.0;

% ---- live depth scan likelihood ----
if ~isempty(z.depth)
    meas = z.depth(:);
    pred = depthPredict(pose, map, sensorOrigin, rsAngles); pred = pred(:);
    K = min(numel(meas), numel(pred));
    meas = meas(1:K); pred = pred(1:K);
    valid = isfinite(meas) & meas>0.05 & meas<2.8 & isfinite(pred) & pred>0;
    if nnz(valid) >= 3
        w = w * exp(sum(-0.5*((meas(valid)-pred(valid))/sigma_depth).^2));
    end
end

% ---- depth-signature matching against top-2 nearest waypoints ----
if ~isempty(z.depth) && isfield(z,'wpDepthSigs') && ~isempty(z.wpDepthSigs)
    meas = z.depth(:);
    validMeas = isfinite(meas) & meas>0.05 & meas<2.8;
    if nnz(validMeas) >= 3
        wpts = z.waypoints;
        dists2 = (wpts(:,1)-pose(1)).^2 + (wpts(:,2)-pose(2)).^2;
        [~, sortIdx] = sort(dists2);
        nCheck = min(2, numel(sortIdx));
        bestNCC = -1;
        for ci = 1:nCheck
            nearWP = sortIdx(ci);
            sigs = z.wpDepthSigs{nearWP};
            sigAngles = linspace(0, 2*pi*(1-1/size(sigs,1)), size(sigs,1))';
            [~, bestOri] = min(abs(angDiff(sigAngles, pose(3))));
            refScan = sigs(bestOri,:)';
            ncc = scanNCC(meas, refScan);
            if ncc > bestNCC; bestNCC = ncc; end
        end
        w_sig = exp((bestNCC - 1) / 0.3);
        w = w^(1-sigWeight) * (w * w_sig)^sigWeight;
    end
end

% ---- bearing + range beacon likelihood ----
if ~isempty(z.beacons)
    for bi = 1:size(z.beacons,1)
        tagID = z.beacons(bi,1); lb = z.beacons(bi,2); dm = z.beacons(bi,3);
        bRow = beaconLoc(beaconLoc(:,1)==tagID,:);
        if isempty(bRow); continue; end
        bx = bRow(1,2); by = bRow(1,3);
        worldAng = atan2(by-pose(2), bx-pose(1));
        predLB = mod(worldAng - pose(3) + pi, 2*pi) - pi;
        angErr = atan2(sin(lb-predLB), cos(lb-predLB));
        w = w * exp(-0.5*(angErr/sigma_beacon)^2);
        predDist = sqrt((bx-pose(1))^2 + (by-pose(2))^2);
        w = w * exp(-0.5*((dm-predDist)/sigma_range)^2);
    end
end

% ---- spin-beacon range-only likelihood ----
if ~isempty(z.spinBeacons)
    seenIDs = unique(z.spinBeacons(:,1));
    for ti = 1:numel(seenIDs)
        tid = seenIDs(ti);
        rows = z.spinBeacons(z.spinBeacons(:,1)==tid, 3);
        meanDist = mean(rows);
        bRow = beaconLoc(beaconLoc(:,1)==tid,:);
        if isempty(bRow); continue; end
        bx = bRow(1,2); by = bRow(1,3);
        predDist = sqrt((bx-pose(1))^2 + (by-pose(2))^2);
        w = w * exp(-0.5*((meanDist-predDist)/sigma_range)^2);
    end
end

if ~isfinite(w) || w < 1e-300; w = 1e-300; end
end




% =========================================================================
% depth visualization
% =========================================================================
function updateDepthViz(latestDepth, rsAngles, pose_est, accPC, map, phase, t)
persistent hDFig hAxPolar hAxCart hPolarLine hCartPC hCartObs hCartPose hDTitle

if isempty(hDFig) || ~isvalid(hDFig)
    hDFig = figure('Name','Depth Sweep Visualizer','NumberTitle','off', ...
        'Position',[1280 50 700 640], 'Color',[0.10 0.10 0.12]);

    hAxPolar = subplot(1,2,1,'Parent',hDFig);
    set(hAxPolar,'Color',[0.10 0.10 0.12],'XColor',[0.7 0.7 0.7], ...
        'YColor',[0.7 0.7 0.7],'GridColor',[0.3 0.3 0.3]);
    hold(hAxPolar,'on'); grid(hAxPolar,'on');
    xlabel(hAxPolar,'angle (rad)','Color',[0.8 0.8 0.8]);
    ylabel(hAxPolar,'depth (m)','Color',[0.8 0.8 0.8]);
    title(hAxPolar,'depth sweep — polar','Color',[0.9 0.9 0.9]);
    xlim(hAxPolar,[-35 35]*pi/180);
    ylim(hAxPolar,[0 3.0]);
    hPolarLine = plot(hAxPolar, NaN, NaN, 'o-', 'Color',[0.20 0.85 0.60], ...
        'LineWidth',1.5,'MarkerSize',5,'MarkerFaceColor',[0.20 0.85 0.60]);

    hAxCart = subplot(1,2,2,'Parent',hDFig);
    set(hAxCart,'Color',[0.10 0.10 0.12],'XColor',[0.7 0.7 0.7], ...
        'YColor',[0.7 0.7 0.7],'GridColor',[0.3 0.3 0.3]);
    hold(hAxCart,'on'); axis(hAxCart,'equal'); grid(hAxCart,'on');
    xlabel(hAxCart,'x (m)','Color',[0.8 0.8 0.8]);
    ylabel(hAxCart,'y (m)','Color',[0.8 0.8 0.8]);
    title(hAxCart,'accumulated point cloud + wall outline','Color',[0.9 0.9 0.9]);

    for i = 1:size(map,1)
        plot(hAxCart, [map(i,1) map(i,3)], [map(i,2) map(i,4)], ...
            '-','Color',[0.4 0.4 0.5],'LineWidth',1);
    end

    hCartObs = scatter(hAxCart, NaN, NaN, 12, [0.20 0.85 0.60], 'filled', ...
        'MarkerFaceAlpha',0.5);

    hCartPC = scatter(hAxCart, NaN, NaN, 4, [0.9 0.5 0.1], 'filled', ...
        'MarkerFaceAlpha',0.15);

    hCartPose = quiver(hAxCart, NaN, NaN, NaN, NaN, 0, ...
        'Color',[1 0.3 0.3],'LineWidth',2,'MaxHeadSize',3);

    hDTitle = sgtitle(hDFig, '', 'Color',[0.9 0.9 0.9]);
end

if ~isempty(latestDepth) && numel(latestDepth) == numel(rsAngles)
    validIdx = isfinite(latestDepth) & latestDepth>0.05 & latestDepth<3.0;
    set(hPolarLine, 'XData', rsAngles(validIdx), 'YData', latestDepth(validIdx));
else
    set(hPolarLine,'XData',NaN,'YData',NaN);
end

if ~isempty(accPC)
    set(hCartPC,'XData',accPC(:,1),'YData',accPC(:,2));
else
    set(hCartPC,'XData',NaN,'YData',NaN);
end

if ~isempty(latestDepth) && ~isempty(pose_est)
    pts = depthToWorldPoints(pose_est, latestDepth, rsAngles, [0.13,0]);
    if ~isempty(pts)
        set(hCartObs,'XData',pts(:,1),'YData',pts(:,2));
    end
end

if ~isempty(pose_est)
    set(hCartPose,'XData',pose_est(1),'YData',pose_est(2), ...
        'UData',0.2*cos(pose_est(3)),'VData',0.2*sin(pose_est(3)));
end

set(hDTitle,'String',sprintf('phase: %s  |  t=%.1fs  |  cloud pts: %d', ...
    phase, t, size(accPC,1)));
drawnow limitrate;
end


% =========================================================================
% helper functions
% =========================================================================
function pts = depthToWorldPoints(pose, depth, rsAngles, sensorOrigin)
valid = isfinite(depth) & depth>0.05 & depth<3.0;
if ~any(valid); pts = zeros(0,2); return; end
angles  = rsAngles(valid);
dvals   = depth(valid);
sx = pose(1) + sensorOrigin(1)*cos(pose(3));
sy = pose(2) + sensorOrigin(1)*sin(pose(3));
worldAngles = pose(3) + angles;
pts = [sx + dvals.*cos(worldAngles), sy + dvals.*sin(worldAngles)];
end


function ncc = scanNCC(a, b)
valid = isfinite(a) & isfinite(b) & a>0.05 & a<3.0 & b>0.05 & b<3.0;
if nnz(valid) < 3; ncc = 0; return; end
av = a(valid); bv = b(valid);
av = av - mean(av); bv = bv - mean(bv);
na = norm(av); nb = norm(bv);
if na < 1e-9 || nb < 1e-9; ncc = 0; return; end
ncc = dot(av,bv) / (na*nb);
ncc = max(-1, min(1, ncc));
end


function d = angDiff(a, b)
d = mod(a - b + pi, 2*pi) - pi;
end



% =========================================================================
% particle init helpers
% =========================================================================

function particles = initParticlesAtWaypoints(waypoints, N)
k = size(waypoints,1); nPerWP = ceil(N/k);
oris = linspace(0, 2*pi*(1-1/nPerWP), nPerWP)';
particles = zeros(nPerWP*k, 4);
row = 1;
for wi = 1:k
    for oi = 1:nPerWP
        j = 0.04*randn(1,2);
        particles(row,:) = [waypoints(wi,1)+j(1), waypoints(wi,2)+j(2), oris(oi), 1/(nPerWP*k)];
        row = row+1;
    end
end
particles = particles(1:N,:);
particles(:,4) = particles(:,4) / sum(particles(:,4));
end


function particles = initParticlesBeaconGuided( ...
    waypoints, beaconLoc, wpBeaconVis, seenTagIDs, N, feasibleFrac)
nWP = size(waypoints,1);

seenColIdx = [];
for ti = 1:numel(seenTagIDs)
    col = find(beaconLoc(:,1) == seenTagIDs(ti));
    if ~isempty(col); seenColIdx(end+1) = col; end %#ok<AGROW>
end

feasibleMask = false(nWP,1);
if ~isempty(seenColIdx)
    for wi = 1:nWP
        if any(wpBeaconVis(wi, seenColIdx)); feasibleMask(wi) = true; end
    end
end
if ~any(feasibleMask)
    fprintf('[seed] no feasible wps — using all\n');
    feasibleMask(:) = true;
end

feasibleWPs = find(feasibleMask);
nFeasible   = numel(feasibleWPs);
fprintf('[seed] feasible wps: %s (%d/%d)\n', mat2str(feasibleWPs'), nFeasible, nWP);

nFeasPart  = round(N * feasibleFrac);
nHedgePart = N - nFeasPart;

nPerFeas = ceil(nFeasPart / nFeasible);
oris = linspace(0, 2*pi*(1-1/nPerFeas), nPerFeas)';
fParts = zeros(nPerFeas*nFeasible, 4); row = 1;
for wi = 1:nFeasible
    wpIdx = feasibleWPs(wi);
    for oi = 1:nPerFeas
        j = 0.04*randn(1,2);
        fParts(row,:) = [waypoints(wpIdx,1)+j(1), waypoints(wpIdx,2)+j(2), oris(oi), 1];
        row = row+1;
    end
end
fParts = fParts(1:nFeasPart,:);

nPerHedge = ceil(nHedgePart / nWP);
oris2 = linspace(0, 2*pi*(1-1/nPerHedge), nPerHedge)';
hParts = zeros(nPerHedge*nWP, 4); row = 1;
for wi = 1:nWP
    for oi = 1:nPerHedge
        j = 0.04*randn(1,2);
        hParts(row,:) = [waypoints(wi,1)+j(1), waypoints(wi,2)+j(2), oris2(oi), 1];
        row = row+1;
    end
end
hParts = hParts(1:nHedgePart,:);

particles = [fParts; hParts];
particles = particles(1:N,:);
particles(:,4) = 1/N;
fprintf('[seed] %d feasible + %d hedge particles\n', nFeasPart, nHedgePart);
end


function particles = initParticlesAroundPose(pose, N, sigXY, sigTheta)
particles = zeros(N,4);
particles(:,1) = pose(1) + sigXY*randn(N,1);
particles(:,2) = pose(2) + sigXY*randn(N,1);
particles(:,3) = mod(pose(3) + sigTheta*randn(N,1) + pi, 2*pi) - pi;
particles(:,4) = ones(N,1) / N;
end


function [convReached, bestWPidx, bestMass] = checkConvergence( ...
    particles, waypoints, radius, threshold)
nWP    = size(waypoints,1);
totalW = sum(particles(:,4));
if totalW < 1e-12; convReached=false; bestWPidx=1; bestMass=0; return; end
masses = zeros(nWP,1);
for wi = 1:nWP
    inR = sqrt((particles(:,1)-waypoints(wi,1)).^2 + ...
               (particles(:,2)-waypoints(wi,2)).^2) < radius;
    masses(wi) = sum(particles(inR,4)) / totalW;
end
[bestMass, bestWPidx] = max(masses);
convReached = bestMass >= threshold;
end


function poses_new = motionModel(poses, u, sigma_d, sigma_a)
N = size(poses,1); deltaD = u(1); deltaA = u(2);
nd = deltaD + sigma_d*randn(N,1);
na = deltaA + sigma_a*randn(N,1);
poses_new = zeros(N,3);
poses_new(:,1) = poses(:,1) + nd.*cos(poses(:,3));
poses_new(:,2) = poses(:,2) + nd.*sin(poses(:,3));
poses_new(:,3) = mod(poses(:,3) + na + pi, 2*pi) - pi;
end

function [cmdV, cmdW, gotopt] = followPathWithPose(path, gotopt, pose_est, closeEnough, epsilon)

if gotopt > size(path,1)
    cmdV = 0;
    cmdW = 0;
    return;
end

target = path(gotopt,:);

x = pose_est(1);
y = pose_est(2);
theta = pose_est(3);

dx = target(1) - x;
dy = target(2) - y;

dist = sqrt(dx^2 + dy^2);

if dist < closeEnough
    gotopt = gotopt + 1;

    if gotopt > size(path,1)
        cmdV = 0;
        cmdW = 0;
        return;
    end

    target = path(gotopt,:);
    dx = target(1) - x;
    dy = target(2) - y;
    dist = sqrt(dx^2 + dy^2);
end

desiredTheta = atan2(dy, dx);
headingError = wrapToPiLocal(desiredTheta - theta);

% kV: proportional gain for linear velocity (controls how aggressively robot moves toward target)
% kW: proportional gain for angular velocity (controls how aggressively robot turns to correct heading)
kV = 0.7;
kW = 0.3;

if abs(headingError) > epsilon
    cmdV = 0;
else
    cmdV = kV * dist;
end

cmdW = kW * headingError;

end

function ang = wrapToPiLocal(ang)
ang = mod(ang + pi, 2*pi) - pi;
end

% =========================================================================
% wall belief update
% =========================================================================
function wb = updateWallBelief(wb, optWalls, pose, depth, rsAngles, sensorOrigin, map)
sx = pose(1) + sensorOrigin(1)*cos(pose(3));
sy = pose(2) + sensorOrigin(1)*sin(pose(3));

PRESENT_THRESH = 8;
ABSENT_THRESH = -8;
MAX_WALL_DIST = 2.5;
DEPTH_TOL = 0.18;

for wi = 1:size(optWalls,1)
    mx = (optWalls(wi,1) + optWalls(wi,3)) / 2;
    my = (optWalls(wi,2) + optWalls(wi,4)) / 2;

    wallDist = sqrt((mx-sx)^2 + (my-sy)^2);
    if wallDist > MAX_WALL_DIST; continue; end

    worldBearing = atan2(my-sy, mx-sx);
    localBearing = mod(worldBearing - pose(3) + pi, 2*pi) - pi;

    [~, rayIdx] = min(abs(rsAngles - localBearing));
    if abs(rsAngles(rayIdx) - localBearing) > (rsAngles(2)-rsAngles(1))*1.5
        continue;
    end

    meas = depth(rayIdx);
    if ~isfinite(meas) || meas <= 0.05; continue; end

    blocked = false;
    for mi = 1:size(map,1)
        [isect,~,~] = intersectPoint(sx,sy,mx,my, ...
            map(mi,1),map(mi,2),map(mi,3),map(mi,4));
        if isect; blocked = true; break; end
    end
    if blocked; continue; end

    if abs(meas - wallDist) < DEPTH_TOL
        wb.score(wi) = wb.score(wi) + 1;
    elseif meas > wallDist + DEPTH_TOL
        wb.score(wi) = wb.score(wi) - 1;
    end

    if wb.score(wi) >= PRESENT_THRESH
        wb.state(wi) = 1;
    elseif wb.score(wi) <= ABSENT_THRESH
        wb.state(wi) = -1;
    end
end
end


% =========================================================================
% live map plot
% =========================================================================
function updateLivePlot(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
    allWaypoints, wpVisited, particles, pose_est, navPath, navPathIdx, phase, t, postConvergence)
persistent hFig hAx hParticles hBestArrow hWPs hTitle hPath

nMand = size(waypoints,1);

if isempty(hFig) || ~isvalid(hFig)
    hFig = figure('Name','Live Robot State','NumberTitle','off','Position',[50 50 1200 560]);
    hAx = axes('Parent',hFig);

    hold(hAx,'on'); axis(hAx,'equal'); grid(hAx,'on');
    xlabel(hAx,'x (m)'); ylabel(hAx,'y (m)');
    title(hAx,'particle map (blue=low, red=high weight)');

    for i = 1:size(map,1)
        plot(hAx,[map(i,1) map(i,3)],[map(i,2) map(i,4)],'k-','LineWidth',2);
    end
    for i = 1:size(optWalls,1)
        plot(hAx,[optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'b--','LineWidth',1);
    end
    for bi = 1:size(beaconLoc,1)
        plot(hAx,beaconLoc(bi,2),beaconLoc(bi,3),'ms','MarkerSize',10,'LineWidth',2);
        text(beaconLoc(bi,2)+0.05, beaconLoc(bi,3)+0.05, ...
            sprintf('t%d',beaconLoc(bi,1)),'Parent',hAx,'Color','m','FontSize',8);
    end

    hParticles = scatter(hAx, NaN, NaN, 10, [0 0 1], 'filled','MarkerFaceAlpha',0.5);
    hBestArrow = quiver(hAx, NaN, NaN, NaN, NaN, 0, 'r','LineWidth',2,'MaxHeadSize',3);
    hPath = plot(hAx, NaN, NaN, 'g-','LineWidth',2);

    hWPs = gobjects(size(allWaypoints,1),1);
    for i = 1:size(allWaypoints,1)
        mk = 'o'; if i > nMand; mk = 'p'; end
        hWPs(i) = plot(hAx, allWaypoints(i,1), allWaypoints(i,2), ...
            mk,'MarkerSize',10,'LineWidth',2,'Color',[0.8 0 0]);
        text(allWaypoints(i,1)+0.05, allWaypoints(i,2)+0.05, ...
            sprintf('wp%d',i),'Parent',hAx,'FontSize',8);
    end
    hTitle = title(hAx,'');

end

% update bar chart only during localization (freeze after convergence)
% particles removed from display
% if ~isempty(particles) && ~postConvergence
%     pw = particles(:,4);
%     pwn = pw / (max(pw)+1e-12);
%     cmap = interp1([0;0.5;1],[0 0 1;1 1 0;1 0 0], pwn);
%     set(hParticles,'XData',particles(:,1),'YData',particles(:,2),'CData',cmap);
% 
% elseif ~postConvergence
%     set(hParticles,'XData',NaN,'YData',NaN);
% end

% always hide particles
set(hParticles,'XData',NaN,'YData',NaN);

if ~isempty(pose_est)
    L = 0.25;
    set(hBestArrow,'XData',pose_est(1),'YData',pose_est(2), ...
        'UData',L*cos(pose_est(3)),'VData',L*sin(pose_est(3)));
end

if ~isempty(navPath) && navPathIdx <= size(navPath,1)
    set(hPath,'XData',navPath(navPathIdx:end,1),'YData',navPath(navPathIdx:end,2));
else
    set(hPath,'XData',NaN,'YData',NaN);
end

for i = 1:size(allWaypoints,1)
    if wpVisited(i); set(hWPs(i),'Color',[0 0.7 0]);
    else; set(hWPs(i),'Color',[0.8 0 0]); end
end
set(hTitle,'String',sprintf('phase: %s  |  t=%.1fs', phase, t));
drawnow limitrate;
end


% =========================================================================
% final map plot
% =========================================================================
function plotFinalMap(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
    allWaypoints, wpVisited, wallBelief, pose_est, accPC)
nMand = size(waypoints,1);
figure('Name','Final Map','NumberTitle','off');
hold on; axis equal; grid on;
title('final map'); xlabel('x (m)'); ylabel('y (m)');

for i = 1:size(map,1)
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'k-','LineWidth',2);
end

if ~isempty(wallBelief) && isfield(wallBelief,'state')
    for i = 1:size(optWalls,1)
        s = wallBelief.state(i);
        if s==1; c='k-'; elseif s==-1; c='g--'; else; c='r-'; end
        plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],c,'LineWidth',1.5);
    end
else
    for i = 1:size(optWalls,1)
        plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'b--','LineWidth',1);
    end
end

if ~isempty(accPC)
    scatter(accPC(:,1), accPC(:,2), 3, [0.9 0.5 0.1], 'filled', 'MarkerFaceAlpha',0.3);
end

for i = 1:size(allWaypoints,1)
    isEC = i>nMand;
    if wpVisited(i); clr=[0 0.7 0]; mk='o'; else; clr=[0.8 0 0]; mk='x'; end
    sz=10; if isEC; sz=14; end
    plot(allWaypoints(i,1),allWaypoints(i,2),mk,'Color',clr,'MarkerSize',sz,'LineWidth',2);
    text(allWaypoints(i,1)+0.05,allWaypoints(i,2)+0.05, ...
        sprintf('wp%d%s',i,repmat('*',1,double(isEC))),'FontSize',8);
end

for bi = 1:size(beaconLoc,1)
    plot(beaconLoc(bi,2),beaconLoc(bi,3),'ms','MarkerSize',10,'LineWidth',2);
    text(beaconLoc(bi,2)+0.05,beaconLoc(bi,3)+0.05, ...
        sprintf('t%d',beaconLoc(bi,1)),'Color','m','FontSize',8);
end

if ~isempty(pose_est)
    quiver(pose_est(1),pose_est(2), ...
        0.2*cos(pose_est(3)),0.2*sin(pose_est(3)),'b','LineWidth',2,'MaxHeadSize',2);
    plot(pose_est(1),pose_est(2),'b.','MarkerSize',15);
end
hold off;
end