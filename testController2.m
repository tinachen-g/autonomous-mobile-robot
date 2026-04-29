function[dataStore] = testController2(Robot,maxTime)
% testController2: simple skeleton program to use with iRobot Create (or simulator).
%
%   dataStore = testCONTROLLER(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
% 
%   technique: 
% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

MAPFILE = 'practicemap2025update.mat';
data = load(MAPFILE);
map = data.map;
optWalls = data.optWalls;
waypoints = data.waypoints;
ECwaypoints = data.ECwaypoints;
beaconLoc = data.beaconLoc;

nMandatory = size(waypoints,1);
nEC = size(ECwaypoints,1);
allWaypoints = [waypoints; ECwaypoints];

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose',          [], ...
                   'odometry',           [], ...
                   'rsdepth',            [], ...
                   'bump',               [], ...
                   'beacon',             []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;
wheel2Center = 0.13;
maxV = 0.2;
sensorOrigin = [0.13, 0];


% spin / navigation parameters
SPIN_RATE = 0.4;
SPIN_DURATION = (2*pi / SPIN_RATE) * 2;


RS_FOV = 58 * pi/180;
NUM_RAYS = 9; % RealSenseDist outputs depth_array(1) = delay in reading
% depth_array(2:10) = depth to point from left to right
rsAngles = linspace(-RS_FOV/2, RS_FOV/2, NUM_RAYS)';  % [9x1], 0=fwd


% -------------------------------------------------------------------------
% particle filter parameters
% -------------------------------------------------------------------------
N_PARTICLES  = 600;
SIGMA_D = 0.02;   % forward odometry noise (m)
SIGMA_A = 0.03;   % angular odometry noise (rad)
SIGMA_DEPTH = 0.12;   % depth likelihood std dev (m)
SIGMA_BEACON = 0.12;   % bearing likelihood std dev (rad)
SIGMA_RANGE = 0.15;   % range-only beacon likelihood std dev (m)


% -------------------------------------------------------------------------
% convergence parameters
%   CONV_MASS_THR: fraction of total weight that must accumulate at a
%   single waypoint (within CONV_WP_RADIUS) to declare convergence
%   CONV_WP_RADIUS: radius around each waypoint center (m) to count mas
% -------------------------------------------------------------------------
CONV_MASS_THR  = 0.65;   % 65% of particle weight at one waypoint
CONV_WP_RADIUS = 0.30;   % m

% -------------------------------------------------------------------------
% beacon-guided seeding parameters
%   SEED_FEASIBLE_FRAC: fraction of particles to seed at beacon-feasible
%   WPs where remainder go uniformly across all non-EC WP.
%       remainder fraction kept spread across all waypoints so the filter can 
%       recover if it locked to the wrong starting position
%   BEACON_SEED_RANGE: max range at which a beacon is considered detectable
%   for seeding purposes
% -------------------------------------------------------------------------
SEED_FEASIBLE_FRAC = 0.85;
BEACON_SEED_RANGE  = 3.0;   % in m, must match BEACON_VIS_RANGE

% -------------------------------------------------------------------------
% orientation sweep parameters
% -------------------------------------------------------------------------
N_ORI_COARSE = 72;
N_ORI_FINE   = 40;
ORI_FINE_WIN = 15 * pi/180;

% -------------------------------------------------------------------------
% state machine
% -------------------------------------------------------------------------
phase = 'SPIN';
phaseStart = 0;

spinBeacons   = [];
bestDepthScan = [];

particles = [];
dr_pose   = [0, 0, 0];
pose_est  = [];
startWaypoint = [];

wpVisited  = false(size(allWaypoints,1), 1);
navPath  = [];
navPathIdx   = 1;

wallBelief = [];

predict_func = @(poses,u) motionModel(poses, u, SIGMA_D, SIGMA_A);



% =========================================================================
% build beacon-waypoint feasibility
%   wpBeaconVis(wi, bi) = true means waypoint wi can see beacon bi
%   stored for reuse during beacon-guided seeding after SPIN
% =========================================================================
BEACON_VIS_RANGE = BEACON_SEED_RANGE;
fprintf('\n=== BEACON-WAYPOINT VISIBILITY (range=%.1fm, wall-occluded) ===\n', ...
    BEACON_VIS_RANGE);

nBeacons = size(beaconLoc, 1);
wpBeaconVis = false(nMandatory, nBeacons);   % [nWP x nBeacon]

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
            wpBeaconVis(wi, bi) = true;
            visible(end+1) = beaconLoc(bi,1); %#ok<AGROW>
        end
    end
    if isempty(visible)
        fprintf('  WP%d (%.2f,%.2f): no beacons visible\n', wi, wx, wy);
    else
        fprintf('  WP%d (%.2f,%.2f): beacons %s\n', wi, wx, wy, mat2str(visible));
    end
end
fprintf('=================================================================\n\n');


%% =========================================================================
% MAIN LOOP
% =========================================================================
SetFwdVelAngVelCreate(Robot, 0,0);
tic
while toc < maxTime

    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);

    %%  ODOMETRY readings
    deltaD = 0; 
    deltaA = 0;
    try
        deltaD = DistanceSensorRoomba(CreatePort);
        deltaA = AngleSensorRoomba(CreatePort);

        dataStore.odometry = [dataStore.odometry; toc deltaD deltaA];
        % store raw dead-reckoning path
    catch
        disp('[PHASE I] Error retrieving or saving odometry data.');
    end

    dr_pose(1) = dr_pose(1) + deltaD*cos(dr_pose(3));
    dr_pose(2) = dr_pose(2) + deltaD*sin(dr_pose(3));
    dr_pose(3) = mod(dr_pose(3)+deltaA+pi, 2*pi)-pi;
    u = [deltaD; deltaA];

    %% BEACON observations (CHECK DURING OPEN LAB)
    currBeaconObs = [];
    try
        tags = RealSenseTag(Robot); % [dt id z x yaw]
        if ~isempty(tags)
            for ti = 1:size(tags,1)
                tagID = tags(ti,2);
                tagZ = tags(ti,3); % distance between the center of the tag and the camera
                tagX = tags(ti,4); % points to robot left
                dist_m = sqrt(tagZ^2 + tagX^2);
                localBearing = atan2(tagX, tagZ);
                currBeaconObs(end+1,:) = [tagID, localBearing, dist_m]; %#ok<AGROW>
            end
        end
    catch
        disp('[PHASE I] no beacon available.');
    end
    if ~isempty(currBeaconObs)
        spinBeacons = [spinBeacons; currBeaconObs]; %#ok<AGROW>
    end


    %% SENSOR, depth and bump! readings
    latestDepth = [];
    if ~isempty(dataStore.rsdepth)
        row = dataStore.rsdepth(end, :);
        if numel(row) >= 11
            latestDepth = row(3:end)';
        elseif numel(row) >= 3
            latestDepth = row(3:end)';
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


    % =====================================================================
    % PARTICLE FILTER update
    % =====================================================================
    if strcmp(phase,'LOCALIZE_POS') && ~isempty(particles)
        % Depth is orientation-dependent — do NOT use it until heading is known.
        % Range-only beacon constraint is heading-independent and sufficient for XY.
        z.depth       = [];           % ← was latestDepth, now disabled
        z.beacons     = [];           % bearing also off — heading unknown
        z.spinBeacons = spinBeacons;
        lh = @(p,z2) measurementLikelihood(p, z2, map, sensorOrigin, rsAngles, ...
            beaconLoc, SIGMA_DEPTH, SIGMA_BEACON, SIGMA_RANGE);
        [particles, best_particle, ~, ~] = PF2(particles, u, z, predict_func, lh);
        pose_est = best_particle;
    end
    if strcmp(phase,'NAVIGATE') && ~isempty(particles) && ~isempty(latestDepth)
        % during navigate: full depth + bearing beacon likelihood
        z.depth = latestDepth;
        z.beacons = currBeaconObs;
        z.spinBeacons = [];   % not used during navigate
        lh = @(p,z2) measurementLikelihood(p, z2, map, sensorOrigin, rsAngles, ...
            beaconLoc, SIGMA_DEPTH, SIGMA_BEACON, SIGMA_RANGE);
        [particles, best_particle, ~, ~] = PF2(particles, u, z, predict_func, lh);
        pose_est = best_particle;
    end

    
    %% CONTROL FUNCTION (send robot commands)

    switch phase
        %TURN in place: robot 360 2 times
        case 'SPIN'
            elapsed = toc - phaseStart;
            if elapsed < SPIN_DURATION
                [cmdV, cmdW] = limitCmds(0, SPIN_RATE, maxV, wheel2Center);
                SetFwdVelAngVelCreate(Robot, cmdV, cmdW);

            else
                SetFwdVelAngVelCreate(Robot, 0, 0);

                % determine which beacons were seen and their mean distances
                if isempty(spinBeacons)
                    fprintf('[SPIN] Done. No beacons — seeding at all mandatory WPs.\n');
                    particles = initParticlesAtWaypoints(waypoints, N_PARTICLES);
                else
                    seenTagIDs = unique(spinBeacons(:,1));
                    fprintf('[SPIN] Done. Beacons seen: %s\n', mat2str(seenTagIDs'));

                    % compute mean observed range per tag for diagnostic logging
                    for ti = 1:numel(seenTagIDs)
                        tid = seenTagIDs(ti);
                        rows = spinBeacons(spinBeacons(:,1)==tid, 3);
                        fprintf('  Tag %d: %d obs, mean range=%.2fm\n', ...
                            tid, numel(rows), mean(rows));
                    end

                    % seed particles using beacon visibility constraints
                    particles = initParticlesBeaconGuided( ...
                        waypoints, beaconLoc, wpBeaconVis, ...
                        seenTagIDs, N_PARTICLES, ...
                        SEED_FEASIBLE_FRAC);
                end

                fprintf('[SPIN->LOC_POS] %d particles initialized\n', N_PARTICLES);
                phase = 'LOCALIZE_POS';
                phaseStart = toc;
            
            end

        case 'LOCALIZE_POS'
            elapsed = toc - phaseStart;
            if ~isempty(particles)
                [convReached, bestWPidx, bestMass] = checkConvergence( ...
                    particles, waypoints, CONV_WP_RADIUS, CONV_MASS_THR);

                posStd = std(particles(:,1:2), 0, 1);
                fprintf('[LOC_POS] t=%.1fs  posStd=(%.3f,%.3f)m  bestWP=%d mass=%.3f\n', ...
                    toc, posStd(1), posStd(2), bestWPidx, bestMass);

                for wpi = 1:nMandatory
                    inR = sqrt((particles(:,1)-waypoints(wpi,1)).^2 + ...
                               (particles(:,2)-waypoints(wpi,2)).^2) < CONV_WP_RADIUS;
                    fprintf('  WP%d mass=%.3f\n', wpi, sum(particles(inR,4)));
                end

                if convReached || elapsed > 20.0
                    if elapsed > 20.0 && ~convReached
                        fprintf('[LOC_POS] Timeout — using highest-mass WP.\n');
                    else
                        fprintf('[LOC_POS] Converged (mass=%.2f at WP%d).\n', ...
                            bestMass, bestWPidx);
                    end

                    % snap to nearest non-EC waypoint
                    dists = sqrt((waypoints(:,1)-pose_est(1)).^2 + ...
                                 (waypoints(:,2)-pose_est(2)).^2);
                    [snapD, startWaypoint] = min(dists);
                    pose_est(1) = waypoints(startWaypoint,1);
                    pose_est(2) = waypoints(startWaypoint,2);
                    fprintf('[LOC_POS] -> WP%d (%.2f,%.2f) snap=%.2fm\n', ...
                        startWaypoint, pose_est(1), pose_est(2), snapD);
                    phase = 'ORIENT'; phaseStart = toc;
                end
            end
        case 'ORIENT'
        % -----------------------------------------------------------------
            SetFwdVelAngVelCreate(Robot, 0, 0);
            fprintf('[ORIENT] print out the current orientation: see what initial guess is:\n');
            fprintf('        theta=%.3frad (%.1fdeg)\n', ...
                    pose_est(3), pose_est(3)*180/pi);

            if ~isempty(spinBeacons)
                fprintf('[ORIENT] check if beacons, turn towards one if possible?\n');
            else
                fprintf('[ORIENT] No beacons — depth orientation sweep.\n');
            end

            particles = initParticlesAroundPose(pose_est, N_PARTICLES, 0.05, 0.10);
            dr_pose = pose_est;
            fprintf('[ORIENT] Pose locked: (%.3f,%.3f,%.3frad)\n', ...
                pose_est(1), pose_est(2), pose_est(3));

            wpVisited(startWaypoint) = true;
            BeepCreate(Robot);
            phase = 'DONE';
            phaseStart = toc;


        case 'EXPLORE'
        

        case 'PLANNING'
            % ================= TODO =================:
            % now we know where we are in the world
            % implement algorithm to visit each waypoint



        case 'DONE'
            SetFwdVelAngVelCreate(Robot, 0, 0);
            plotFinalMap(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
                allWaypoints, wpVisited, wallBelief, pose_est);
            break;

    end % switch cases

    % live plot
    updateLivePlot(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
        allWaypoints, wpVisited, particles, pose_est, navPath, navPathIdx, phase, toc);

    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 5
        SetFwdVelAngVelCreate(Robot, 0,0);
        break;
    end
    
end

SetFwdVelAngVelCreate(Robot, 0,0 );

fprintf('[END CONTROLLER] t=%.1fs\n', toc);
end



% --------------------- HELPERs -------------------------

function particles = initParticlesAtWaypoints(waypoints, N)
% seed uniformly across all waypoints with uniform orientations
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

% -------------------------------------------------------------------------
function particles = initParticlesBeaconGuided( ...
    waypoints, beaconLoc, wpBeaconVis, seenTagIDs, N, feasibleFrac)
% seed particles using beacon visibility constraints.
%
%     a waypoint is "feasible" if it can see AT LEAST ONE of the beacons
%     the robot observed. waypoints that can't see any observed beacon are
%     implausible given the measurement(down-weight them)
%
%     feasibleFrac (default 0.85) of particles go to feasible WPs.
%     (1 - feasibleFrac) go uniformly across ALL mandatory WPs as a
%     recovery hedge, in case a beacon was mis-detected or occluded.
%
%   note: don't require the WP to see ALL observed beacons, just any one
%   conservative bc beacon FOV is limited, some
%   observations during the spin may miss
    
    nWP = size(waypoints,1);
    nBeacons = size(beaconLoc,1);
    
    % map seenTagIDs to column indices in wpBeaconVis
    seenColIdx = [];
    for ti = 1:numel(seenTagIDs)
        col = find(beaconLoc(:,1) == seenTagIDs(ti));
        if ~isempty(col)
            seenColIdx(end+1) = col; %#ok<AGROW>
        end
    end
    
    % feasible = WP that can see at least one observed beacon
    feasibleMask = false(nWP,1);
    if ~isempty(seenColIdx)
        for wi = 1:nWP
            if any(wpBeaconVis(wi, seenColIdx))
                feasibleMask(wi) = true;
            end
        end
    end
    
    % fallback: if no feasible WPs found,
    % treat all as feasible to avoid seeding zero particles
    if ~any(feasibleMask)
        fprintf('[SEED] WARNING: no feasible WPs found — using all WPs\n');
        feasibleMask(:) = true;
    end
    
    feasibleWPs  = find(feasibleMask);
    allWPs = (1:nWP)';
    nFeasible1 = numel(feasibleWPs);
    
    fprintf('[SEED] Feasible WPs: %s (%d/%d)\n', mat2str(feasibleWPs'), nFeasible, nWP);
    
    nFeasPart  = round(N * feasibleFrac);
    nHedgePart = N - nFeasPart;
    
    % particles at feasible waypoints
    nPerFeas = ceil(nFeasPart / nFeasible);
    oris = linspace(0, 2*pi*(1-1/nPerFeas), nPerFeas)';
    fParts = zeros(nPerFeas*nFeasible, 4);
    row = 1;
    for wi = 1:nFeasible
        wpIdx = feasibleWPs(wi);
        for oi = 1:nPerFeas
            j = 0.04*randn(1,2);
            fParts(row,:) = [waypoints(wpIdx,1)+j(1), waypoints(wpIdx,2)+j(2), oris(oi), 1];
            row = row+1;
        end
    end
    fParts = fParts(1:nFeasPart, :);
    
    nPerHedge = ceil(nHedgePart / nWP);
    oris2 = linspace(0, 2*pi*(1-1/nPerHedge), nPerHedge)';
    hParts = zeros(nPerHedge*nWP, 4);
    row = 1;
    for wi = 1:nWP
        for oi = 1:nPerHedge
            j = 0.04*randn(1,2);
            hParts(row,:) = [waypoints(wi,1)+j(1), waypoints(wi,2)+j(2), oris2(oi), 1];
            row = row+1;
        end
    end
    hParts = hParts(1:nHedgePart, :);
    
    % combine and normalize
    particles = [fParts; hParts];
    particles = particles(1:N, :);
    particles(:,4) = 1/N;   % uniform initial weights — likelihood will differentiate
    
    fprintf('[SEED] %d feasible + %d hedge particles\n', nFeasPart, nHedgePart);
end

% -------------------------------------------------------------------------
function particles = initParticlesAroundPose(pose, N, sigXY, sigTheta)
% tight Gaussian cloud around a known pose — used after ORIENT
particles = zeros(N,4);
particles(:,1) = pose(1) + sigXY*randn(N,1);
particles(:,2) = pose(2) + sigXY*randn(N,1);
particles(:,3) = mod(pose(3) + sigTheta*randn(N,1) + pi, 2*pi) - pi;
particles(:,4) = ones(N,1) / N;
end

% -------------------------------------------------------------------------
function w = measurementLikelihood(pose, z, map, sensorOrigin, rsAngles, ...
    beaconLoc, sigma_depth, sigma_beacon, sigma_range)
% combined depth + bearing beacon + range-only beacon likelihood.
%
%   z is a struct with fields:
%     z.depth      [9x1] real depth scan (used every call)
%     z.beacons    [Mx3] [tagID, localBearing, dist_m] — heading-aware,
%                  only populated during NAVIGATE
%     z.spinBeacons [Mx3] same format — used during LOCALIZE_POS for
%                  range-only (heading-independent) constraint
w = 1.0;

% depth likelihood (unchanged from v1)
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

% full bearing+distance beacon likelihood (NAVIGATE phase)
if ~isempty(z.beacons)
    for bi = 1:size(z.beacons,1)
        tagID = z.beacons(bi,1); lb = z.beacons(bi,2); dm = z.beacons(bi,3);
        bRow = beaconLoc(beaconLoc(:,1)==tagID,:);
        if isempty(bRow); 
            continue; 
        end
        bx = bRow(1,2); by = bRow(1,3);
        worldAng = atan2(by-pose(2), bx-pose(1));
        predLB = mod(worldAng - pose(3) + pi, 2*pi) - pi;
        angErr = atan2(sin(lb-predLB), cos(lb-predLB));
        w = w * exp(-0.5*(angErr/sigma_beacon)^2);
        predDist = sqrt((bx-pose(1))^2 + (by-pose(2))^2);
        w = w * exp(-0.5*((dm-predDist)/0.4)^2);
    end
end

% range-only beacon likelihood (LOCALIZE_POS phase — no heading required)
%   for each observed tag, penalize particles whose euclidean distance
%   to that beacon doesn't match the measured dist_m.
%   this is purely positional and heading-independent.
if ~isempty(z.spinBeacons)
    % deduplicate: use mean observed range per tag
    seenIDs = unique(z.spinBeacons(:,1));
    for ti = 1:numel(seenIDs)
        tid  = seenIDs(ti);
        rows = z.spinBeacons(z.spinBeacons(:,1)==tid, 3);
        meanDist = mean(rows);   % average over spin observations
        bRow = beaconLoc(beaconLoc(:,1)==tid,:);
        if isempty(bRow); 
            continue; 
        end
        bx = bRow(1,2); by = bRow(1,3);
        predDist = sqrt((bx-pose(1))^2 + (by-pose(2))^2);
        w = w * exp(-0.5*((meanDist-predDist)/sigma_range)^2);
    end
end

if ~isfinite(w) || w < 1e-300; w = 1e-300; end
end

% -------------------------------------------------------------------------
function [convReached, bestWPidx, bestMass] = checkConvergence( ...
    particles, waypoints, radius, threshold)
% check convergence using waypoint mass concentration.
%
%   sums normalized particle weights within radius of each waypoint.
%   if any WP accumulates > threshold fraction, declare convergence

nWP = size(waypoints,1);
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

% -------------------------------------------------------------------------

function poses_new = motionModel(poses, u, sigma_d, sigma_a)
    N = size(poses,1); deltaD = u(1); deltaA = u(2);
    nd = deltaD + sigma_d*randn(N,1);
    na = deltaA + sigma_a*randn(N,1);
    poses_new = zeros(N,3);
    poses_new(:,1) = poses(:,1) + nd.*cos(poses(:,3));
    poses_new(:,2) = poses(:,2) + nd.*sin(poses(:,3));
    poses_new(:,3) = mod(poses(:,3) + na + pi, 2*pi) - pi;
end


% -------------------------------------------------------------------------
function updateLivePlot(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
    allWaypoints, wpVisited, particles, pose_est, navPath, navPathIdx, phase, t)
%   left:  map with particles overlaid (colored by weight)
%   right: weight mass bar chart per waypoint + convergence indicator

persistent hFig hAx hAxBar hParticles hBestArrow hWPs hTitle hBars hPath hConvLine

nMand = size(waypoints,1);

if isempty(hFig) || ~isvalid(hFig)
    hFig = figure('Name','Live Robot State','NumberTitle','off','Position',[50 50 1200 560]);
    hAx = subplot(1,2,1,'Parent',hFig);
    hAxBar = subplot(1,2,2,'Parent',hFig);

    % --- map panel ---
    hold(hAx,'on'); axis(hAx,'equal'); grid(hAx,'on');
    xlabel(hAx,'X (m)'); ylabel(hAx,'Y (m)');
    title(hAx,'Particle map (blue=low weight, red=high weight)');

    for i = 1:size(map,1)
        plot(hAx,[map(i,1) map(i,3)],[map(i,2) map(i,4)],'k-','LineWidth',2);
    end
    for i = 1:size(optWalls,1)
        plot(hAx,[optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'b--','LineWidth',1);
    end
    for bi = 1:size(beaconLoc,1)
        plot(hAx,beaconLoc(bi,2),beaconLoc(bi,3),'ms','MarkerSize',10,'LineWidth',2);
        text(beaconLoc(bi,2)+0.05, beaconLoc(bi,3)+0.05, ...
            sprintf('T%d',beaconLoc(bi,1)),'Parent',hAx,'Color','m','FontSize',8);

        
    end
    % --- mini compass (unit circle reference) ---
    theta = linspace(0, 2*pi, 100);
    r = 0.3;    % size of compass
    cx = min(map(:,[1 3]),[],'all') + 0.5;   % position (top-left-ish)
    cy = max(map(:,[2 4]),[],'all') + 0.5;
    
    plot(hAx, cx + r*cos(theta), cy + r*sin(theta), 'k:'); % circle
    
    text(cx + r, cy, ...
        sprintf('0'), ...
        'Parent', hAx, 'HorizontalAlignment','left');
    
    text(cx, cy + r, ...
        sprintf('\\pi/2\n(%.2f)', pi/2), ...
        'Parent', hAx, 'HorizontalAlignment','center');
    
    text(cx - r, cy, ...
        sprintf('\\pi\n(%.2f)', pi), ...
        'Parent', hAx, 'HorizontalAlignment','right');
    
    text(cx, cy - r, ...
        sprintf('-\\pi/2\n(%.2f)', -pi/2), ...
        'Parent', hAx, 'HorizontalAlignment','center');
    
    % axes lines
    plot(hAx, [cx cx+r], [cy cy], 'k-');
    plot(hAx, [cx cx], [cy cy+r], 'k-');

    hParticles = scatter(hAx, NaN, NaN, 10, [0 0 1], 'filled', 'MarkerFaceAlpha', 0.5);

    hBestArrow = quiver(hAx, NaN, NaN, NaN, NaN, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 3);
    hPath = plot(hAx, NaN, NaN, 'g--', 'LineWidth', 1.5);

    hWPs = gobjects(size(allWaypoints,1), 1);
    for i = 1:size(allWaypoints,1)
        mk = 'o'; if i > nMand; mk = 'p'; end
        hWPs(i) = plot(hAx, allWaypoints(i,1), allWaypoints(i,2), ...
            mk, 'MarkerSize', 10, 'LineWidth', 2, 'Color', [0.8 0 0]);
        text(allWaypoints(i,1)+0.05, allWaypoints(i,2)+0.05, ...
            sprintf('WP%d',i), 'Parent', hAx, 'FontSize', 8);
    end
    hTitle = title(hAx,'');

    % --- weight bar panel ---
    bar(hAxBar, 1:nMand, zeros(nMand,1), 'FaceColor', [0.2 0.5 0.8]);
    hBars = findobj(hAxBar,'Type','Bar');
    hold(hAxBar,'on');

    % convergence threshold line
    hConvLine = yline(hAxBar, 0.65, 'r--', 'LineWidth', 1.5);   % 65% threshold

    xlabel(hAxBar,'Waypoint'); ylabel(hAxBar,'Normalized weight mass');
    title(hAxBar,'Particle mass per WP  (red line = convergence threshold)');
    xlim(hAxBar,[0.5 nMand+0.5]); ylim(hAxBar,[0 1]);
    xticks(hAxBar, 1:nMand);
    xticklabels(hAxBar, arrayfun(@(i)sprintf('WP%d',i), 1:nMand, 'UniformOutput', false));
end

% update particle scatter with weight-based color
if ~isempty(particles)
    pw = particles(:,4);
    pwn = pw / (max(pw)+1e-12);   % normalize 0..1 for color mapping

    % interpolate colors: blue (low) -> yellow -> red (high)
    cmap = interp1([0; 0.5; 1], [0 0 1; 1 1 0; 1 0 0], pwn);

    set(hParticles, 'XData', particles(:,1), 'YData', particles(:,2), 'CData', cmap);

    % bar chart: normalized mass at each mandatory WP
    totalW = sum(pw);
    masses = zeros(nMand,1);
    for wpi = 1:nMand
        inR = sqrt((particles(:,1)-waypoints(wpi,1)).^2 + ...
                   (particles(:,2)-waypoints(wpi,2)).^2) < 0.30;
        masses(wpi) = sum(pw(inR)) / max(totalW, 1e-12);
    end
    set(hBars,'YData',masses);
    ylim(hAxBar,[0, max(max(masses)+0.05, 0.1)]);
    % keep conv line visible after ylim change
    delete(hConvLine);
    hConvLine = yline(hAxBar, 0.65, 'r--', 'LineWidth', 1.5);
else
    set(hParticles,'XData',NaN,'YData',NaN);
end

if ~isempty(pose_est)
    L = 0.25;
    set(hBestArrow, 'XData', pose_est(1), 'YData', pose_est(2), ...
        'UData', L*cos(pose_est(3)), 'VData', L*sin(pose_est(3)));
end

if ~isempty(navPath) && navPathIdx <= size(navPath,1)
    set(hPath,'XData',navPath(navPathIdx:end,1),'YData',navPath(navPathIdx:end,2));
else
    set(hPath,'XData',NaN,'YData',NaN);
end

for i = 1:size(allWaypoints,1)
    if wpVisited(i); set(hWPs(i),'Color',[0 0.7 0]);
    else;            set(hWPs(i),'Color',[0.8 0 0]); end
end
set(hTitle,'String',sprintf('Phase: %s  |  t=%.1fs',phase,t));
drawnow limitrate;
end

% -------------------------------------------------------------------------
function plotFinalMap(map, optWalls, waypoints, ECwaypoints, beaconLoc, ...
    allWaypoints, wpVisited, wallBelief, pose_est)
nMand = size(waypoints,1);
figure('Name','Final Map','NumberTitle','off');
hold on; axis equal; grid on;
title('Final Map'); xlabel('X (m)'); ylabel('Y (m)');
for i = 1:size(map,1)
    plot([map(i,1) map(i,3)],[map(i,2) map(i,4)],'k-','LineWidth',2);
end
if ~isempty(wallBelief)
    for i = 1:size(optWalls,1)
        s = wallBelief.state(i);
        if s==1; c='r-'; elseif s==-1; c='g--'; else; c='b--'; end
        plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],c,'LineWidth',1.5);
    end
else
    for i = 1:size(optWalls,1)
        plot([optWalls(i,1) optWalls(i,3)],[optWalls(i,2) optWalls(i,4)],'b--','LineWidth',1);
    end
end
for i = 1:size(allWaypoints,1)
    isEC = i>nMand;
    if wpVisited(i); clr=[0 0.7 0]; mk='o'; else; clr=[0.8 0 0]; mk='x'; end
    sz=10; if isEC; sz=14; end
    plot(allWaypoints(i,1),allWaypoints(i,2),mk,'Color',clr,'MarkerSize',sz,'LineWidth',2);
    text(allWaypoints(i,1)+0.05,allWaypoints(i,2)+0.05, ...
        sprintf('WP%d%s',i,repmat('*',1,double(isEC))),'FontSize',8);
end
for bi = 1:size(beaconLoc,1)
    plot(beaconLoc(bi,2),beaconLoc(bi,3),'ms','MarkerSize',10,'LineWidth',2);
    text(beaconLoc(bi,2)+0.05,beaconLoc(bi,3)+0.05, ...
        sprintf('T%d',beaconLoc(bi,1)),'Color','m','FontSize',8);
end
if ~isempty(pose_est)
    quiver(pose_est(1),pose_est(2), ...
        0.2*cos(pose_est(3)),0.2*sin(pose_est(3)),'b','LineWidth',2,'MaxHeadSize',2);
    plot(pose_est(1),pose_est(2),'b.','MarkerSize',15);
end
hold off;
end