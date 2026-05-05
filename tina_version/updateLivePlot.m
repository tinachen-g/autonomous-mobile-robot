function updateLivePlot(particles, weights, estimatedPose, dataStore, ...
                        map, optWalls, waypoints, ECwaypoints, beaconLoc, phase)
% UPDATELIVEPLOT  Refresh the competition live-view figure.
%
%   Call once per control-loop iteration (after state updates, before
%   the next sensor read).  Uses drawnow limitrate so the repaint never
%   stalls the control loop.
%
%   INPUTS
%     particles      N×3  [x y theta] particle array
%     weights        N×1  normalised particle weights
%     estimatedPose  1×3  [x y theta] best pose estimate ([] if unknown)
%     dataStore      struct  must contain fields:
%                      .visitedWaypoints  k×2  [x y] of confirmed visits
%                      .optWallStatus     m×1  0=unresolved 1=present -1=absent
%                      .odomRelTraj       T×2  [dx dy] offsets from odom origin
%     map            n×4  known wall segments  [x1 y1 x2 y2]
%     optWalls       m×4  optional wall segments [x1 y1 x2 y2]
%     waypoints      k×2  [x y] regular waypoints
%     ECwaypoints    j×2  [x y] extra-credit waypoints
%     beaconLoc      b×3  [tagNum x y]
%     phase          char  current phase string (shown in title)
%
%   TRAJECTORY NOTE
%     Instead of storing world x,y directly, store odometry-relative offsets
%     from the robot's dead-reckoning origin.  In your odometry update block:
%
%       dataStore.odomRelTraj = [dataStore.odomRelTraj; x, y];
%
%     where x,y are the dead-reckoning accumulators (starting at 0,0).
%     This function anchors that shape to estimatedPose once localisation
%     converges, so the trajectory "slides" into place rather than jumping.
%     Before localisation the path is drawn from the odom origin in grey.
%
%   OPTIONAL WALL STATUS CONVENTION
%     dataStore.optWallStatus(i) = 0   unresolved  → drawn red dashed
%     dataStore.optWallStatus(i) = 1   confirmed present → drawn black
%     dataStore.optWallStatus(i) = -1  confirmed absent  → not drawn

% ── persistent handles (survive across calls, reset when figure closes) ──
persistent hFig hAxes ...
           hParticles hPoseArrow hTrajectory ...
           hKnownWalls hOptPresent hOptUnresolved ...
           hWaypoints hWaypointsVisited hECwaypoints hECvisited ...
           hBeacons hPhaseText hTimeText;

% ── initialise figure once ───────────────────────────────────────────────
if isempty(hFig) || ~isvalid(hFig)
    hFig = figure('Name','Robot Live View','NumberTitle','off', ...
                  'Color',[0.12 0.12 0.14], ...
                  'Position',[50 50 820 680]);
    hAxes = axes('Parent',hFig, ...
                 'Color',[0.12 0.12 0.14], ...
                 'XColor',[0.55 0.55 0.58], 'YColor',[0.55 0.55 0.58], ...
                 'GridColor',[0.25 0.25 0.28], 'GridAlpha',0.4, ...
                 'MinorGridColor',[0.2 0.2 0.22], ...
                 'FontSize',9, 'FontName','Menlo', ...
                 'TickLength',[0.005 0.005]);
    hold(hAxes,'on');
    axis(hAxes,'equal');
    grid(hAxes,'on');
    xlabel(hAxes,'x (m)','Color',[0.55 0.55 0.58]);
    ylabel(hAxes,'y (m)','Color',[0.55 0.55 0.58]);

    % ── static legend proxy patches (drawn once) ─────────────────────
    plot(hAxes, NaN, NaN, 'Color',[0.35 0.70 0.90],'LineWidth',1.5, ...
         'DisplayName','Known walls');
    plot(hAxes, NaN, NaN, 'Color',[0.20 0.80 0.40],'LineWidth',1.8, ...
         'DisplayName','Opt wall present');
    plot(hAxes, NaN, NaN, '--','Color',[0.90 0.35 0.35],'LineWidth',1.2, ...
         'DisplayName','Opt wall unresolved');
    plot(hAxes, NaN, NaN, 'o','Color',[0.45 0.75 0.98], ...
         'MarkerFaceColor',[0.45 0.75 0.98],'MarkerSize',6, ...
         'DisplayName','Waypoint');
    plot(hAxes, NaN, NaN, 'o','Color',[0.20 0.80 0.40], ...
         'MarkerFaceColor',[0.20 0.80 0.40],'MarkerSize',7, ...
         'DisplayName','Visited');
    plot(hAxes, NaN, NaN, 'p','Color',[1.0 0.75 0.20], ...
         'MarkerFaceColor',[1.0 0.75 0.20],'MarkerSize',10, ...
         'DisplayName','EC waypoint');
    plot(hAxes, NaN, NaN, 's','Color',[0.90 0.55 0.90], ...
         'MarkerFaceColor',[0.90 0.55 0.90],'MarkerSize',7, ...
         'DisplayName','Beacon');
    plot(hAxes, NaN, NaN, '-','Color',[0.85 0.85 0.88],'LineWidth',1.0, ...
         'DisplayName','Trajectory');
    scatter(hAxes, NaN, NaN, 6, [0.6 0.6 0.65], 'filled', ...
            'DisplayName','Particles');

    hLeg = legend(hAxes,'show','Location','eastoutside');
    set(hLeg,'Color',[0.17 0.17 0.19],'TextColor',[0.82 0.82 0.85], ...
             'EdgeColor',[0.30 0.30 0.33],'FontSize',8,'FontName','Menlo');

    % ── draw known walls as one line object (NaN separators between segments)
    % This produces a single legend entry instead of one per segment.
    nanSep  = NaN(size(map,1), 1);
    wallX   = [map(:,1), map(:,3), nanSep]';   % 3×n, each col is [x1;x2;NaN]
    wallY   = [map(:,2), map(:,4), nanSep]';
    plot(hAxes, wallX(:), wallY(:), ...
         'Color',[0.35 0.70 0.90],'LineWidth',1.5,'HandleVisibility','off');

    % ── beacon markers (static) ───────────────────────────────────────
    if ~isempty(beaconLoc)
        hBeacons = plot(hAxes, beaconLoc(:,2), beaconLoc(:,3), 's', ...
                        'Color',[0.90 0.55 0.90], ...
                        'MarkerFaceColor',[0.90 0.55 0.90], ...
                        'MarkerSize',8,'LineWidth',0.5,'HandleVisibility','off');
        for b = 1:size(beaconLoc,1)
            text(hAxes, beaconLoc(b,2)+0.05, beaconLoc(b,3)+0.05, ...
                 sprintf('B%d',beaconLoc(b,1)), ...
                 'Color',[0.90 0.55 0.90],'FontSize',7,'FontName','Menlo');
        end
    end

    % ── all waypoint labels (static) ─────────────────────────────────
    for w = 1:size(waypoints,1)
        text(hAxes, waypoints(w,1)+0.05, waypoints(w,2)+0.05, ...
             sprintf('W%d',w), ...
             'Color',[0.55 0.75 0.95],'FontSize',7,'FontName','Menlo');
    end
    if ~isempty(ECwaypoints)
        for w = 1:size(ECwaypoints,1)
            text(hAxes, ECwaypoints(w,1)+0.05, ECwaypoints(w,2)+0.05, ...
                 sprintf('EC%d',w), ...
                 'Color',[1.0 0.75 0.20],'FontSize',7,'FontName','Menlo');
        end
    end

    % ── phase / time text ─────────────────────────────────────────────
    hPhaseText = text(hAxes, 0,0,'', 'Units','normalized', ...
                      'Position',[0.02 0.97], ...
                      'Color',[0.85 0.85 0.88],'FontSize',10, ...
                      'FontName','Menlo','FontWeight','bold', ...
                      'VerticalAlignment','top');
    hTimeText  = text(hAxes, 0,0,'', 'Units','normalized', ...
                      'Position',[0.02 0.91], ...
                      'Color',[0.55 0.55 0.60],'FontSize',8, ...
                      'FontName','Menlo','VerticalAlignment','top');

    % ── pre-allocate dynamic graphics objects ─────────────────────────
    hParticles        = scatter(hAxes, NaN, NaN, 6, [0.6 0.6 0.65], 'filled', ...
                                'HandleVisibility','off');
    hPoseArrow        = quiver(hAxes, NaN, NaN, NaN, NaN, 0, ...
                               'Color',[1.0 0.40 0.30],'LineWidth',2.0, ...
                               'MaxHeadSize',0.6,'HandleVisibility','off');
    hTrajectory       = plot(hAxes, NaN, NaN, '-', ...
                             'Color',[0.85 0.85 0.88],'LineWidth',1.0, ...
                             'HandleVisibility','off');
    hKnownWalls       = [];   % already drawn above
    hOptPresent       = gobjects(0);
    hOptUnresolved    = gobjects(0);
    hWaypoints        = plot(hAxes, waypoints(:,1), waypoints(:,2), 'o', ...
                             'Color',[0.45 0.75 0.98], ...
                             'MarkerFaceColor',[0.45 0.75 0.98], ...
                             'MarkerSize',7,'LineWidth',0.5, ...
                             'HandleVisibility','off');
    hWaypointsVisited = plot(hAxes, NaN, NaN, 'o', ...
                             'Color',[0.20 0.80 0.40], ...
                             'MarkerFaceColor',[0.20 0.80 0.40], ...
                             'MarkerSize',9,'LineWidth',1.0, ...
                             'HandleVisibility','off');
    if ~isempty(ECwaypoints)
        hECwaypoints  = plot(hAxes, ECwaypoints(:,1), ECwaypoints(:,2), 'p', ...
                             'Color',[1.0 0.75 0.20], ...
                             'MarkerFaceColor',[1.0 0.75 0.20], ...
                             'MarkerSize',11,'LineWidth',0.5, ...
                             'HandleVisibility','off');
        hECvisited    = plot(hAxes, NaN, NaN, 'p', ...
                             'Color',[0.20 0.80 0.40], ...
                             'MarkerFaceColor',[0.20 0.80 0.40], ...
                             'MarkerSize',12,'LineWidth',1.0, ...
                             'HandleVisibility','off');
    else
        hECwaypoints = [];
        hECvisited   = [];
    end

    % draw map bounding box for axis limits
    allX = [map(:,1); map(:,3)];
    allY = [map(:,2); map(:,4)];
    pad  = 0.5;
    axis(hAxes, [min(allX)-pad max(allX)+pad min(allY)-pad max(allY)+pad]);
end

% ── PARTICLES ────────────────────────────────────────────────────────────
% Color encodes confidence: dim gray (low weight) → bright white/yellow
% (high weight).  Avoids the blue-cluster artifact that occurs when the
% old R/G/B formula drives R and G to zero as weights converge.
wNorm  = weights / max(weights + 1e-12);   % 0..1, 1 = highest-weight particle
szVec  = 4 + 20 * wNorm;                  % 4–24 pt²
% dim gray at wNorm=0, warm bright white at wNorm=1
cMat   = [0.35 + 0.60*wNorm, ...   % R: 0.35 → 0.95
          0.35 + 0.55*wNorm, ...   % G: 0.35 → 0.90
          0.35 + 0.40*wNorm];      % B: 0.35 → 0.75  (warm, not blue)
set(hParticles, 'XData', particles(:,1), 'YData', particles(:,2), ...
                'SizeData', szVec, 'CData', cMat);

% ── TRAJECTORY ───────────────────────────────────────────────────────────
% The trajectory is stored as dead-reckoning offsets from the odom origin
% (dataStore.odomRelTraj, starting at [0 0]).  Once estimatedPose is
% available the whole path is rigidly transformed so it anchors to the
% localised pose rather than jumping when the first hard pose reset fires.
%
% Transform:  worldPt = R(th_est) * odomPt' + [x_est; y_est]
% where th_est is taken from the first moment estimatedPose was set
% (stored in dataStore.localisedAtOdom — see patch notes).
if isfield(dataStore,'odomRelTraj') && size(dataStore.odomRelTraj,1) > 1
    rel = dataStore.odomRelTraj;   % T×2 odometry-relative path
    if ~isempty(estimatedPose)
        % Use the odom pose at the moment localisation fired so the
        % rotation undoes exactly the dead-reckoning drift up to that point.
        if isfield(dataStore,'localisedAtOdom') && ~isempty(dataStore.localisedAtOdom)
            odomAnchor = dataStore.localisedAtOdom;   % [x0 y0 th0] at lock time
        else
            odomAnchor = [0 0 0];
        end
        % Shift path so the lock-point is the origin, then rotate+translate
        shifted = rel - odomAnchor(1:2);              % T×2
        th = estimatedPose(3) - odomAnchor(3);        % net rotation correction
        R  = [cos(th) -sin(th); sin(th) cos(th)];
        world = (R * shifted')';                      % T×2
        world(:,1) = world(:,1) + estimatedPose(1);
        world(:,2) = world(:,2) + estimatedPose(2);
        set(hTrajectory, 'XData', world(:,1), 'YData', world(:,2), ...
                         'Color',[0.85 0.85 0.88]);
    else
        % Pre-localisation: draw raw odom path dimmed so it's clear it's
        % unanchored.  It will snap into place once estimatedPose arrives.
        set(hTrajectory, 'XData', rel(:,1), 'YData', rel(:,2), ...
                         'Color',[0.45 0.45 0.48]);
    end
end

% ── ESTIMATED POSE ARROW ─────────────────────────────────────────────────
if ~isempty(estimatedPose)
    arrowLen = 0.18;
    set(hPoseArrow, ...
        'XData', estimatedPose(1), 'YData', estimatedPose(2), ...
        'UData', arrowLen*cos(estimatedPose(3)), ...
        'VData', arrowLen*sin(estimatedPose(3)));
end

% ── OPTIONAL WALLS ───────────────────────────────────────────────────────
% Delete and redraw only when wall status array has entries
% (status changes are rare so this is cheap)
if ~isempty(optWalls)
    % Ensure status vector exists and is the right length
    if ~isfield(dataStore,'optWallStatus') || ...
            length(dataStore.optWallStatus) ~= size(optWalls,1)
        dataStore.optWallStatus = zeros(size(optWalls,1),1);
    end
    status = dataStore.optWallStatus;

    % delete old optional-wall lines
    delete(hOptPresent(isvalid(hOptPresent)));
    delete(hOptUnresolved(isvalid(hOptUnresolved)));
    hOptPresent    = gobjects(0);
    hOptUnresolved = gobjects(0);

    for i = 1:size(optWalls,1)
        x1 = optWalls(i,1); y1 = optWalls(i,2);
        x2 = optWalls(i,3); y2 = optWalls(i,4);
        if status(i) == 1          % confirmed present → solid green
            hOptPresent(end+1) = plot(hAxes, [x1 x2], [y1 y2], ...
                'Color',[0.20 0.80 0.40],'LineWidth',2.2, ...
                'HandleVisibility','off');
        elseif status(i) == 0      % unresolved → dashed red
            hOptUnresolved(end+1) = plot(hAxes, [x1 x2], [y1 y2], '--', ...
                'Color',[0.90 0.35 0.35],'LineWidth',1.2, ...
                'HandleVisibility','off');
        end
        % status == -1 (absent): draw nothing
    end
end

% ── VISITED WAYPOINTS ────────────────────────────────────────────────────
if isfield(dataStore,'visitedWaypoints') && ~isempty(dataStore.visitedWaypoints)
    set(hWaypointsVisited, ...
        'XData', dataStore.visitedWaypoints(:,1), ...
        'YData', dataStore.visitedWaypoints(:,2));
end
if isfield(dataStore,'visitedECWaypoints') && ~isempty(dataStore.visitedECWaypoints) ...
        && ~isempty(hECvisited)
    set(hECvisited, ...
        'XData', dataStore.visitedECWaypoints(:,1), ...
        'YData', dataStore.visitedECWaypoints(:,2));
end

% ── PHASE / TIME TEXT ────────────────────────────────────────────────────
nVisited = 0;
if isfield(dataStore,'visitedWaypoints')
    nVisited = size(dataStore.visitedWaypoints,1);
end
nEC = 0;
if isfield(dataStore,'visitedECWaypoints')
    nEC = size(dataStore.visitedECWaypoints,1);
end

set(hPhaseText, 'String', sprintf('[ %s ]', phase));
set(hTimeText,  'String', sprintf('visited %d/%d WP  |  %d EC  |  t=%.1fs', ...
    nVisited, size(waypoints,1), nEC, toc));

% ── FLUSH ────────────────────────────────────────────────────────────────
drawnow limitrate;

end