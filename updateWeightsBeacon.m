function weights = updateWeightsBeacon(particles, weights, beaconObsRaw, beaconLoc, waypoints)
% TAG-SET MATCHING: weights particles by how well their nearest waypoint's
% expected visible beacons match the observed tag set

    if isempty(beaconObsRaw)
        return;
    end

    MAX_BEACON_DIST = 2.5;   % meters that the robot can see (TODO: check during open hours)
    MATCH_BONUS = 15.0;  % log-weight boost per matched tag
    MISS_PENALTY = 3.0;   % log-weight penalty per missing expected tag
    EXTRA_PENALTY = 2.0;   % log-weight penalty per extra observed tag
    SOFT_FLOOR = 0.01;  % minimum weight for non-matching particles

    observedTags = unique(beaconObsRaw(:,1));

    % --- precompute expected tag set for every waypoint ---
    nWP = size(waypoints, 1);
    expectedTagSets = cell(nWP, 1);
    for w = 1:nWP
        wx = waypoints(w, 1);
        wy = waypoints(w, 2);
        dists = sqrt((beaconLoc(:,2) - wx).^2 + (beaconLoc(:,3) - wy).^2);
        expectedTagSets{w} = beaconLoc(dists <= MAX_BEACON_DIST, 1);
    end

    % --- score each waypoint against observed tags ---
    wpScores = zeros(nWP, 1);
    for w = 1:nWP
        expected = expectedTagSets{w};
        nMatch  = numel(intersect(observedTags, expected));
        nMiss   = numel(setdiff(expected, observedTags));
        nExtra  = numel(setdiff(observedTags, expected));
        wpScores(w) = MATCH_BONUS * nMatch ...
                    - MISS_PENALTY * nMiss ...
                    - EXTRA_PENALTY * nExtra;
    end

    fprintf('\n===== WAYPOINT TAG-SET SCORES =====\n');
    for w = 1:nWP
        fprintf('WP%d (%.2f,%.2f): score=%.1f | expected=[%s]\n', ...
            w, waypoints(w,1), waypoints(w,2), wpScores(w), ...
            num2str(expectedTagSets{w}'));
    end

    % --- assign each particle a score via nearest waypoint ---
    N = size(particles, 1);
    logWeights = log(weights + 1e-300);

    for i = 1:N
        px = particles(i,1);
        py = particles(i,2);
        dists = sqrt((waypoints(:,1) - px).^2 + (waypoints(:,2) - py).^2);
        [~, nearestWP] = min(dists);
        logWeights(i) = logWeights(i) + wpScores(nearestWP);
    end

    % --- normalize with soft floor so no particle is fully killed ---
    logWeights = logWeights - max(logWeights);
    weights = exp(logWeights);
    weights = weights / sum(weights);
    weights = max(weights, SOFT_FLOOR / N);
    weights = weights / sum(weights);

    fprintf('\n===== FINAL WAYPOINT DISTRIBUTION =====\n');
    for w = 1:nWP
        mask = false(N,1);
        for i = 1:N
            dists = sqrt((waypoints(:,1)-particles(i,1)).^2 + (waypoints(:,2)-particles(i,2)).^2);
            [~,nw] = min(dists); mask(i) = (nw == w);
        end
        fprintf('WP%d (%.2f,%.2f): weight=%.4f\n', w, waypoints(w,1), waypoints(w,2), sum(weights(mask)));
    end
end