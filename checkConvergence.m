function [converged, bestPose, confidence] = checkConvergence(particles, weights, waypoints)

    CONVERGE_THRESH = 0.75;  % single waypoint must own this fraction of weight

    nWP = size(waypoints, 1);
    N   = size(particles, 1);
    wpWeights = zeros(nWP, 1);

    for i = 1:N
        dists = sqrt((waypoints(:,1)-particles(i,1)).^2 + (waypoints(:,2)-particles(i,2)).^2);
        [~, nearest] = min(dists);
        wpWeights(nearest) = wpWeights(nearest) + weights(i);
    end

    [maxW, bestWP] = max(wpWeights);
    confidence = maxW;

    fprintf('Waypoint weights: ');
    for w = 1:nWP
        fprintf('WP%d=%.3f  ', w, wpWeights(w));
    end
    fprintf('\n');

    if maxW >= CONVERGE_THRESH
        % best pose = weighted mean of particles near that waypoint
        mask = false(N,1);
        for i = 1:N
            dists = sqrt((waypoints(:,1)-particles(i,1)).^2 + (waypoints(:,2)-particles(i,2)).^2);
            [~,nw] = min(dists);
            mask(i) = (nw == bestWP);
        end
        w_sub = weights(mask) / sum(weights(mask));
        bestPose = sum(particles(mask,:) .* w_sub, 1);
        converged = true;
    else
        bestPose = [];
        converged = false;
    end
end