function particles = seedParticles(waypoints, N)
% seed N particles clustered at each waypoint.
% each cluster gets floor(N / k) particles; remainder goes to first cluster.
    k = size(waypoints, 1);
    perWaypoint = floor(N / k);
    particles  = zeros(N, 3);
    idx = 1;
    for i = 1:k
        nThis = perWaypoint + (i == 1) * (N - perWaypoint * k);
        orients = linspace(-pi, pi, nThis)';
        % small position noise around each waypoint (std 0.05 m)
        noise   = 0.05 * randn(nThis, 2);
        particles(idx:idx+nThis-1, :) = [waypoints(i,1) + noise(:,1), ...
                                          waypoints(i,2) + noise(:,2), ...
                                          orients];
        idx = idx + nThis;
    end
end
 