function visualizePF(particles, weights, beaconObs, beaconLoc)
% VISUALIZEPF
%   Visualize particle filter state: particles colored by weight,
%   beacon locations, and observed bearing rays from best particle.
%
%   Call BEFORE and AFTER updateWeightsBeacon to compare.

    figure(); clf; hold on; axis equal;
    title('Particle Filter Visualization');

    % --- Normalize weights for color scaling ---
    w_plot = weights / (max(weights) + 1e-300);

    % --- Plot particles colored by weight ---
    scatter(particles(:,1), particles(:,2), 20, w_plot, 'filled');
    colormap(jet);
    clim([0 1]);
    colorbar;

    % --- Best particle ---
    [~, idx] = max(weights);
    best = particles(idx, :);
    plot(best(1), best(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    quiver(best(1), best(2), cos(best(3))*0.4, sin(best(3))*0.4, ...
           0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    % --- Beacon world locations ---
    for i = 1:size(beaconLoc, 1)
        bx = beaconLoc(i, 2);
        by = beaconLoc(i, 3);
        plot(bx, by, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
        text(bx + 0.1, by + 0.1, sprintf('ID %d', beaconLoc(i,1)), ...
             'FontSize', 9, 'Color', 'r');
    end


    % % --- Weight histogram inset (helps see distribution shape) ---
    % ax2 = axes('Position', [0.72 0.12 0.18 0.18]);
    % histogram(ax2, weights, 20, 'FaceColor', [0.2 0.6 0.2]);
    % title(ax2, 'Weight dist.', 'FontSize', 8);
    % xlabel(ax2, 'w', 'FontSize', 7);

    axes(gca);  % return focus to main axes
    xlabel('X'); ylabel('Y');
    grid on;
    drawnow;
end