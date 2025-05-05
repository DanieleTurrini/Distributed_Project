function plotUAVTrajectories_function(numUAV, trajectories, trajectories_est, Dimgrid)
    % Function to generate trajectory and covariance plots for each UAV
    %
    % Input:
    %   numUAV - Number of UAVs
    %   trajectories - Real trajectories (numUAV x 4 x time)
    %   trajectories_est - Estimated trajectories (numUAV x 4 x time)
    %   Dimgrid - Dimensions of the grid [X, Y]

    for i = 1:numUAV
        % Figure for trajectories
        figure(4 + i);
        
        % X-Dimension
        subplot(4,1,1);
        plot(squeeze(trajectories(i,1,:)), 'b'); % Plot real trajectory in blue
        hold on;
        plot(squeeze(trajectories_est(i,1,:)), 'r--'); % Plot estimated trajectory in red dashed line
        hold off;
        xlabel('Time');
        ylabel('X');
        title(sprintf('X-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

        % Y-Dimension
        subplot(4,1,2);
        plot(squeeze(trajectories(i,2,:)), 'b'); % Plot real trajectory in blue
        hold on;
        plot(squeeze(trajectories_est(i,2,:)), 'r--'); % Plot estimated trajectory in red dashed line
        hold off;
        xlabel('Time');
        ylabel('Y');
        title(sprintf('Y-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

        % Z-Dimension
        subplot(4,1,3);
        plot(squeeze(trajectories(i,3,:)), 'b'); % Plot real trajectory in blue
        hold on;
        plot(squeeze(trajectories_est(i,3,:)), 'r--'); % Plot estimated trajectory in red dashed line
        hold off;
        xlabel('Time');
        ylabel('Z');
        title(sprintf('Z-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

        % Theta-Dimension
        subplot(4,1,4);
        plot(squeeze(trajectories(i,4,:)), 'b'); % Plot real trajectory in blue
        hold on;
        plot(squeeze(trajectories_est(i,4,:)), 'r--'); % Plot estimated trajectory in red dashed line
        hold off;
        xlabel('Time');
        ylabel('Theta');
        title(sprintf('Theta-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

    end

    % Figure for trajectories in the x-y plane
    figure;
    hold on;
    colors = lines(numUAV); % Generate a set of distinct colors
    
    plot(-1,-1, 'k', 'LineWidth', 0.5); % Plot real trajectory
    plot(-1,-1, '--k', 'LineWidth', 0.5); % Plot estimated trajectory

    for i = 1:numUAV
        plot(squeeze(trajectories(i,1,:)), squeeze(trajectories(i,2,:)), 'Color', colors(i,:), 'LineWidth', 1.5); % Plot real trajectory
        plot(squeeze(trajectories_est(i,1,:)), squeeze(trajectories_est(i,2,:)), '--', 'Color', colors(i,:), 'LineWidth', 1.5); % Plot estimated trajectory
    end
    hold off;
    xlabel('X');
    ylabel('Y');
    title('Trajectories in the X-Y Plane');
    legend('Real Trajectory', 'Estimated Trajectory');
    xlim([0 Dimgrid(1)]); % Set x-axis limits based on grid dimensions
    ylim([0 Dimgrid(2)]); % Set y-axis limits based on grid dimensions
    grid on; % Enable grid
end