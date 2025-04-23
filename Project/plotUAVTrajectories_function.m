function plotUAVTrajectories_function(numUAV, trajectories, trajectories_est, P_trace)
    % Funzione per generare i grafici delle traiettorie e della covarianza per ciascun UAV
    %
    % Input:
    %   numUAV - Numero di UAV
    %   trajectories - Traiettorie reali (numUAV x 4 x time)
    %   trajectories_est - Traiettorie stimate (numUAV x 4 x time)
    %   P_trace - Traccia della matrice di covarianza (numUAV x time)

    for i = 1:numUAV
        % Figura per le traiettorie
        figure(4 + i);
        
        % X-Dimension
        subplot(4,1,1);
        plot(squeeze(trajectories(i,1,:)), 'b');
        hold on;
        plot(squeeze(trajectories_est(i,1,:)), 'r--');
        hold off;
        xlabel('Time');
        ylabel('X');
        title(sprintf('X-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

        % Y-Dimension
        subplot(4,1,2);
        plot(squeeze(trajectories(i,2,:)), 'b');
        hold on;
        plot(squeeze(trajectories_est(i,2,:)), 'r--');
        hold off;
        xlabel('Time');
        ylabel('Y');
        title(sprintf('Y-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

        % Z-Dimension
        subplot(4,1,3);
        plot(squeeze(trajectories(i,3,:)), 'b');
        hold on;
        plot(squeeze(trajectories_est(i,3,:)), 'r--');
        hold off;
        xlabel('Time');
        ylabel('Z');
        title(sprintf('Z-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

        % Theta-Dimension
        subplot(4,1,4);
        plot(squeeze(trajectories(i,4,:)), 'b');
        hold on;
        plot(squeeze(trajectories_est(i,4,:)), 'r--');
        hold off;
        xlabel('Time');
        ylabel('Theta');
        title(sprintf('Theta-Dimension of UAV %d', i));
        legend('Real Trajectory', 'Estimated Trajectory');

    end
end