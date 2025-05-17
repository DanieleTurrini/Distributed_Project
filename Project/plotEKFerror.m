function plotEKFerror(numUAV, est_error)

    for i = 1:numUAV
        % Create a new figure for each UAV
        figure(100+i);
        title('Estimation Error');

        % Plot X and Y errors
        subplot(3,1,1);
        xlabel('Time'); % Label for x-axis
        ylabel('X-Y Error'); % Label for y-axis
        title(sprintf('X and Y errors of UAV %d', i)); % Title for the subplot

        hold on;
        % Plot X error in red
        plot(squeeze(est_error(i,1,:)),'r');
        % Plot Y error in blue
        plot(squeeze(est_error(i,2,:)),'b');

        legend('Error in X', 'Error in Y'); % Add legend
        hold off;

        % Plot Z error
        subplot(3,1,2);
        title(sprintf('Z error of UAV %d', i)); % Title for the subplot
        xlabel('Time'); % Label for x-axis
        ylabel('Z Error'); % Label for y-axis
        hold on;
        % Plot Z error in blue
        plot(squeeze(est_error(i,3,:)),'b');

        legend('Error in Z'); % Add legend
        hold off;

        % Plot Theta error
        subplot(3,1,3);
        title(sprintf('Theta error of UAV %d', i)); % Title for the subplot
        xlabel('Time'); % Label for x-axis
        ylabel('Theta Error'); % Label for y-axis
        hold on; 
        % Plot Theta error in blue
        plot(squeeze(est_error(i,4,:)),'b');

        legend('Error in Theta'); % Add legend
        hold off;
    end
end