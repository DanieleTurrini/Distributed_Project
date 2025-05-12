function plotEKFerror(numUAV, est_error)

    for i = 1:numUAV

        figure(100+i);
        title('Estimation Error');

        subplot(3,1,1);
        xlabel('Time');
        ylabel('X-Y Error');
        title(sprintf('X and Y errors of UAV %d', i));

        hold on;
        plot(squeeze(est_error(i,1,:)),'r');
        plot(squeeze(est_error(i,2,:)),'b');

        legend('Error in X', 'Error in Y');
        hold off;

        
 
        subplot(3,1,2);
        title(sprintf('Z error of UAV %d', i));
        xlabel('Time');
        ylabel('Z Error');
        hold on;
        plot(squeeze(est_error(i,3,:)),'b');

        legend('Error in Z');
        hold off;

        subplot(3,1,3);
        title(sprintf('Theta error of UAV %d', i));
        xlabel('Time');
        ylabel('Theta Error');
        hold on; 
        plot(squeeze(est_error(i,4,:)),'b');

        legend('Error in Theta');
        hold off;

    end