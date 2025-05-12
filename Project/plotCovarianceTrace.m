function plotCovarianceTrace(numUAV,P_trace)

    for i = 1:numUAV

        % Figura per la traccia della matrice di covarianza
        % figure(6 + numUAV);
       
        % subplot(numUAV,1,i);
        figure(6 + i);
        plot(squeeze(P_trace(i,:)));
        xlabel('Time');
        ylabel('Covariance');
        title(sprintf('Trace of Covariance matrix of UAV %d', i));

    end
end
