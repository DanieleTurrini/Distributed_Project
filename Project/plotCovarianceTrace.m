function plotCovarianceTrace(numUAV,P_trace)


    for i = 1:numUAV
        figure;
        % Plot the trace of  the covariance matrix for the current UAV over time.
        plot(squeeze(P_trace(i,:)));

        xlabel('Time');
        ylabel('Covariance');
        title(sprintf('Trace of Covariance matrix of UAV %d', i));
    end
end
