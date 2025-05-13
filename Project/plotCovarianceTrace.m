function plotCovarianceTrace(numUAV,P_trace)

    for i = 1:numUAV
        
        figure(6 + numUAV);

        subplot(numUAV,1,i);
        plot(squeeze(P_trace(i,:)));
        xlabel('Time');
        ylabel('Covariance');
        title(sprintf('Trace of Covariance matrix of UAV %d', i));

    end
end
