% plotCovarianceTrace plots the trace of the covariance matrix over time for each UAV.
% 
%   plotCovarianceTrace(numUAV, P_trace) creates a separate figure for each UAV,
%   displaying how the trace of its covariance matrix evolves over time.

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
