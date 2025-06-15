% plotConsensus_function Visualizes the consensus estimation of fire parameters by multiple UAVs.
%   This function generates a figure with three subplots showing the estimated X coordinate,
%   Y coordinate, and extension (sigma) of a specified fire over time. The real values and
%   the estimates from each UAV are plotted for comparison, allowing assessment of the consensus
%   and estimation accuracy among the UAVs.

function plotConsensus_function (numFir, numfig, numUAV, posFirStoreReal, sigmaFirStoreReal, FirStore)

    figure(numfig);

    % X Estimate

    subplot(3,1,1);
    xlabel('Time');
    ylabel('X Coordinate');
    title(sprintf('Estimated X Coordinate of fire %d', numFir));
    hold on;
    plot(squeeze(posFirStoreReal(1,1,:)), '--', 'DisplayName', 'Real X');
    legend;
    for k = 1:numUAV
        plot(squeeze(FirStore(k,1,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    

    % Y Estimate

    subplot(3,1,2);
    xlabel('Time');
    ylabel('Y Coordinate');
    title(sprintf('Estimated Y Coordinate of fire %d',numFir));
    hold on;
    plot(squeeze(posFirStoreReal(1,2,:)), '--', 'DisplayName', 'Real Y');
    legend;
    for k = 1:numUAV
        plot(squeeze(FirStore(k,2,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    

    % Sigma Estimate

    subplot(3,1,3);
    xlabel('Time');
    ylabel('Extension');
    title(sprintf('Estimated Extension of fire %d',numFir));
    hold on;
    plot(sigmaFirStoreReal(1,:), '--', 'DisplayName', 'Real Extension');
    legend;
    for k = 1:numUAV
        plot(squeeze(FirStore(k,3,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
   
end