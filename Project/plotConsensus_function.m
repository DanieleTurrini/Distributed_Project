function plotConsensus_function (numFir, numfig, numUAV, posFir1StoreReal, sigmaFir1StoreReal, Fir1Store)
    % Function to generate plots of the estimated coordinates and the extension of fire 1
    %
    % Input:
    %   numUAV - Number of UAVs
    %   posFir1StoreReal - Real positions of fire 1 (1 x 2 x time)
    %   posFir1StoreX - Estimated X positions of fire 1 (numUAV x 1 x time)
    %   posFir1StoreY - Estimated Y positions of fire 1 (numUAV x 1 x time)
    %   sigmaFir1StoreReal - Real extension of fire 1 (1 x time)
    %   sigmaFir1Stor - Estimated extensions of fire 1 (numUAV x 1 x time)

    figure(numfig);

    % Plot of the estimated X coordinate
    subplot(3,1,1);
    xlabel('Time');
    ylabel('X Coordinate');
    title(sprintf('Estimated X Coordinate of fire %d',numFir));
    hold on;
    plot(squeeze(posFir1StoreReal(1,1,:)), '--', 'DisplayName', 'Real X');
    legend;
    for k = 1:numUAV
        plot(squeeze(Fir1Store(k,1,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    

    % Plot of the estimated Y coordinate
    subplot(3,1,2);
    xlabel('Time');
    ylabel('Y Coordinate');
    title(sprintf('Estimated Y Coordinate of fire %d',numFir));
    hold on;
    plot(squeeze(posFir1StoreReal(1,2,:)), '--', 'DisplayName', 'Real Y');
    legend;
    for k = 1:numUAV
        plot(squeeze(Fir1Store(k,2,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    

    % Plot of the estimated extension
    subplot(3,1,3);
    xlabel('Time');
    ylabel('Extension');
    title(sprintf('Estimated Extension of fire %d',numFir));
    hold on;
    plot(squeeze(sigmaFir1StoreReal(1,1,:)), '--', 'DisplayName', 'Real Extension');
    legend;
    for k = 1:numUAV
        plot(squeeze(Fir1Store(k,3,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
   
end