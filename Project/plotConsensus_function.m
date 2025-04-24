function plotConsensus_function (numFir, numfig, numUAV, posFir1StoreReal, sigmaFir1StoreReal, Fir1Store)
    % Funzione per generare i grafici delle coordinate stimate e dell'estensione del fuoco 1
    %
    % Input:
    %   numUAV - Numero di UAV
    %   posFir1StoreReal - Posizioni reali del fuoco 1 (1 x 2 x time)
    %   posFir1StoreX - Posizioni stimate X del fuoco 1 (numUAV x 1 x time)
    %   posFir1StoreY - Posizioni stimate Y del fuoco 1 (numUAV x 1 x time)
    %   sigmaFir1StoreReal - Estensione reale del fuoco 1 (1 x time)
    %   sigmaFir1Stor - Estensioni stimate del fuoco 1 (numUAV x 1 x time)

    figure(numfig);

    % Grafico della coordinata X stimata
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
    

    % Grafico della coordinata Y stimata
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
    

    % Grafico dell'estensione stimata
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