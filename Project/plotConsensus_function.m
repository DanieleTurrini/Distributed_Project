function plotConsensus_function (numUAV, posFir1StoreReal, posFir1StoreX, posFir1StoreY, sigmaFir1StoreReal, sigmaFir1Stor)
    % Funzione per generare i grafici delle coordinate stimate e dell'estensione del fuoco 1
    %
    % Input:
    %   numUAV - Numero di UAV
    %   posFir1StoreReal - Posizioni reali del fuoco 1 (1 x 2 x time)
    %   posFir1StoreX - Posizioni stimate X del fuoco 1 (numUAV x 1 x time)
    %   posFir1StoreY - Posizioni stimate Y del fuoco 1 (numUAV x 1 x time)
    %   sigmaFir1StoreReal - Estensione reale del fuoco 1 (1 x time)
    %   sigmaFir1Stor - Estensioni stimate del fuoco 1 (numUAV x 1 x time)

    figure(20);

    % Grafico della coordinata X stimata
    subplot(3,1,1);
    xlabel('Time');
    ylabel('X Coordinate');
    title('Estimated X Coordinate of fire 1');
    hold on;
    plot(squeeze(posFir1StoreReal(1,1,:)), '--', 'DisplayName', 'Real X');
    for k = 1:numUAV
        plot(squeeze(posFir1StoreX(k,1,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    legend;

    % Grafico della coordinata Y stimata
    subplot(3,1,2);
    xlabel('Time');
    ylabel('Y Coordinate');
    title('Estimated Y Coordinate of fire 1');
    hold on;
    plot(squeeze(posFir1StoreReal(1,2,:)), '--', 'DisplayName', 'Real Y');
    for k = 1:numUAV
        plot(squeeze(posFir1StoreY(k,1,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    legend;

    % Grafico dell'estensione stimata
    subplot(3,1,3);
    xlabel('Time');
    ylabel('Extension');
    title('Estimated Extension of fire 1');
    hold on;
    plot(squeeze(sigmaFir1StoreReal(1,1,:)), '--', 'DisplayName', 'Real Extension');
    for k = 1:numUAV
        plot(squeeze(sigmaFir1Stor(k,1,:)), 'DisplayName', sprintf('UAV %d', k));
    end
    hold off;
    legend;
end