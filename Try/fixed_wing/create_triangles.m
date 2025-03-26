function rotated_vertices = create_triangles(stato)
    
    % Parametri per il triangolo (in coordinate locali)
    triangle_length = 8;   % lunghezza (direzione)
    triangle_width  = 8;   % larghezza (laterale)

    % Coordinate locali dei vertici del triangolo:
    % Il triangolo punta verso destra lungo l'asse x
    vertices = [0,          triangle_width/2;   % vertice superiore sinistro
                0,         -triangle_width/2;   % vertice inferiore sinistro
                triangle_length,  0];           % punta (destro)

    % Estrai posizione e orientamento del drone
    x = stato(1);
    y = stato(2);
    theta = stato(3);

    % Costruisci la matrice di rotazione
    R = [cos(theta) -sin(theta); 
         sin(theta)  cos(theta)];

    % Ruota e trasla i vertici
    rotated_vertices = (R * vertices')';  % ruota: trasforma ogni riga
    rotated_vertices(:,1) = rotated_vertices(:,1) + x; % trasla in x
    rotated_vertices(:,2) = rotated_vertices(:,2) + y; % trasla in y

    % Chiudere il triangolo ripetendo il primo punto
    rotated_vertices = [rotated_vertices; rotated_vertices(1,:)];