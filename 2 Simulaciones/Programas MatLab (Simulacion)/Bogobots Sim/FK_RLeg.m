function T06 = FK_RLeg(L,q)
    % 
    % Regresa la matriz de transformacion homogenea del sistem coordenado
    % inercial hasta el sistema coordenado del pie derecho del robot humanoide

    % parametros del robot
    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);
    % valores articulares
    q7 = q(7); q9 = q(9); q11 = q(11); q13 = q(13); q15 = q(15); q17 = q(17);

    % FK pierna derecha
    T01 = [rotz(-q7),[0;0;0]; 0 0 0 1]; % cadera
    T12 = [rotx(-q9),[0;0;0]; 0 0 0 1]; % cadera
    T23 = [roty(-q11),[0;0;0]; 0 0 0 1]; % cadera
    T34 = [roty(-q13),[0;0;-L3]; 0 0 0 1]; %rodilla
    T45 = [roty(q15),[0;0;-L4]; 0 0 0 1]; %tobillo
    T56 = [rotx(q17),[0;0;0]; 0 0 0 1]; %tobillo
    T06 = T01*T12*T23*T34*T45*T56;
end