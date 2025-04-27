function T012 = FK_LLeg(L,q)
    % 
    % Regresa la matriz de transformacion homogenea del sistem coordenado
    % inercial hasta el sistema coordenado del pie izquierdo del robot humanoide

    % parametros del robot
    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);
    
    % valores articulares
    q8 = q(8); q10 = q(10); q12 = q(12); q14 = q(14); q16 = q(16); q18 = q(18);

    % FK pierna izquierda
    T07 = [rotz(-q8),[0;0;0]; 0 0 0 1]; % cadera
    T78 = [rotx(-q10),[0;0;0]; 0 0 0 1]; % cadera
    T89 = [roty(q12),[0;0;0]; 0 0 0 1]; % cadera
    T910 = [roty(q14),[0;0;-L3]; 0 0 0 1]; % rodilla
    T1011 = [roty(-q16),[0;0;-L4]; 0 0 0 1]; % tobillo 
    T1112 = [rotx(q18),[0;0;0]; 0 0 0 1]; % tobillo 
    T012 = T07*T78*T89*T910*T1011*T1112;
end