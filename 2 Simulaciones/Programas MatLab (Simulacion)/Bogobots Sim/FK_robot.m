% Estas funciones obtienen la cinematica directa de todo el Bogobot y de
% cada una de las cadenas. Se recomienda no cambiar la cinemática de todo
% el robot y en su lugar modificar unicamente la cadena que se requiera.

%   Estructura del vector de las dimensiones del robot.
%       L = [leg1 leg2 leg3 leg4 arm1 arm2 arm3 arm4]

%   Escructura de los valores articulares que recibe la funcion de
%   cinematica directa.
%   Q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16 q17 q18]
% -------------------------------------------------------------------------
% Cinematica directa de todo el robot -------------------------------------
% -------------------------------------------------------------------------
function [Points, T0F] = FK_robot(L,q)
% 
% Regresa los puntos de todas las extremidades del robot con base en la
% cinemática directa
%
%   Points [[1],[P],[x;y;z]] = arreglo con todos los puntos donde esta el
%   robot organizado de la forma.
%       Points(:,:,1) = todos los valores de los puntos en X
%       Points(:,:,2) = todos los valores de los puntos en Y
%       Points(:,:,3) = todos los valores de los puntos en Z
%   T0F = [T0D TFD T0I TFI B0D BFD B0I BFI];

    % parametros del robot
    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);
    D1 = L(5); D2 = L(6); D3 = L(7); D4 = L(8);
    % valores articulares
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
    q7 = q(7); q9 = q(9); q11 = q(11); q13 = q(13); q15 = q(15); q17 = q(17);
    q8 = q(8); q10 = q(10); q12 = q(12); q14 = q(14); q16 = q(16); q18 = q(18);
    
    % FK PIERNAS ----------------------------------------------------
    % ---------------------------------------------------------------

    % FK robot pierna derecha ---------------------------------------
    T01 = [rotz(-q7),[0;0;0]; 0 0 0 1]; % cadera
    T12 = [rotx(-q9),[0;0;0]; 0 0 0 1]; % cadera
    T23 = [roty(-q11),[0;0;0]; 0 0 0 1]; % cadera
    T34 = [roty(-q13),[0;0;-L3]; 0 0 0 1]; %rodilla
    T45 = [roty(q15),[0;0;-L4]; 0 0 0 1]; %tobillo
    T56 = [rotx(q17),[0;0;0]; 0 0 0 1]; %tobillo

    T03 = T01*T12*T23;
    T04 = T03*T34;
    T06 = T04*T45*T56;
    TpieD = T06*[eye(3), [5;0;0]; 0 0 0 1];
    
    PD = [0,-L1,L3+L4]; % para acomodar la pata derecha en dibujo
    
    % FK robot pierna izquierda ------------------------------------
    T07 = [rotz(-q8),[0;0;0]; 0 0 0 1]; % cadera
    T78 = [rotx(-q10),[0;0;0]; 0 0 0 1]; % cadera
    T89 = [roty(q12),[0;0;0]; 0 0 0 1]; % cadera
    T910 = [roty(q14),[0;0;-L3]; 0 0 0 1]; % rodilla
    T1011 = [roty(-q16),[0;0;-L4]; 0 0 0 1]; % tobillo 
    T1112 = [rotx(q18),[0;0;0]; 0 0 0 1]; % tobillo 

    T09 = T07*T78*T89;
    T010 = T09*T910;
    T012 = T010*T1011*T1112;
    TpieI = T012*[eye(3), [5;0;0]; 0 0 0 1];

    PI = [0,L1,L3+L4]; % para acomodar pata izquierda en dibujo
    
    % FK BRAZOS -----------------------------------------------------
    % ---------------------------------------------------------------
    
    % FK robot brazo derecho ----------------------------------------
    B01 = [roty(-q1),[0;0;0]; 0 0 0 1]; % hombro
    B12 = [rotx(-q3),[0;0;0]; 0 0 0 1]; % hombro
    B23 = [rotx(q5), [0;0;-D3]; 0 0 0 1]; % codo
    B34 = [eye(3), [0;0;-D4]; 0 0 0 1]; % mano

    B02 = B01*B12;
    B03 = B02*B23;
    B04 = B03*B34;

    BD = [0,(-L1-D2),(L3+L4)+(D1)]; % para acomodar brazo derecho en dibujo

    % FK robot brazo izquierdo --------------------------------------
    B05 = [roty(q2),[0;0;0]; 0 0 0 1]; % hombro
    B56 = [rotx(-q4),[0;0;0];0 0 0 1]; % hombro
    B67 = [rotx(q6), [0;0;-D3]; 0 0 0 1]; % codo
    B78 = [eye(3), [0;0;-D4]; 0 0 0 1]; % mano

    B06 = B05*B56;
    B07 = B06*B67;
    B08 = B07*B78;
    
    BI = [0,(L1+D2),(L3+L4+D1)]; % para acomodar brazo izquierdo en dibujo

    % PUNTOS NECESARIOS PARA GENERAR EL DIBUJO ----------------------
    % ---------------------------------------------------------------
    
    % pierna derecha
    P1 = T06(1:3,4)' + PD; % punto tobillo
    P2 = TpieD(1:3,4)' + PD; % punto punta pie
    P3 = T04(1:3,4)' + PD; % punto rodilla
    P4 = T03(1:3,4)' + PD; % punto cadera

    % pierna izquierda
    P5 = T09(1:3,4)' + PI; % punto cadera
    P6 = T010(1:3,4)' + PI; % punto rodilla
    P7 = T012(1:3,4)' + PI; % punto tobillo
    P8 = TpieI(1:3,4)' + PI; % punto punta pie

    % brazo derecho
    P9 = B04(1:3,4)' + BD; % punto mano
    P10 = B03(1:3,4)' + BD; % punto codo
    P11 = B02(1:3,4)' + BD; % punto hombro

    % brazo izquierdo
    P12 = B06(1:3,4)' + BI; % punto hombro
    P13 = B07(1:3,4)' + BI; % punto codo
    P14 = B08(1:3,4)' + BI; % punto mano

    % renglon, elemento, profundidad
    Points(:,:,1) = [P2(1), P1(1), P3(1), P4(1), P5(1), P6(1), P7(1), P8(1), P9(1), P10(1), P11(1), P12(1), P13(1), P14(1)];
    Points(:,:,2) = [P2(2), P1(2), P3(2), P4(2), P5(2), P6(2), P7(2), P8(2), P9(2), P10(2), P11(2), P12(2), P13(2), P14(2)];
    Points(:,:,3) = [P2(3), P1(3), P3(3), P4(3), P5(3), P6(3), P7(3), P8(3), P9(3), P10(3), P11(3), P12(3), P13(3), P14(3)];

    % para colocar ejes coordenados
    % piernas
    T0D = [eye(3),PD';0 0 0 1];
    TFD = [eye(3),PD';0 0 0 1]*T06;
    T0I = [eye(3),PI';0 0 0 1];
    TFI = [eye(3),PI';0 0 0 1]*T012;
    % brazos
    B0D = [eye(3),BD';0 0 0 1];
    BFD = [eye(3),BD';0 0 0 1]*B04;
    B0I = [eye(3),BI';0 0 0 1];
    BFI = [eye(3),BI';0 0 0 1]*B08;


    T0F = [T0D TFD T0I TFI B0D BFD B0I BFI];
end