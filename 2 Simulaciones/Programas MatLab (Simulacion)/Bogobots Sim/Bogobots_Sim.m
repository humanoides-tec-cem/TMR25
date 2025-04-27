% L = [L1 L2 L3 L4 D1 D2 D3 D4]
        % L1 = dist eje de sym en Z del robot y la pata
        % L2 = se desprecia el valor en la cinematica
        % L3 = dist muslo
        % L4 = dist pierna
        % D1 = altura del torax (dist piernas a brazos)
        % D2 = dist eje pierna a ejee brazo sobre Y
        % D3 = dist brazo
        % D4 = dist antebrazo

% IDs 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18

% q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16 q17 q18]

    % rmpath('C:\Program Files\MATLAB\R2024a\toolbox\phased\phased');
    L = [7 0 18.5 18.5 32 4 20 19]; % Bogo 3

    % ------------- algoritmo sencillo para probar la animacion -------------
    % qR = [0 0 pi/6 -pi/6 2*pi/7 -2*pi/7 0 0 0 0 pi/8 -pi/8 -2*pi/8 2*pi/8 -pi/8 pi/8 0 0; ...
    %       0 0 pi/7 -pi/7 2*pi/7 -2*pi/7 0 0 0 0 pi/6 -pi/6 -2*pi/6 2*pi/6 -pi/6 pi/6 0 0; ...
    %       0 0 pi/6 -pi/6 2*pi/7 -2*pi/7 0 0 0 0 pi/8 -pi/8 -2*pi/8 2*pi/8 -pi/8 pi/8 0 0];

    % ------------- pose inicial propuesta en espacio articular
    %qR = [0 0 pi/6 -pi/6 2*pi/7 -2*pi/7 0 0 0 0 pi/8 -pi/8 -2*pi/8 2*pi/8 -pi/8 pi/8 0 0]; % start position
    %qS = [0 0 pi/4 -pi/6 2*pi/7 -2*pi/7 0 0 0 0 pi/8 -pi/8 -2*pi/8 2*pi/8 -pi/8 pi/8 0 0];

    % DEFINIR EL SET POINT AL QUE VA A LLEGAR EL ROBOT S = [P;K] 
    % Donde P es el vector de posicion y K el del orientacion
    setPoint = [0; 0; -25; 0; 0; 0];

    % LEER LA POSE DEL ROBOT
    ptsR = [0 0 -31 0, 0 0 -31 0, 12 5 -25, 12 -5 -25]; %start pos
    q = IK_robot (L, ptsR); % esta es la posicion actual de los acutadores del robot 
    qDraw = q;

    for i = 1:1
    % CALCULAR LA CINEMATICA DIRECTA PARA OBTENER LA POSE ACTUAL DEL ROBOT
    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);
    q7 = q(7); q9 = q(9); q11 = q(11); q13 = q(13); q15 = q(15); q17 = q(17);

    % vector de posicion P
    x = L3*(cos(q7)*sin(q11) - cos(q11)*sin(q7)*sin(q9)) + L4*(cos(q13)*(cos(q7)*sin(q11) - cos(q11)*sin(q7)*sin(q9)) + sin(q13)*(cos(q7)*cos(q11) + sin(q7)*sin(q9)*sin(q11)));
    y = - L3*(sin(q7)*sin(q11) + cos(q7)*cos(q11)*sin(q9)) - L4*(cos(q13)*(sin(q7)*sin(q11) + cos(q7)*cos(q11)*sin(q9)) + sin(q13)*(cos(q11)*sin(q7) - cos(q7)*sin(q9)*sin(q11)));
    z = - L4*(cos(q9)*cos(q11)*cos(q13) - cos(q9)*sin(q11)*sin(q13)) - L3*cos(q9)*cos(q11);

    % vector de orientacion K
    T01 = [rotz(-q7),[0;0;0]; 0 0 0 1]; % cadera
    T12 = [rotx(-q9),[0;0;0]; 0 0 0 1]; % cadera
    T23 = [roty(-q11),[0;0;0]; 0 0 0 1]; % cadera
    T34 = [roty(-q13),[0;0;-L3]; 0 0 0 1]; %rodilla
    T45 = [roty(q15),[0;0;-L4]; 0 0 0 1]; %tobillo
    T56 = [rotx(q17),[0;0;0]; 0 0 0 1]; %tobillo
    T06 = T01*T12*T23*T34*T45*T56; 
    rot = T06(1:3,1:3)
    isdiag(rot)
    
    if isdiag(rot)
        K = (2*pi/2)*[T06(1,1)+1; ...
                    T06(2,2)+1; ...
                    T06(3,3)+1];
    else

        l = [T06(3,2)-T06(2,3); ...
             T06(1,3)-T06(3,1); ...
             T06(2,1)-T06(1,2)];
    
        mag_l = sqrt(l(1,1)^2 + l(2,1)^2 + l(3,1)^2);
        theta = atan2(mag_l, T06(1,1) + T06(2,2)+ T06(3,3) -1);
    
        K = theta * (l/mag_l)

    end

    K(1,1) = atan2(sin(K(1,1)),cos(K(1,1)));
    K(2,1) = atan2(sin(K(2,1)),cos(K(2,1)));
    K(3,1) = atan2(sin(K(3,1)),cos(K(3,1)));
    K

    P_Robot = [x;y;z;K]

    % EVALUAR EL JACOBIANO
    [Jt_RLeg, Jt_LLeg, Jt_RArm, Jt_LArm] = jacobians (L,q); % usar la q actual del robot
    q_RLeg = [q(7) q(9) q(11) q(13) q(15) q(17)]'
    % USAR NEWTON RAPSHON
    error = P_Robot-setPoint
    qNew_RLeg = q_RLeg - inv(Jt_RLeg)*(error);

    %Jt_RLeg
    %inv(Jt_RLeg)
    q_RLeg 
    inv(Jt_RLeg)*(error)
    qNew_RLeg

    %qNew = q; 
    q(7) = qNew_RLeg(1);
    q(9) = qNew_RLeg(2);
    q(11) = qNew_RLeg(3);
    q(13) = qNew_RLeg(4);
    q(15) = qNew_RLeg(5);
    q(17) = qNew_RLeg(6);
    qDraw = [qDraw; q];

    q7 = q(7); q9 = q(9); q11 = q(11); q13 = q(13); q15 = q(15); q17 = q(17);
    x = L3*(cos(q7)*sin(q11) - cos(q11)*sin(q7)*sin(q9)) + L4*(cos(q13)*(cos(q7)*sin(q11) - cos(q11)*sin(q7)*sin(q9)) + sin(q13)*(cos(q7)*cos(q11) + sin(q7)*sin(q9)*sin(q11)))
    y = - L3*(sin(q7)*sin(q11) + cos(q7)*cos(q11)*sin(q9)) - L4*(cos(q13)*(sin(q7)*sin(q11) + cos(q7)*cos(q11)*sin(q9)) + sin(q13)*(cos(q11)*sin(q7) - cos(q7)*sin(q9)*sin(q11)))
    z = - L4*(cos(q9)*cos(q11)*cos(q13) - cos(q9)*sin(q11)*sin(q13)) - L3*cos(q9)*cos(q11)

    end
    % ptsR = [0 0 -35 0, 0 0 -20 0, 12 5 -25, 12 -5 -25]; %start pos
    % qR = IK_robot (L, ptsR); % esta es la posicion actual de los acutadores del robot
    % qDraw = qR; % guardar en el vector para dibujar
    % dt  = 0.1; % tiempo entre cada pose
    % 
    % pts1 = [0 5 -20 0, 0 0 -35 0, 12 5 -25, 12 -5 -15]; %set point
    % pts2 = [0 0 -35 0, 0 0 -20 0, 12 5 -15, 12 -5 -25]; %set point
    % qS = [IK_robot(L, pts1); IK_robot(L, pts2);...
    %       IK_robot(L, pts1); IK_robot(L, pts2);
    %       IK_robot(L, pts1); IK_robot(L, pts2)]; % set point
    % 
    % for j = 1:size(qS,1)
    %     for i = 1:30
    %         % encontrar la posicion por Newton Rapson
    %         qR = ctr_NewtonRapson(L, qR, qS(j,:));
    %         qDraw = [qDraw; qR]; % guardar en el vector para dibujar
    %     end
    % end
    
    % --------------------------------------------------------------------
    % usar la FK para calcular donde quedan todas las cosas que se van a dibujar
    %qDraw = [q; qNew];

    [Points,T] = MoveRobot_FK(L,qDraw);
    % dibujar el robot
    dt = 0.1;
    drawRobot (L,Points,T,dt,0);

