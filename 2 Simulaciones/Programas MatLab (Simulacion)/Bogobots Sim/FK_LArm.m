function B08 = FK_LArm(L,q)
% 
% Regresa la matriz de transformacion homogenea del sistema coordenado
% inercial hasta la punta del brazo izquierdo del robot humanoide

% parametros del robot
D1 = L(5); D2 = L(6); D3 = L(7); D4 = L(8);

% valores articulares
q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);

% FK brazo izquierdo
B05 = [roty(q2),[0;0;0]; 0 0 0 1]; % hombro
B56 = [rotx(-q4),[0;0;0];0 0 0 1]; % hombro
B67 = [rotx(q6), [0;0;-D3]; 0 0 0 1]; % codo
B78 = [eye(3), [0;0;-D4]; 0 0 0 1]; % mano

B08 = B05*B56*B67*B78;

end