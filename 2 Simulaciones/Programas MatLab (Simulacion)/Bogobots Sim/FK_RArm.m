function B04 = FK_RArm(L,q)
% 
% Regresa la matriz de transformacion homogenea del sistem coordenado
% inercial hasta la punta del brazo derecho del robot humanoide

% parametros del robot
D1 = L(5); D2 = L(6); D3 = L(7); D4 = L(8);

% valores articulares
q1 = q(1); q3 = q(3); q5 = q(5);

% FK brazo derecho
B01 = [roty(-q1),[0;0;0]; 0 0 0 1]; % hombro
B12 = [rotx(-q3),[0;0;0]; 0 0 0 1]; % hombro
B23 = [rotx(q5), [0;0;-D3]; 0 0 0 1]; % codo
B34 = [eye(3), [0;0;-D4]; 0 0 0 1]; % mano

B04 = B01*B12*B23*B34;
end