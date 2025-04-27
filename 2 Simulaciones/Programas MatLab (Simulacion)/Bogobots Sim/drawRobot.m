% Esta funcion dibuja al robot humanoide completo en el espacio 3D. Por
% favor no modificar esta funcion porque podria ser fatal para el programa.

%   Estructura del vector de las dimensiones del robot.
%       L = [leg1 leg2 leg3 leg4 arm1 arm2 arm3 arm4]

%   Escructura de los puntos para dibujar que recibe la funcion 
%       Points(:,:,1) = posicion de X de todos los puntos
%       Points(:,:,2) = posicion de Y de todos los puntos
%       Points(:,:,3) = posicion de Z de todos los puntos

% Transformaciones homogeneas del frame inercial al frame final de cada una
% de las cadenas.
%   T = [T0D T01 B0D B0I]

% Intervalo de tiempo entra cada una de las poses 
%   dt = #s

% Bandera para activar o desactivar que se dibujen los frames
%   frame_ON = 1/0

function drawRobot (L,Points,T,dt,frame_ON)
%   
% Funcion que dibuja el robot humanoide y recibe:
%
%   L = distancias de las extremidades del robot
%   Points = arreglo con puntos (x,y,z) del robot
%   T = arreglo con matrices 0 a F de cada cadena
%   dt = intervalo de tiempo para dibujar
%   frame_ON = intervalo de tiempo entre cada pose

    L1 = L(1); L2 = L(2); L3 = L(3); L4 = L(4);
    D1 = L(5); D2 = L(6); D3 = L(7); D4 = L(8);

    j = 1;
    for i=1:size(Points,1)
        
        plot3(Points(i,1:8,1)',Points(i,1:8,2)',Points(i,1:8,3)',"-bo") % piernas
        hold on 
        plot3(Points(i,9:end,1)',Points(i,9:end,2)',Points(i,9:end,3)',"-ro") %brazos
        plot3([0 0]', [-L1,-L1-D2]', [L3+L4,L3+L4+D1]',"k") % torax
        plot3([0 0]', [L1,L1+D2]', [L3+L4,L3+L4+D1]',"k") % torax
        % plot3([0 0]', [0 0]', [L3+L4+D1,L3+L4+D1+5]',"-ko")
        hold off

        xlabel("X"); ylabel("Y"); zlabel("Z")
        grid on; grid minor
        axis([-1 1 -1 1 0 1]*75); 

        if frame_ON == 1
            addFrames(T(j:j+3,:),4);
        end
        %view(90,0) %front view
        %view(0,0) %side view
        view(45,20)
        pause(dt)
        j = j+4;
    end
end

function addFrames(T,Tsize)
    T0D = T(:,1:4); TFD = T(:,5:8);
    T0I = T(:,9:12); TFI = T(:,13:16); 
    B0D = T(:,17:20); BFD = T(:,21:24);
    B0I = T(:,25:28); BFI = T(:,29:32);
    
    hold on
    trplot(T0D,'length',Tsize,"color","#DDDDDD","frame","T0D")
    trplot(TFD,'length',Tsize,"color","#DDDDDD","frame","TFD")
    trplot(T0I,'length',Tsize,"color","#DDDDDD","frame","T0I")
    trplot(TFI,'length',Tsize,"color","#DDDDDD","frame","TFI")

    trplot(B0D,'length',Tsize,"color","#DDDDDD","frame","B0D")
    trplot(BFD,'length',Tsize,"color","#DDDDDD","frame","BFD")
    trplot(B0I,'length',Tsize,"color","#DDDDDD","frame","B0I")
    trplot(BFI,'length',Tsize,"color","#DDDDDD","frame","BFI")
    hold off
end