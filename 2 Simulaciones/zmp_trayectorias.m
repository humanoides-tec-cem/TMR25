%% TEST ZMP ECUATIONS -----------------------------------------------------
% h 50 entre partas 11 
% constatntes 
g = 981; % gravedad (cm/s2)
h = 10%40; % altura del robot (cm)

% generales
tf = 0.4; % tiempo que dura el paso (s)

%% Y ZMP ------------------------------------------------------------------

% generales 
Yzmp = 10.2; % lo max que se desplaza la cadera (cm)

% para el caso del movimiento en Y
k1 = (-Yzmp)/(1+exp(sqrt(g/h)*tf));
k2 = k1*exp(sqrt(g/h)*tf);

syms t
y(t) = k1*exp(sqrt(g/h)*t) + k2*exp(-sqrt(g/h)*t) + Yzmp;

%% X ZMP ------------------------------------------------------------------
syms t
% generales 
Xzmp = 1; % lo max que se desplaza la cadera (cm)

%while t < tTot
    
    c = 2;
    if c == 1 
        1
        % caso 1: separar los pies
        k1 = (-Xzmp)/(1-exp((sqrt(g/h)*tf)));
        k2 = k1*exp((-sqrt(g/h)*tf));
    end
    if c == 2
        2
        % caso 2: pasar pie de atras a adelante
        k1 = (-Xzmp*exp((sqrt(g/h)*tf))) / ((1-exp((sqrt(g/h)*tf)))* (1+exp((sqrt(g/h)*tf))));
        k2 = -k1;
    end 
    if c == 3
        % caso 3: juntar los pies
        k1 = (-Xzmp) / ((1-exp((sqrt(g/h)*tf)))* (1+exp((sqrt(g/h)*tf))));
        k2 = -k1*exp(2*(sqrt(g/h)*tf));
    end
    % ecuacion general 
         
    x_ec1(t) = k1*exp(sqrt(g/h)*t) + k2*exp(-sqrt(g/h)*t);

%end
%% Graficar las 3 trayectorias de la cadera
t = linspace(0,tf);

subplot(2,1,1);
plot(t,x_ec1(t));
title('Pos X[t] (Ec3)')
xlabel("t [seg]")
ylabel("x [cm]")

subplot(2,1,2);
plot(t,y(t));
title('Pos Y[t]')
xlabel("t [seg]")
ylabel("y [cm]")

%subplot(2,2,4);
figure
plot(x_ec1(t),y(t));
title('Pos cadera X[t] (Ec3), Y[t]')
xlabel("x [cm]")
ylabel("y [cm]")

