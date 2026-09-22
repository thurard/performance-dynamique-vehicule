close all;  
clear;      
clc;    
%Caractéritisque prédéfinies :
m = 1310 ;  %Masse du véhicule en Kg
Iz = 1760 ; %Moment d'inertie en Kg.m^+2
Lf = 1.2 ;  %Demi empattement avant en m
Lr = 1.4 ;  %Demi empattement arrière en m 
L = Lr+Lf;  %empattement total en m
Cf = 69740; %Rigidité de dérive avant en N.rad^-1
Cr = 63460; %Rigidité de dérive arrière en N.rad^-1

%Variables de temps :
dt =0.01;
Tf = 10;
T_ = (0:dt:Tf)';
Nb=length(T_);

%Entrées du modèle : 
u_ = 50/3.6 * ones (length(T_),1); %La vitesse linéaire du véhicule au CdG en m.s^-1
u_(T_>=3) = 55/3.6; %Légère accélération
Beta_= 6* pi/180* ones (length(T_),1); %Angle de braquage en rad

%Les differentes sorties : 

% Initialisation des états à l'instant k=1 :
% dTheta_0 est la vitesse de lacet r (rad/s)
dTheta_0 = 0;
Delta_0 = 0; 
Theta_0 = 0; 
x_0=0;
y_0=0;
% Stockage: 
dTheta_ = zeros(Nb, 1);    % Vitesse de lacet r
Delta_ = zeros(Nb, 1);     % Angle de dérive beta
Theta_ = zeros(Nb, 1);     % Angle de cap
Delta_r = zeros(Nb, 1);    % Angle de dérive arrière alpha_r
Delta_f = zeros(Nb, 1);    % Angle de dérive avant alpha_f
dThetaSG_=zeros(Nb, 1);    % Vitesse de lacet r sans glissement
Ff_=zeros(Nb, 1);          % Force latérale avant
Fr_=zeros(Nb, 1);          % Force latérale arrière
x_=zeros(Nb,1);            % Position x CdG
y_=zeros(Nb,1);            % Position y CdG
v_=zeros(Nb,1);            % Calcul de la vitesse latérale v (m/s)

for k = 2:Nb
    u_1=u_(k);
    Beta_1=Beta_(k);
    dThetaSG_=u_1/L * tan(Beta_);

    %Composantes de la matrice A (Jacobienne) :

    a11 = -(Lr^2*Cf + Lr^2*Cr)/(u_1*Iz);
    a12 = (Lr*Cr - Cf*Lf)/Iz;
    a21 = (Lr*Cr - Lf*Cf)/(m*(u_1).^2)-1;
    a22 = -(Cf+Cr)/(m*u_1);
    b1 =  Lf*Cf/Iz;
    b2 =  Cf/(m*u_1);

    %Valeur des differents angles :

    %Mise à jour de l'état
    d2Theta_1 = a11 * dTheta_0 + a12 * Delta_0 + b1 * Beta_1; %Vitesse de lacet
    dDelta_1 = a21 * dTheta_0 + a22 * Delta_0 + b2 * Beta_1; %Angle de dérive global

    dTheta_1 = dTheta_0 + dt * d2Theta_1;
    Theta_1 = Theta_0 + dt * dTheta_1;
    Delta_1 = Delta_0 + dt * dDelta_1;

    % Composante latérale de la vitesse au CdG
    Vy_1 = u_1 * Delta_1; 

    % Norme de la vitesse du CdG
    V_1 = sqrt(u_1^2 + Vy_1^2);

    % Vitesse latérale à l'essieu avant
    Vy_f_1 = u_1 * Delta_1 + Lf * dTheta_1;

    % Norme de la vitesse à l'essieu avant
    uf_1 = sqrt(u_1^2 + Vy_f_1^2);

    %Mise à jour des dérives avant et arrière
    Delta_r1 = Delta_1 - Lr * dTheta_1 / u_1;
    Delta_f1 = Delta_1 - Beta_1 + Lf * dTheta_1 / u_1;

    %Mise à jour tableau x et y
    x_1=x_0 + u_1*cos(Theta_1)*dt;
    y_1=y_0 + u_1*sin(Theta_1)*dt;
    
    %Mise à jour des forces 
    Ff_1 = -Cf * Delta_f1;
    Fr_1 = -Cr * Delta_r1;
    
    %Sortie calculées via les entrés
    dTheta_(k) = dTheta_1;
    Delta_(k) = Delta_1;
    Theta_(k) = Theta_1;
    Delta_r(k) = Delta_r1;
    Delta_f(k) = Delta_f1;
    x_(k) = x_1;
    y_(k)=y_1;
    v_(k) = u_1 .* tan(Delta_1);
    Ff_(k) = Ff_1;
    Fr_(k) = Fr_1;

    dTheta_0 = dTheta_1; 
    Delta_0 = Delta_1;
    Theta_0 = Theta_1;
    x_0=x_1;
    y_0=y_1;
    
end

% Position y/x
figure;
plot(x_,y_);
title('Position y/x');
xlabel('x');
ylabel('y');

% Tracé de la variations vitesse de lacet / Temps
figure;
plot(T_,dTheta_ * 180/pi);
title('Tracé de la variations vitesse de lacet / Temps');
xlabel('Temps (s)');
ylabel('Vitesse de lacet (deg/s)');

% Tracé de la variations angle de dérive CdG / Temps
figure;
plot(T_,Delta_ * 180/pi);
title('Tracé de la variations angle de dérive CdG / Temps');
xlabel('Temps');
ylabel('Angle de dérive (deg)');

% Tracé de la variations des angles de dérive av/ar / Temps
figure;
plot(T_,Delta_r);
hold on;
plot(T_,Delta_f);
legend('Delta_r','Delta_f');
title('Tracé de la variations des angles de dérive av/ar / Temps');
xlabel('Temps');
ylabel('Deg/s')
hold off;

% Tracé de la variation des forces avant et arrière / Temps
figure;
plot(T_,Ff_);
hold on;
plot(T_,Fr_);
legend('Ff','Fr');
title('Tracé de la variation des forces avant et arrière / Temps');
xlabel('Temps');
ylabel('N');
hold off;

% Tracé de la vitesse longitudinale du CdG (u) / Temps
figure;
plot(T_, u_);
title('Vitesse longitudinale du CdG (u) / Temps');
xlabel('Temps (s)');
ylabel('Vitesse (m/s)');
grid on;

% Tracé de la vitesse latérale v (m/s) par rapport au temps
figure;
plot(T_,v_);
title('Tracé de la vitesse latérale v (m/s) par rapport au temps');
xlabel('Temps');
ylabel('vitesse latérale (m/s)');