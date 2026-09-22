%% ======================================================================
%  BATTLE.m  --  Duel AE86 vs FC3S  (version BOUCLE FERMEE, physique honnete)
%  ----------------------------------------------------------------------
%  Preuve d'emergence : on simule aussi la FC SANS perte de grip (temoin).
%  Sans la perte -> la FC garde la tete. Avec la perte -> la 86 passe.
%
%  Necessite MATLAB R2016b+ (fonctions locales dans un script).
%% ======================================================================
close all; clear; clc;

%% -------------------- CIRCUIT (centerline + bordures) -----------------
R = 38.0; xa = 48.2; ya = 38.0;   % arc : rayon et centre
road = 12.0;                       % largeur de piste (m)
ds = 0.25;                         % pas d'echantillonnage de la ligne (m)

% Centerline : droite d'entree (y=0) -> epingle 180 deg -> droite de sortie (y=76)
xs1 = (0:ds:xa-ds)';                         P1 = [xs1, zeros(size(xs1))];
phi = (-pi/2:ds/R:pi/2)';                    P2 = [xa+R*cos(phi), ya+R*sin(phi)];
xs3 = (xa:-ds:-130)';                         P3 = [xs3, (2*ya)*ones(size(xs3))];
P = [P1; P2; P3];                             N = size(P,1);

% Abscisse curviligne et cap (tangente) de la piste
S = [0; cumsum(hypot(diff(P(:,1)), diff(P(:,2))))];
PSI = atan2(gradient(P(:,2)), gradient(P(:,1)));

% Rayon local -> vitesse max en virage (precalcul, commun aux 2 voitures)
Rloc = local_radius(P, N);

% Bordures continues (pour l'affichage)
tb = linspace(-pi/2, pi/2, 160);
Ri = R - road/2;  Re = R + road/2;
x_int = [0, xa, xa+Ri*cos(tb), -40];   y_int = [ road/2,  road/2, ya+Ri*sin(tb), 2*ya-road/2];
x_ext = [0, xa, xa+Re*cos(tb), -40];   y_ext = [-road/2, -road/2, ya+Re*sin(tb), 2*ya+road/2];

%% -------------------- PARAMETRES DE SIMULATION ------------------------
dt = 0.004;  Tf = 11.0;
g  = 9.81;

% ----- Voiture 1 : TRUENO AE86 (suit la corde, jamais de perte de grip) -----
ae = car_params('AE86', 940, 1760);
ae.line_offset = 2.5;    % ligne interieure (la corde)  [m]
ae.y0 = ae.line_offset;  ae.x0 = 0;

% ----- Voiture 2 : MAZDA FC3S (part devant, perd le grip a 3.5 s) -----
fc = car_params('FC3S', 1205, 2200);
fc.line_offset = 0.0;    % ligne centrale
fc.y0 = 0;  fc.x0 = 16;  % 16 m devant au depart

% Evenement de grip de la FC : mu x 0.5 entre 3.5 s et 8.5 s
grip.t0 = 3.5;  grip.t1 = 8.5;  grip.fac = 0.5;
no_grip = struct('t0',0,'t1',0,'fac',1);   % pas d'evenement (temoin)

%% -------------------- SIMULATIONS -------------------------------------
RES_ae  = simCar(ae, no_grip, P,S,PSI,Rloc,N, Tf,dt,g);   % AE86
RES_fc  = simCar(fc, grip,    P,S,PSI,Rloc,N, Tf,dt,g);   % FC avec perte de grip
RES_fc0 = simCar(fc, no_grip, P,S,PSI,Rloc,N, Tf,dt,g);   % FC TEMOIN (sans perte)

T = RES_ae.T;

% Progression le long du circuit -> ecart de course (indice suivi de facon monotone)
prog_ae  = pathProgress(P,S,RES_ae.X, RES_ae.Y, N);
prog_fc  = pathProgress(P,S,RES_fc.X, RES_fc.Y, N);
prog_fc0 = pathProgress(P,S,RES_fc0.X,RES_fc0.Y,N);
gap  = prog_fc  - prog_ae;     % > 0 : FC devant
gap0 = prog_fc0 - prog_ae;     % temoin

kpass = find(diff(sign(gap))~=0, 1);   % instant du depassement
if isempty(kpass), t_pass = NaN; else, t_pass = T(kpass); end
fprintf('Depassement de la 86 a t = %.1f s ; a l''arrivee : AE86 devant de %.1f m\n', ...
        t_pass, -gap(end));
fprintf('FC : lacet max %.0f deg/s (SANS Cyaw), vitesse mini %.0f km/h\n', ...
        max(abs(RES_fc.r))*180/pi, min(RES_fc.U)*3.6);

%% -------------------- FIGURES D'ANALYSE -------------------------------
figure('Name','Duel AE86 vs FC3S','Color','w','Position',[80 80 1200 780]);

xr = [grip.t0 grip.t1 grip.t1 grip.t0];   % rectangle de l'evenement de grip

subplot(2,2,1); hold on; axis equal; grid on;
fill([x_ext fliplr(x_int)],[y_ext fliplr(y_int)],[.9 .9 .9],'EdgeColor','none');
plot(x_int,y_int,'k','LineWidth',1.3); plot(x_ext,y_ext,'k','LineWidth',1.3);
plot(P(:,1),P(:,2),'--','Color',[.6 .6 .6]);
hA=plot(RES_ae.X,RES_ae.Y,'b','LineWidth',2);
hF=plot(RES_fc.X,RES_fc.Y,'r','LineWidth',2);
if ~isnan(t_pass)
    plot(RES_ae.X(kpass),RES_ae.Y(kpass),'bp','MarkerSize',14,'MarkerFaceColor','b');
    plot(RES_fc.X(kpass),RES_fc.Y(kpass),'rp','MarkerSize',14,'MarkerFaceColor','r');
end
legend([hA hF],{'AE86 (Trueno)','FC3S'},'Location','southwest');
title('Trajectoires (pilote automatique, aucun braquage scripte)');
xlabel('X (m)'); ylabel('Y (m)'); xlim([-30 100]); ylim([-12 90]);

subplot(2,2,2); hold on; grid on;
yl=[min(gap)-2 max(gap)+2];
hG=fill(xr,[yl(1) yl(1) yl(2) yl(2)],[1 .9 .7],'EdgeColor','none');
plot([0 T(end)],[0 0],'r:');
if ~isnan(t_pass), plot([t_pass t_pass],yl,'b--'); end
h1=plot(T,gap,'k','LineWidth',2);
h2=plot(T,gap0,'--','Color',[.5 .5 .5],'LineWidth',1.4);
title('Ecart de course : le depassement EMERGE de la perte de grip');
xlabel('t (s)'); ylabel('avance  FC - AE86  (m)'); ylim(yl);
legend([h1 h2 hG],{'avec perte de grip','temoin (sans perte)','grip reduit'},'Location','southwest');

subplot(2,2,3); hold on; grid on;
fill(xr,[-40 -40 40 40],[1 .9 .7],'EdgeColor','none');
h1=plot(T,RES_fc.r*180/pi,'r','LineWidth',1.5);
h2=plot(T,RES_ae.r*180/pi,'b','LineWidth',1.5);
title('Vitesse de lacet -- bornee SANS FC\_Cyaw (pas de tete-a-queue)');
xlabel('t (s)'); ylabel('lacet (deg/s)'); legend([h1 h2],{'FC3S','AE86'}); ylim([-40 40]);

subplot(2,2,4); hold on; grid on;
fill(xr,[-3 -3 8 8],[1 .9 .7],'EdgeColor','none');
h1=plot(T,RES_fc.Fr/1000,'r','LineWidth',1.5);
h2=plot(T,RES_fc.Ff/1000,'r--','LineWidth',1.5);
h3=plot(T,RES_ae.Fr/1000,'b','LineWidth',1);
title('Forces laterales : chute nette du grip FC a t = 3.5 s');
xlabel('t (s)'); ylabel('force (kN)'); legend([h1 h2 h3],{'FC arriere','FC avant','AE86 arriere'});

%% -------------------- ANIMATION ---------------------------------------
figure('Name','INITIAL D : AE86 vs FC3S','Color','w'); hold on; axis equal; grid on;
fill([x_ext fliplr(x_int)],[y_ext fliplr(y_int)],[.9 .9 .9],'EdgeColor','none');
plot(x_int,y_int,'k','LineWidth',1.3); plot(x_ext,y_ext,'k','LineWidth',1.3);
xlabel('X (m)'); ylabel('Y (m)'); title('INITIAL D : AE86 vs FC3S');
xlim([-30 100]); ylim([-12 90]);

w = 1.8;   % largeur voiture (dessin)
ae_lx = [ae.Lf ae.Lf -ae.Lr -ae.Lr ae.Lf];  ae_ly = [w/2 -w/2 -w/2 w/2 w/2];
fc_lx = [fc.Lf fc.Lf -fc.Lr -fc.Lr fc.Lf];  fc_ly = [w/2 -w/2 -w/2 w/2 w/2];
h_ta = plot(0,0,'b-','LineWidth',1);  h_ca = plot(0,0,'b-','LineWidth',2);
h_tf = plot(0,0,'r-','LineWidth',1);  h_cf = plot(0,0,'r-','LineWidth',2);
h_txt = text(-25,85,'','FontSize',11,'BackgroundColor','w','Margin',3);

step = 12;
for k = 1:step:numel(T)
    c=cos(RES_ae.Th(k)); s=sin(RES_ae.Th(k));
    set(h_ta,'XData',RES_ae.X(1:k),'YData',RES_ae.Y(1:k));
    set(h_ca,'XData',RES_ae.X(k)+ae_lx*c-ae_ly*s,'YData',RES_ae.Y(k)+ae_lx*s+ae_ly*c);
    c=cos(RES_fc.Th(k)); s=sin(RES_fc.Th(k));
    set(h_tf,'XData',RES_fc.X(1:k),'YData',RES_fc.Y(1:k));
    set(h_cf,'XData',RES_fc.X(k)+fc_lx*c-fc_ly*s,'YData',RES_fc.Y(k)+fc_lx*s+fc_ly*c);
    if T(k)>=grip.t0 && T(k)<grip.t1, st = '  -- FC : perte de grip'; else, st = ''; end
    set(h_txt,'String',sprintf('t = %.1f s%s', T(k), st));
    drawnow; pause(0.01);
end

%% ======================================================================
%                        FONCTIONS LOCALES
%% ======================================================================
function car = car_params(name, m, Iz)
    g=9.81; Lf=1.1; Lr=1.3; L=Lf+Lr;
    Cf=69740; Cr=63460; mu=1.05;  Cp=1.3; Ep=-0.5;
    Fmaxf = mu*m*g*Lr/L;  Fmaxr = mu*m*g*Lf/L;
    car = struct('name',name,'m',m,'Iz',Iz,'Lf',Lf,'Lr',Lr,'L',L,'g',g, ...
        'mu',mu,'Cp',Cp,'Ep',Ep,'Fmaxf',Fmaxf,'Fmaxr',Fmaxr, ...
        'Bf',Cf/(Cp*Fmaxf),'Br',Cr/(Cp*Fmaxr), ...
        'v_cruise',88/3.6,'dmax',deg2rad(35), ...
        'a_acc',0.35*g,'a_brake',0.75*g,'mu_plan',1.05,'a_lat_use',0.62, ...
        'ke',2.2,'kc',0.00,'k_lift',0.40,'line_offset',0,'x0',0,'y0',0);
end

function res = simCar(car, grip, P,S,PSI,Rloc,N, Tf,dt,g)
    T=(0:dt:Tf)'; M=numel(T);
    X=zeros(M,1); Y=zeros(M,1); Th=zeros(M,1); U=zeros(M,1);
    Be=zeros(M,1); Rr=zeros(M,1); Ff=zeros(M,1); Fr=zeros(M,1); Del=zeros(M,1);
    x=car.x0; y=car.y0; th=0; u=car.v_cruise; beta=0; r=0;
    X(1)=x; Y(1)=y; U(1)=u;
    inear=nearestIdx(P,x,y,1,N);
    win=round(25/ (S(2)-S(1)));           % ~25 m de look-ahead vitesse
    ufloor=6.0;
    for k=2:M
        t=T(k);
        inear=nearestIdx(P,x,y,inear,N);
        % --- grip courant (perte de la FC pendant l'evenement) ---
        f=1.0; if t>=grip.t0 && t<grip.t1, f=grip.fac; end
        Fmaxf=car.Fmaxf*f; Fmaxr=car.Fmaxr*f;
        % --- PILOTE lateral : controleur STANLEY (essieu avant) ---
        xf=x+car.Lf*cos(th); yf=y+car.Lf*sin(th);
        i_f=nearestIdx(P,xf,yf,inear,N);
        psi_p=PSI(i_f);
        ex=xf-P(i_f,1); ey=yf-P(i_f,2);
        e_cross=-sin(psi_p)*ex+cos(psi_p)*ey;      % >0 : a gauche de la piste
        e_track=e_cross-car.line_offset;           % ecart / ligne suivie
        psi_err=wrapPi(psi_p-th);
        delta=psi_err+atan2(-car.ke*e_track, u);
        delta=delta-car.kc*r;                      % contre-braquage optionnel (kc=0 par defaut)
        delta=max(-car.dmax, min(car.dmax, delta));
        % --- PILOTE longitudinal : vitesse limitee par le grip planifie ---
        i2=min(N, inear+win);
        Rmin=min(Rloc(inear:i2));
        v_corner=sqrt(car.a_lat_use*car.mu_plan*g*Rmin);
        v_tar=min(car.v_cruise, v_corner);
        v_tar=v_tar/(1.0+car.k_lift*max(0.0, abs(e_track)-1.0));  % leve le pied si large
        if u<v_tar, u=min(v_tar, u+car.a_acc*dt); else, u=max(v_tar, u-car.a_brake*dt); end
        u=max(u, ufloor);
        % --- DYNAMIQUE : bicyclette + Pacejka, AUCUN Cyaw ---
        a_f=beta+car.Lf*r/u-delta;
        a_r=beta-car.Lr*r/u;
        Ff1=pace(a_f,car.Bf,car.Cp,car.Ep,Fmaxf);
        Fr1=pace(a_r,car.Br,car.Cp,car.Ep,Fmaxr);
        r_dot=(car.Lf*Ff1-car.Lr*Fr1)/car.Iz;
        beta_dot=(Ff1+Fr1)/(car.m*u)-r;
        r=r+dt*r_dot;  beta=beta+dt*beta_dot;  th=th+dt*r;
        vlat=u*tan(max(-1.2,min(1.2,beta)));
        x=x+(u*cos(th)-vlat*sin(th))*dt;
        y=y+(u*sin(th)+vlat*cos(th))*dt;
        X(k)=x; Y(k)=y; Th(k)=th; U(k)=u; Be(k)=beta; Rr(k)=r;
        Ff(k)=Ff1; Fr(k)=Fr1; Del(k)=delta;
    end
    res=struct('T',T,'X',X,'Y',Y,'Th',Th,'U',U,'beta',Be,'r',Rr, ...
               'Ff',Ff,'Fr',Fr,'delta',Del);
end

function Fy = pace(alpha,B,C,E,Fmax)   % formule magique de Pacejka
    Fy = -Fmax*sin(C*atan(B*alpha - E*(B*alpha - atan(B*alpha))));
end

function i = nearestIdx(P,x,y,ih,N)     % point de la ligne le plus proche (fenetre avant)
    lo=max(1,ih-5); hi=min(N,ih+400);
    d=(P(lo:hi,1)-x).^2+(P(lo:hi,2)-y).^2;
    [~,j]=min(d); i=lo+j-1;
end

function prog = pathProgress(P,S,X,Y,N)  % abscisse curviligne, hint suivi (monotone)
    M=numel(X); prog=zeros(1,M); ih=1;
    for k=1:M
        ih=nearestIdx(P,X(k),Y(k),ih,N);
        prog(k)=S(ih);
    end
end

function Rr = local_radius(P,N)         % rayon local par cercle circonscrit (3 points)
    Rr=1e6*ones(N,1); w=8;
    for i=1:N
        i0=max(1,i-w); i1=min(N,i+w);
        a=P(i0,:); b=P(i,:); c=P(i1,:);
        ab=b-a; cb=b-c;
        cr=ab(1)*cb(2)-ab(2)*cb(1);
        if abs(cr)>1e-6
            Rr(i)=min(1e6,(hypot(ab(1),ab(2))*hypot(cb(1),cb(2))*hypot(a(1)-c(1),a(2)-c(2)))/(2*abs(cr)));
        end
    end
end

function a = wrapPi(a),  a = mod(a+pi, 2*pi) - pi;  end
