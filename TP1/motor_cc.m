% Controle Digital
%
% Exemplo de projeto de controlador digital pelo lugar das raizes:
% Sistema motor cc;
% No domínio 's', H(s) = K/(Js + b) * (Ls + R) + K^2;

close all; clear; clc;

%% Escolha do periodo de amostragem:
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;

%% Modelo discreto do PROCESSO:

% Processo instavel em MA:
num=K;
den = [(J * L) (J * R + L * b) (b * R + K ^ 2)];
Gps = tf(num, den);


% Resposta continua em malha aberta
figure; step(Gps, 0:0.1:5);
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
title('Resposta ao degrau para MA');

% Periodo de amostragem
T = 0.12;

% Planta discreta:
Gz = c2d(Gps, T, 'zoh');
zpk(Gz);

% Resposta discreta em malha fechada
sys = feedback(Gz, 1);
[y, t] = step(sys, 5);
figure; stairs(t,y);
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
title('Resposta ao degrau para MF');

%% Especificacoes de desempenho:

Mpmax = 10; % percentual de overshoot
ts = 1.82; % tempo de subida
ta = 3; % tempo de acomodacao

% Regioes que atendem as especificacoes:
zetamin = 0.6*(1 - Mpmax/100); % taxa de amortecimento
wnmin = 1.8/ts; % frequencia natural
zetawnmin = 4.6/ta; % zeta*wn, em que wn esta em rad/s

% Especificacoes (um pouco mais rigoross que valores minimos) que atendem regioes:
zeta = 0.5;
wn = 6.0*T; % em rad
zeta*wn/T; % Observe que  esse valor deve ser > zetawnmin calculado acima
r0 = exp(-zeta*wn); % a regiao interna a esse raio delimita o ta minimo

%% Projeto direto:

% Projeto digital usando lugar das raizes:
figure; zgrid; % referencia do grid

%% 1) Considerando compensador estatico:
K = 0:0.001:50;
figure; rlocus(Gz, K); zgrid(zeta, wn); hold on; plotcircle(0,0,r0,'k-.');
[Kc1, raizes1] = rlocfind(Gz);
figure; bode(Gz*Kc1); grid on;
% Simulacao da resposta ao degrau em MF
Tz1 = feedback(Gz*Kc1,1);
zpk(Tz1);
figure; step(Tz1); grid on;
figure(5); Tuz1 = Kc1/(1+Gz*Kc1); step(Tuz1); grid;

%% 2) PID:
zc = 0.9; % zero do controlador
Dz = tf([1 -zc], [1 -1], T);

figure; rlocus(Gz*Dz, K); zgrid(zeta, wn); hold on; plotcircle(0,0,r0,'k-.')
[Kc2, raizes2] = rlocfind(Gz*Dz);
figure; bode(Gz*Dz*Kc2); grid on;
figure; nyquist(Gz*Dz*Kc2); grid on;
% Simulacao da resposta ao degrau em MF
Tz2 = feedback(Gz*Dz*Kc2,1);
zpk(Tz2)
figure; [x2, t] = step(Tz2);grid on;
figure(5); Tuz2 = Kc2*Dz/(1+Gz*Kc2*Dz); step(Tuz2); grid;

figure; stairs(t, x2);
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
title('Sistema com controlador PI, Resposta ao degrau para MF');
