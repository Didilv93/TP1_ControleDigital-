% ELT013 - Controle Digital
%
% Exemplo de projeto de controlador digital pelo lugar das raizes:
%
% Prof. Bruno Teixeira, DELT/UFMG

close all; clear all; clc;

%% Escolha do periodo de amostragem:
T = 0.05;

%% Modelo discreto do PROCESSO:

% Processo instavel em MA:
Gps = tf([1], [1 -2]);

% Planta discreta:
Gz = c2d(Gps, T, 'zoh');
zpk(Gz)

%% Especificacoes de desempenho:

Mpmax = 25; % percentual de overshoot
ts = 0.5; % tempo de subida
ta = 1.5; % tempo de acomodacao

% Regioes que atendem as especificacoes:
zetamin = 0.6*(1 - Mpmax/100) % taxa de amortecimento
wnmin = 1.8/ts % frequencia natural
zetawnmin = 4.6/ta % zeta*wn, em que wn esta em rad/s

% Especificacoes (um pouco mais rigoross que valores minimos) que atendem regioes:
zeta = 0.55
wn = 6.0*T % em rad
zeta*wn/T % Observe que  esse valor deve ser > zetawnmin calculado acima
r0 = exp(-zeta*wn) % a regiao interna a esse raio delimita o ta minimo

%% Projeto direto:

% Projeto digital usando lugar das raizes:
figure; zgrid; % referencia do grid

%% 1) Considerando compensador estatico:
K = [0:0.001:50];
figure; rlocus(Gz, K); zgrid(zeta, wn); hold on; plotcircle(0,0,r0,'k-.')
Kc1 = 8;
[Kc1, raizes1] = rlocfind(Gz)
figure; bode(Gz*Kc1); grid on;
% Simulacao da resposta ao degrau em MF
Tz1 = feedback(Gz*Kc1,1);
zpk(Tz1)
figure; step(Tz1); grid on;
figure(5); Tuz1 = Kc1/(1+Gz*Kc1); step(Tuz1); grid;

%% 2) PI:
zc = 0.9; % zero do controlador
Dz = tf([1 -zc], [1 -1], T);

figure; rlocus(Gz*Dz, K); zgrid(zeta, wn); hold on; plotcircle(0,0,r0,'k-.')
[Kc2, raizes2] = rlocfind(Gz*Dz)
 Kc2 = 20;
figure; bode(Gz*Dz*Kc2); grid on;
figure; nyquist(Gz*Dz*Kc2); grid on;
% Simulacao da resposta ao degrau em MF
Tz2 = feedback(Gz*Dz*Kc2,1);
zpk(Tz2)
figure; step(Tz2);grid on;
figure(5); Tuz2 = Kc2*Dz/(1+Gz*Kc2*Dz); step(Tuz2); grid;