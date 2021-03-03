% Controle Digital
%
% Exemplo de projeto de controlador digital pelo lugar das raizes:
% Sistema motor DC;
% No domínio 's', G(s) = K/[(Js + b) * (Ls + R) + K^2];
% Entrada: Tensão
% Saída: Velocidade

close all; clear; clc;

%% Escolha do periodo de amostragem:
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;

%% Modelo continuo do processo

% Processo em MA:
num=K;
den = [(J * L) (J * R + L * b) (b * R + K ^ 2)];
Gps = tf(num, den);


% Resposta contínua em malha aberta
figure; step(Gps, 0:0.1:5);
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
title('Resposta ao degrau para MA, sistema contínuo');

%% Modelo discreto do processo:

% Periodo de amostragem
% Calculando pelo tempo de acomodação ts
% ts/15 < T < ts/6 Ou aproximadamente 0.12 < T < 0.3

T = 0.15;

% Planta discreta:
[numz,denz] = c2dm(num,den,T,'zoh');
Gz = c2d(Gps, T, 'zoh');
zpk(Gz);

% Resposta discreta em malha fechada
sys = feedback(Gz, 1);
[y, t] = step(sys, 5);
figure; stairs(t,y);
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
title('Resposta ao degrau para MF, sistema discreto');

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


%% 1) PID:

Kp = 100;
Ki = 200;
Kd = 10;

[dencz,numcz]=c2dm([1 0],[Kd Kp Ki],T,'tustin'); 

numaz = conv(numz,numcz);
denaz = conv(denz,dencz);
[numaz_cl,denaz_cl] = cloop(numaz,denaz);

[x2] = dstep(numaz_cl,denaz_cl,101);
t=0:0.12:12;
stairs(t,x2)
xlabel('Time (seconds)')
ylabel('Velocity (rad/s)')
title('Stairstep Response:with PID controller')

%2 Considerando compensador estatico

rlocus(numaz,denaz)
title('Root Locus of Compensated System')

dencz = conv([1 -1],[1.6 1]);
numaz = conv(numz,numcz);
denaz = conv(denz,dencz);

rlocus(numaz,denaz)
title('Root Locus of Compensated System');

% Escolher um polo em -0.625 para anular o zero do sistema não compesado

[K,poles] = rlocfind(numaz,denaz);
[numaz_cl,denaz_cl] = cloop(K*numaz,denaz);

[x3] = dstep(numaz_cl,denaz_cl,101);
t=0:0.12:12;
stairs(t,x3)
xlabel('Tempo (s)'); ylabel('Velocidade (rad/s)');
title('Sistema com controlador PID compensado, Resposta ao degrau para MF');
 