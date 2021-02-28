% ELT013 - Controle Digital
%
% Exemplo de projeto de controlador digital pelo lugar das raizes:
% Controle de azimute de uma antena para captacao de sinais de satelite
%
% Prof. Bruno Teixeira, DELT/UFMG, 12/05/2017
% (baseado nas notas de aula do Prof. Reinaldo Palhares)

close all; clear all; clc;

%% Escolha do periodo de amostragem:
T = 1; % Avalie o efeito da mudan?a de T no projeto. Esse valor e suficiente???

%% Modelo discreto do PROCESSO:

% Planta analogica:
tau = 10; % J/B (momento de inercia/atrito viscoso)
numGs = 1;
denGs = [tau 1 0];
Gs = tf(numGs, denGs);
Gs = zpk(Gs) % Forma fatorada da FT

% Processo discreto:
Gz = c2d(Gs, T, 'zoh')

%% Especificacoes de desempenho:

% Rastreamento de rampa y = 0,01t rad/s com ess <= 0,01 rad
Kvmin = 1;

% Maxima sobre-elevacao
Mpmax = 16;
% ===> zeta >= 0.6*(1 - Mpmax/100)
zetamin = 0.6*(1 - Mpmax/100)

% Tempo de acomodacao maximo
tsmax = 10;
% ===> zeta wn >= 4,6 / tsmax:
zetawnmin = 4.6/tsmax
raiomin = exp(-T*zetawnmin)

figure(1); zgrid(zetamin,[],'new'); hold on; plotcircle(0,0,raiomin,'k-.');

%% Projeto direto: COMPENSADOR ESTATICO

hold on; 
K = [0:0.005:50];
rlocus(Gz,K);
zgrid(zetamin,[]); hold on; plotcircle(0,0,raiomin,'k-.');
axis([-1.1 1.1 -1.1 1.1]);
[Kc, raizes] = rlocfind(Gz) 

Kcr = 2; % Ganho critico

% Avaliacao de desempenho:
Kv = dcgain(Kcr*Gz*tf([1 -1],[T 0],T))
% OBS1: Kv = K ==> basta escolher K > Kvmin
%       Para K < 0.08, tem-se zeta < zetamin. 
%       No entanto, com isso o Kv = 0.08 << 1 nao e satisfeito.
%       Nao ha valor de K que satisfaca tsmin (raiomin).
%       ==> O lugar das raizes deve ser atraido pra local de raio menor.

%% Projeto direto: COMPENSADOR DINAMICO (tentativa 1)

[Zg,Pg,Kg] = zpkdata(Gz);
Dz = tf([1 -Pg{1,1}(2)], [1 -0.01*raiomin], T)

figure; rlocus(Gz*Dz);
zgrid(zetamin,[]); hold on; plotcircle(0,0,raiomin,'k-.');
axis([-1.1 1.1 -1.1 1.1]);
[Kc, raizes] = rlocfind(Gz*Dz) 

Tz = Kc*Gz*Dz/(1+Kc*Gz*Dz) % Tz = Y(z)/R(z) tem como saida a PV
Tuz = Kc*Dz/(1+Kc*Gz*Dz) % Tuz = U(z)/R(z) tem como saida a MV

% Avaliacao de desempenho:
% Constante de velocidade:
Kv = dcgain(Kc*Gz*Dz*tf([1 -1],[T 0],T))

% Respostas ao degrau e a rampa:
figure(5); hold on; step(Tz);
kT = [0:T:(30)]; 
rk = 0.01*kT;
figure(6); hold on; lsimplot(Tz,rk,kT); grid;
figure(7); hold on; uk = lsim(Tuz,rk,kT); stairs(kT, uk, 'b');

%% Projeto direto: COMPENSADOR DINAMICO (tentativa 2)

[Zg,Pg,Kg] = zpkdata(Gz);
% Dz = tf([1 -0.9*Pg{1,1}(2)], [1 -0.05*raiomin], T)
Dz = tf([1 -0.88], [1 0.5],T)

figure; rlocus(Gz*Dz);
zgrid(zetamin,[]); hold on; plotcircle(0,0,raiomin,'k-.');
axis([-1.1 1.1 -1.1 1.1]);
[Kc, raizes] = rlocfind(Gz*Dz) 
% OBS.: Escolha Kc = 13

Tz = zpk(minreal(Kc*Gz*Dz/(1+Kc*Gz*Dz))) % Tz = Y(z)/R(z) tem como saida a PV
Tuz = zpk(minreal(Kc*Dz/(1+Kc*Gz*Dz))) % Tuz = U(z)/R(z) tem como saida a MV

% Avaliacao de desempenho:
% Constante de velocidade:
Kv = dcgain(Kc*Gz*Dz*tf([1 -1],[T 0],T))

% Respostas ao degrau e a rampa:
figure(5); hold on; step(Tz); grid;
kT = [0:T:(30)]; 
rk = 0.01*kT;
figure(6); hold on; lsimplot(Tz,rk,kT); grid;
figure(7); hold on; uk = lsim(Tuz,rk,kT); stairs(kT, uk, 'r'); ylabel('u(k)'); xlabel('kT_s'); grid;
 