% ELT013 - Controle Digital
%
% Exemplo de projeto de controlador por emulacao:
% Controle de azimute de uma antena para captacao de sinais de satelite
%
% Prof. Bruno Teixeira, DELT/UFMG, 11/05/2017
% (baseado nas notas de aula do Prof. Reinaldo Palhares)

close all; clear all; clc;

%% PROJETO ANALOGICO:

%% Planta analogica:
% Obs: Assume-se que antena tenha ganho igual a 10.
tau = 10; % J/B (momento de inercia/atrito viscoso)
numGs = 1;
denGs = [tau 1 0];
Gs = tf(numGs, denGs);
Gs = zpk(Gs) % Forma fatorada da FT

%% Especificacoes de desempenho:

% Maxima sobre-elevacao
Mpmax = 16;
% ===> zeta >= 0.6*(1 - Mpmax/100)
zetamin = 0.6*(1 - Mpmax/100)

% Tempo de acomodacao maximo
tsmax = 10;
% ===> zeta*wn >= 4,6 / tsmax:
zetawnmin = 4.6/tsmax

% Tempo de amostragem T <= tr / 10, em que tr e o tempo de subida
% ===> T <= 1,8/(10*wn)
% ===> T será definido adiante

% Rastreamento de rampa y = 0,01t rad/s com ess <= 0,01 rad
% Se ess = 0,01*T/K_v <= 0,01 ===> Kv >= T


%% Controlador analogico:

% Lugar das raizes para compensador estatico:
rlocus(Gs); hold on;
plot([-zetawnmin -zetawnmin], [-0.05 0.05], 'k--'); 
plot(linspace(-2*zetawnmin,0,100),tan(pi-acos(zetamin)).*linspace(-2*zetawnmin,0,100),'k-.')
axis([-2*zetawnmin 0.02 -0.05 0.05])

pause;

% Compensador avanco:
zs = -0.1;
ps = -1.0;
Ds = tf([1 -zs], [1  -ps])

% Qual seria o ganho Kc do compensador?
% Note que G(s)*D(s)/(1+G(s)*D(s)) = 0,1*Kc/(s^2 + s + 0,1*K) = wn^2/(s^2+2*zeta*wn*s+wn^2)
%
% Considerando zetawnmin acima, escolhemos zeta*wn = 0,5
%
% Considerando zetamin acima, zetamin*wnmax = 0,5 ==> wn <= 0,5/zetamin
wnmax = 0.5/zetamin
% Para wn = wnmax ==> Kc >= wnmax^2/0,1:
Kc = wnmax^2/0.1

figure;
K = [0:0.001:20]; % Escolhendo valores de K pros quais o lugar das raizes sera tracado
rlocus(Gs*Ds, K); hold on;
plot([-zetawnmin -zetawnmin], [-1.25 1.25], 'k--'); 
plot(linspace(-2*zetawnmin,0,100),tan(pi-acos(zetamin)).*linspace(-2*zetawnmin,0,100),'k-.')
% axis([-2*zetawnmin 0.02 -1.25 1.25])

[Kc, raizes] = rlocfind(Gs*Ds) 

% OBS1: Avalie o efeito da variacao do ganho do compensador Kc 

% Pelo lugar das raizes, Kc > 2.5, garante que as raizes satisfazem aos
% criterios de Mpmax e tsmax. No entanto, o erro esta maior que o desejado
% Kc = 2.5;

% Para Kc proximo de 10, consegue-se aumentar Kv e reduzir o ess
% Kc = 10;  ===> wn = 1;
wn = abs(raizes(1)); % Usando a relação de que, para polos complexos conjugados, wn equivale ao modulo desses polos
tr = 1.8/wn
T = tr/10; % Tempo de amostragem
T = 0.2 % Arredondando

% OBS2: Avalie o efeito de aumento e reducao de T 

Ts = Gs*(Kc*Ds)/(1 + Gs*(Kc*Ds)); % Ts = Y(s)/R(s) tem como saida a PV
Tus = (Kc*Ds)/(1+(Gs*Kc*Ds)) % Tus = U(s)/R(s) tem como saida a MV
Ts = zpk(minreal(Ts))
Tus = zpk(minreal(Tus))
figure(5); step(Ts); title('Resposta ao degrau da MF'); grid on;
t = [0:0.001:30];
rt = 0.01*t;
figure(6); lsimplot(Ts,rt,t); grid on; title('Resposta a rampa da MF');
figure(7); ut = lsim(Tus,rt,t); plot(t, ut, 'b');

%sisotool(Gs*Ds)

%% Controlador digital:

Gz = c2d(Gs, T, 'zoh') % A planta *sempre* deve ser discretizada por esse metodo!
Dz = c2d(Kc*Ds, T, 'matched')
Tz = zpk(minreal(Gz*Dz/(1+Gz*Dz))) % Tz = Y(z)/R(z) tem como saida a PV
Tuz = zpk(minreal(Dz/(1+Gz*Dz))) % Tuz = U(z)/R(z) tem como saida a MV

% Dz.InputName = 'e';  Dz.OutputName = 'u';
% Gz.InputName = 'u';  Gz.OutputName = 'y';
% Sum = sumblk('e = r-y');
% Tz = connect(Gz,Dz,Sum,'r','y');

%% Avaliacao de desempenho:

% Margens de ganho e fase:
[Gm, Pm, Wcg, Wcp] = margin(Kc*Gs*Ds)
[Gm, Pm, Wcg, Wcp] = margin(Gz*Dz)

% Respostas ao degrau e a rampa:
figure(5); hold on; step(Tz);
kT = [0:T:(30)]; 
rk = 0.01*kT;

figure(6); hold on; lsimplot(Tz,rk,kT);
figure(7); hold on; uk = lsim(Tuz,rk,kT); stairs(kT, uk, 'r');

% Resposta em frequencia da MA:
figure; nyquist(Kc*Gs*Ds); hold on; nyquist(Gz*Dz); legend('Analogico','Digital');
plotcircle(0,0,1,'k-.'); axis([-5 0.5 -2 2]);

figure; bode(Kc*Ds*Gs); hold on; bode(Dz*Gz); legend('Analogico','Digital')

% Resposta em frequencia da MF
figure; bode(Ts); hold on; bode(Tz); legend('Analogico','Digital')