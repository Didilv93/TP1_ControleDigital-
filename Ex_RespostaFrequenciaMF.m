% Controle Digital
% Analise de estabilidade de sistemas em MF
%
% Bruno Teixeira, DELT/UFMG, 1/4/2014

close all; clear all; 

% FT em MA: G(e^jw)
num = [0.4 0.3];
den = conv([1 -1],[1 -0.4]);
T = 1;
Gma = tf(num, den, T)

% Diagrama de Bode de G(e^jw):
N = 1000;
w = logspace(-2, log10(pi/T), N); % 1000 valores de w entre 0,01 e pi/T
figure;
bode(Gma, w);
grid on;

% Margem de ganho e de fase do sistema:
[Gm, Pm, Wcg, Wcp] = margin(Gma);
Gm_dB = 20*log10(Gm);

% pause;

% % Diagrama de Bode de G(jw_w):
Gmaw = d2c(Gma,'tustin')
hold on;
bode(Gmaw, w);

% Diagrama polar da resposta em frequencia em MA
figure; 
nyquist(Gma);
axis([-3 1 -3 3]);

% pause; 

grid on; % Produz os circulos M, que permitem determinar resposta em frequencia da MF

% Carta de Nichols:
figure; nichols(Gma, w);

% pause;


ngrid; % produz as curvas de magnitude logaritmica que permitem determinar resposta em frequencia da MF
%pause;

% Lugar das raizes:
figure; 
rlocus(Gma);
% pause;

% Resposta em frequencia em MF (diagrama de Bode):
Tmf = feedback(Gma,1)
figure(1); hold on;
bode(Tmf, w);
legend('MA','MA aprox.','MF');

% pause;
% sisotool(Gma);