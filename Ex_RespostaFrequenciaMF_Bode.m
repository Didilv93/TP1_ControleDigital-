
% Controle Digital
% Analise de estabilidade relativa de sistemas em MF por diagrama de Bode
%
% Bruno Teixeira, DELT/UFMG, 2/5/2017

close all; clear all; clc;

% FT em MA: G(e^jw)
num = [0.4 0.3];
den = conv([1 -1],[1 -0.4]);
T = 0.1;
Gma = tf(num, den, T)

% Diagrama de Bode de G(e^jw):
N = 1000;
w = logspace(-2, log10(pi/T), N); % 1000 valores de w entre 0,01 e ws/2
figure; dbode(Gma, T, w); grid on;
% Margem de ganho e de fase do sistema:
[Gm, Pm, Wcg, Wcp] = margin(Gma)
Gm_dB = 20*log10(Gm)

% Diagrama de Bode de G(jw_w):
Gmaw = d2c(Gma,'tustin')
hold on; bode(Gmaw, w);

% Resposta em frequencia em MF:
Tmf = feedback(Gma,1)
hold on; dbode(Tmf, T, w); legend('MA','MA aprox.','MF');