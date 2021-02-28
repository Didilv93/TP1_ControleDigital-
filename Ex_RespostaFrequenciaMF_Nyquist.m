% Controle Digital
% Analise de estabilidade relativa de sistemas em MF por criterio de Nyquist:
%
% Bruno Teixeira, DELT/UFMG, 2/5/2017

close all; clear all; clc;

% FT em MA analogico: G(jw)
num = [1];
den = conv([1 0],[1 1]);
Gmas = tf(num, den)

% Margem de ganho e de fase do sistema analogico:
[Gm, Pm, Wcg, Wcp] = margin(Gmas)

% FT em MA amostrado: G(e^jw)
T = 0.1;
Gmad = c2d(Gmas, T, 'zoh')
[numz, denz, T] = tfdata(Gmad,'v');
roots(numz), roots(denz)

% Margem de ganho e de fase do sistema amostrado:
[Gm, Pm, Wcg, Wcp] = margin(Gmad)

% Diagramas de Nyquist:
figure; 
nyquist(Gmas); hold on; plotcircle(0,0,1,'k')
nyquist(Gmad); axis([-1.6 0 -1 1])