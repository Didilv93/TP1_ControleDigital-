% Resposta em frequencia de sistemas a tempo discreto
% OBS.: Execute esse arquivo no modo "Run and Advance"
%
% Bruno Teixeira, DELT/UFMG, 18/04/2017

close all; clear all; clc;

%% Sistema 1: POLO SIMPLES
numGHs = 1;
denGHs = [10 1];
sysGHs = tf(numGHs,denGHs)
T = 5;
sysGHz = c2d(sysGHs, T, 'zoh')
[numGHz, denGHz] = tfdata(sysGHz,'v');

bode(numGHs,denGHs); hold on;
dbode(numGHz, denGHz, T); legend('continuo','discreto');
figure; subplot(211); nyquist(sysGHs); hold on; nyquist(sysGHz); legend('continuo','discreto');
subplot(212); nichols(sysGHs); hold on; nichols(sysGHz); legend('continuo','discreto');

% Observe que, para o sistema cont?nuo, a fase converge para -90,
% ao passo que, para o caso discreto, a fase converge para -180

%% Sistema 2: POLO SIMPLES E ZERO SIMPLES
close all;
numGHs = [1 1];
denGHs = [10 1];
sysGHs = tf(numGHs,denGHs)
T = 3;
sysGHz = c2d(sysGHs, T, 'zoh')
[numGHz, denGHz] = tfdata(sysGHz,'v');

figure;
bode(numGHs,denGHs); hold on;
dbode(numGHz, denGHz, T); legend('continuo','discreto');
figure; subplot(211); nyquist(sysGHs); hold on; nyquist(sysGHz); legend('continuo','discreto');
subplot(212); nichols(sysGHs); hold on; nichols(sysGHz); legend('continuo','discreto');

%% Sistema 3: INTEGRADOR + POLO SIMPLES
close all;
numGHs = [0 1];
denGHs = conv([1 0],[10 1]);
sysGHs = tf(numGHs,denGHs)
T = 1;
sysGHz = c2d(sysGHs, T, 'zoh')
[numGHz, denGHz] = tfdata(sysGHz,'v');

figure;
bode(numGHs,denGHs); hold on;
dbode(numGHz, denGHz, T); legend('continuo','discreto');
figure; subplot(211); nyquist(sysGHs); hold on; nyquist(sysGHz); legend('continuo','discreto');
subplot(212); nichols(sysGHs); hold on; nichols(sysGHz); legend('continuo','discreto');

%% Sistema 4: INTEGRADOR DUPLO + POLO SIMPLES + ZERO SIMPLES
close all;
numGHs = [10 1];
denGHs = conv([1 0],conv([1 0],[100 1]));
sysGHs = tf(numGHs,denGHs)
T = 1;
sysGHz = c2d(sysGHs, T, 'zoh')
[numGHz, denGHz] = tfdata(sysGHz,'v');

figure;
bode(numGHs,denGHs); hold on;
dbode(numGHz, denGHz, T); legend('continuo','discreto');
figure; subplot(211); nyquist(sysGHs); hold on; nyquist(sysGHz); legend('continuo','discreto');
subplot(212); nichols(sysGHs); hold on; nichols(sysGHz); legend('continuo','discreto');

%% Sistema 5: POLOS COMPLEXOS + ZERO SIMPLES
close all;

numGHs = [1/0.1 1];
denGHs = [1/0.0001 2*0.1/0.01 1];
sysGHs = tf(numGHs,denGHs)
T = 1;
sysGHz = c2d(sysGHs, T, 'zoh')
[numGHz, denGHz] = tfdata(sysGHz,'v');

figure;
bode(numGHs,denGHs); hold on;
dbode(numGHz, denGHz, T); legend('continuo','discreto');
figure; subplot(211); nyquist(sysGHs); hold on; nyquist(sysGHz); legend('continuo','discreto');
subplot(212); nichols(sysGHs); hold on; nichols(sysGHz); legend('continuo','discreto');

pause;
close all;
