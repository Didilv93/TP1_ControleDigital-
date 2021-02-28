% Resposta em frequencia de sistemas a tempo discreto
%
% Bruno Teixeira, DELT/UFMG, 05/11/2012

close all; clear all;

% Exercicio 1:

% G(z):
T = 1;
num = [0 0.368 0.264];
den = [1 -1.368 0.368];

figure(1);
dbode(num, den, T); title('dbode: (0, ws/2]');
% Observe que, devido a periodicidade da resposta em frequencia de G(z), a
% funcao 'dbode' traca o grafico apenas de 0 a ws/2

freq = logspace(-1,2,10000);
[mag, phase, freq] = dbode(num, den, T, freq);
figure(2);
subplot(211); semilogx(freq, 20*log10(mag)); title('dbode: periodica'); xlabel('\omega (rad/s)'); ylabel('Mag (dB)');
subplot(212); semilogx(freq, wrapTo180(phase));         xlabel('\omega (rad/s)'); ylabel('Fase (graus)');

figure(3);
subplot(211); plot(freq, 20*log10(mag)); title('dbode: frequencia nao-logaritmica'); xlabel('\omega (rad/s)'); ylabel('Mag (dB)');
subplot(212); plot(freq, wrapTo180(phase));         xlabel('\omega (rad/s)'); ylabel('Fase (graus)');

% G(w)
sysw = d2c(tf(num, den, T), 'tustin');
[numw, denw] = tfdata(sysw,'v');
% % numw = -0.0381*conv([1 -2],[1 12.14]);
% % denw = conv([1 0],[1 0.924]);
figure(4);
%bode(sysw); title('bode: G(w)');
[magw, phasew, freqw] = bode(numw,denw,freq); 
subplot(211); semilogx(freq, 20*log10(mag));  hold on; semilogx(freq, 20*log10(magw), 'r'); xlabel('\omega, \omega_w'); ylabel('Mag (dB)'); grid on; legend('G(e^{j\omega})','G(j\omega_w)'); axis([0.1 100 -40 50])
subplot(212); semilogx(freq, wrapTo180(phase));  hold on; semilogx(freq, wrapTo180(phasew), 'r'); xlabel('\omega, \omega_w'); ylabel('Fase (graus)'); grid on; %axis([0.1 1000 -200 -80])


