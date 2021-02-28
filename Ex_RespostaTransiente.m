% ELT013 - Controle Digital
%
% Efeito da amostragem na resposta transiente
%
% Prof. Bruno Teixeira, DELT/UFMG, 01/09/2017

%% Planta analogica:
numGs = 1;
denGs = [1 1 0];
Gs = tf(numGs, denGs);
Gs = zpk(Gs) % Forma fatorada da FT

%% Planta "amostrada":
T1 = 1;
Gz1 = c2d(Gs, T1, 'zoh')

T2 = 0.1;
Gz2 = c2d(Gs, T2, 'zoh')

%% Malha fechada:
Ts = minreal(Gs/(1+Gs))
Tz1 = minreal(Gz1/(1+Gz1))
Tz2 = minreal(Gz2/(1+Gz2))

[nz1,dz1] = tfdata(Tz1,'v'); y1 = dstep(nz1,dz1); 
[nz2,dz2] = tfdata(Tz2,'v'); y2 = dstep(nz2,dz2);

figure;
step(Ts,'k'); hold on; 
stairs([0:T1:T1*(length(y1)-1)]', y1, 'b');
stairs([0:T2:T2*(length(y2)-1)]', y2, 'r');
hold on; axis([0 12 -0.01 1.6]); xlabel('kT_s (s)'); ylabel('y(k)');