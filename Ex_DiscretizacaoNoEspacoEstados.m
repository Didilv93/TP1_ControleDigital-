% ELT013 - Controle Digital
% Discretizacao no espaco de estados
%
% Prof. Bruno Teixeira, DELT/UFMG, 05/2017

%% Sistema: satelite duplo integrador
% Entrada: torque, Saida: posicao angular

close all; clc;

% Funcao de transferencia em Laplace
nGs = 1;
dGs = [1 0 0];
Gs = tf(nGs, dGs)

% Espaco de estados (forma canonica de controle):
[Ac, Bc, Cc, Dc] = tf2ss(nGs, dGs);
SysS = ss(Ac, Bc, Cc, Dc)

%% Discretizacao do modelo em tempo continuo:

Ts = 0.1;

% Forma 1: pela funcao do Matlab
SysZ = c2d(SysS, Ts, 'zoh')
[A, B, C, D, Ts] = ssdata(SysZ)

% Forma 2: analiticamente
% a) Aproximacao de primeira ordem:
A1 = eye(size(Ac)) + Ts*Ac;
B1 = Ts*(eye(size(Ac)) + (Ts/2)*Ac)*Bc;
SysZ1 = ss(A1, B1, Cc, Dc, Ts)

% b) Aproximacao de segunda ordem:
A2 = A1 + ((Ts^2)/2)*Ac^2;
B2 = B1 + Ts*( ((Ts^2)/6)*Ac^2 )*Bc;
SysZ2 = ss(A2, B2, Cc, Dc, Ts)

%% Obtendo funcao de transferencia em Z a partir do modelo discretizado no espaco de estados:

[nGz, dGz] = ss2tf(A, B, C, D);
Gz = zpk(minreal(tf(nGz, dGz, Ts)))

[nGz1, dGz1] = ss2tf(A1, B1, Cc, Dc);
Gz1 = zpk(minreal(tf(nGz1, dGz1, Ts)))

[nGz2, dGz2] = ss2tf(A2, B2, Cc, Dc);
Gz2 = zpk(minreal(tf(nGz2, dGz2, Ts)))