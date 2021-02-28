%Universidade Federal de Minas Gerais
%Escola de Engenharia
%Departamento de Engenharia Eletr�nica
%Controle Digital

%Aluno: Italo Jos� Dias
%Professor: Leonardo Mozelli

%Controle do n�vel do tanque pelo m�todo do lugar das ra�zes

%Comandos Iniciais
clear
close all
clc

%Defini��es iniciais
%Dados do modelo
m = 1626;
Cr = 0.01;
Cd = 0.29;
rho = 1.23;
Area = 4;
vel_lin = 10;
ts = 0.01; %tempo de amostragem

drag_lin = Cd*rho*Area*vel_lin;
alfa = (drag_lin+Cr)/m;

% forma linearizada, em espaco de estados, em tempo continuo
A = [0 1; 0 -alfa];
B = [0;1/m];
C = [1 0]; 
D = 0;

%Fun��o de transferencia continua
syms s
Gs = collect(C*inv(s*eye(2)-A)*B,s);
num = [6.1501e-04];
den = [1 14.2780 0];
Gs = zpk(tf(num,den));

%Discretiza��o do modelo
Gz = zpk(c2d(Gs,ts,'zoh'));
%sisotool(Gz)
den = [5 -3];
num = 5000*[300 -260];