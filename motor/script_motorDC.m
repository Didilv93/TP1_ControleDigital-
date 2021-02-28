%Universidade Federal de Minas Gerais
%Escola de Engenharia
%Departamento de Engenharia Eletrônica
%Controle Digital

%Aluno: Italo José Dias
%Professor: Leonardo Mozelli

%Controle do Motor DC pelo método de conformação de malha

%Comandos Iniciais
clear
close all
clc

%Definições iniciais
%Dados do modelo
J = 0.01; %torque
b = 0.1; %coefieciente de atrito viscoso
K = 0.01; %Constante de proporcionalidade entre corrente de armadura e torque induzido no rotor
R = 1; %Resistência do circuito de armadura
L = 0.5; %Indutância do circuito de armadura proporcional aos enrolamentos das bobinas
ts = 0.05*4/10; %Tempo de amostragem

%Cálculo do controlador
%Modelo dinâmico da planta
Gs = tf([K],[(L*J) (R*J+L*b) (R*b+K^2)]); %Dinâmica do motor
Gz = zpk(c2d(Gs,ts,'zoh')); %Discretização do modelo do motor, segurador de ordem zero

%Cálculos Auxiliares
syms w z
Cz = 108*(1+0.4*w)*(1+0.2*w)/(w*(1+0.13*w));
Cz = collect(Cz,z)
w = (z-1)/ts;
Cz = 108*(1+0.4*w)*(1+0.2*w)/(w*(1+0.13*w));
Cz = collect(Cz,z)

%Controlador com os valores encontrados no sisotool
num = [21600 -39960 18468];
den = [325 -600 275];
Cz = zpk(tf(num,den,ts));

step(Gz*Cz/(1+Gz*Cz))
load('sisotool_motorDC.mat')

