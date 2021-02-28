%--------------------------------------------------------------------------
%  Efeito da Taxa de Amostragem no Desempenho em Malha Fechada
%--------------------------------------------------------------------------
%  Modificado de:
%  Figure 3.7       Digital Control of Dynamic Systems, 3e 
%                   Franklin, Powell, Workman
%                   Addison-Wesley Publishing Company, 1998
%--------------------------------------------------------------------------

clf
clear

% Micro servomotor:
numG = 360000;
denG = conv([1 60],[1 600]);

% Controlador PID:
Kp = 5;
Td = .0008;
Ti = .003;

numD=Kp*[Ti*Td Ti 1];
denD=[Ti 0];
[numCL,denCL]=feedback(conv(numG,numD),conv(denG,denD),1,1);
t=0:.0002:.01;
yc=step(numCL,denCL,t);

%--------------------------------------------------------------------------
% Escolha do periodo de amostragem:

% Resposta em frequencia do sistema em malha aberta e malha fechada:
bode(numG,denG); hold on; bode(numCL,denCL); legend('open loop','closed loop')

% Observe que: 
% Em MA, wB = 60rad/s (frequencia da banda de passagem)
% Em MF, wBcl = 2000rad/s

% Para 6 < ws/wBcl < 25:
% Escolha o fator de 10:
T = 2*pi/(10*2000)
% Arredondando:
T = 0.0003;

%--------------------------------------------------------------------------
% Implementacao analogica:

figure;
plot(t*1000,yc,'-');
grid;
hold on
title('Step response of a micro-motor');
xlabel('time (ms)');
ylabel('speed (rad/s)');

%--------------------------------------------------------------------------
% Implementacao digital:

% Efeito do conversor A/D no processo:
[numGd,denGd]=c2dm(numG,denG,T,'zoh');

% PID digital: 
a=1+T/Ti+Td/T
b=1+2*Td/T
c=Td/T
numDd=Kp*[a -b c];
denDd=[1 -1 0];

[numCLd,denCLd]=feedback(conv(numGd,numDd),conv(denGd,denDd),1,1); % Malha fechada

td=0:T:.01;
N=length(td);
yd1=dstep(numCLd,denCLd,N);

plot(td*1000,yd1,'-');
plot(td*1000,yd1,'*');

% Avaliando efeito da *reducao* do periodo de amostragem:
% Obs.: esse valor e aprox.: ws = 30wBcl 
T = 0.0001; 

[numGd,denGd]=c2dm(numG,denG,T,'zoh'); % Efeito do conversor A/D no processo
a=1+T/Ti+Td/T % PID digital 
b=1+2*Td/T
c=Td/T
numDd=Kp*[a -b c];
denDd= [1 -1 0];
[numCLd,denCLd]=feedback(conv(numGd,numDd),conv(denGd,denDd),1,1); % Malha fechada

td=0:T:.01;
N=length(td);
yd2=dstep(numCLd,denCLd,N);

plot(td*1000,yd2,'-');
plot(td*1000,yd2,'o');

text(4.1,.6,'--------- continuous')
text(4.1,.4,'-*--*--*- digital, T = 0.3 msec')
text(4.1,.2,'-o--o--o- digital, T = 0.1 msec')

hold off