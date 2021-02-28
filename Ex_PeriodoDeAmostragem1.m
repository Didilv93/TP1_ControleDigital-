%--------------------------------------------------------------------------
% Efeito da Taxa de Amostragem no Desempenho em Malha Fechada
%--------------------------------------------------------------------------
% Modificado de:
% Figure 3.2      Digital Control of Dynamic Systems, 3e 
%                 Franklin, Powell, Workman
%                 Addison-Wesley Publishing Company, 1998
% Computes the continuous time response and the digital time response 
% assuming the use of Euler's forward rectangular rule
%--------------------------------------------------------------------------

clf
clear

% plant definition
numG=1;
denG=[1 0 0];

% continuous compensation
Ko=70;
a=2;            % D(s) zero
b=10;           % D(s) pole
numD=Ko*[1 a];
denD=[1 b];

num=conv(numG,numD);
den=conv(denG,denD);

% analog closed loop
[numcl,dencl]=feedback(num,den,1,1);
tf=1;
t=0:.01:tf;
yc=step(numcl,dencl,t);
subplot(2,1,1)
plot(t,yc,'-'),grid
axis([0 1 0 1.5])
hold on

% now do the digital case

fs= 20;  % Hz
T=1/fs;
[numGd,denGd]=c2dm(numG,denG,T,'zoh'); % adding the effect of D/A converter to the plant
numDd=Ko*[1 -(1-a*T)]; % discretizing the compensator
denDd=[1 -(1-b*T)];
% Obs.: Ha diferentes maneiras de se discretizar um controlador analogico
numd=conv(numGd,numDd);
dend=conv(denGd,denDd);
[numcld,dencld]=feedback(numd,dend,1,1); % digital closed loop at fs = 20Hz

N=tf*fs;
yd=dstep(numcld,dencld,N);
td=0:T:(N-1)*T;
plot(td,yd,'*')
plot(td,yd,'-')
ylabel('output y')
title('Continuous and digital response using Eulers method')
text(.25,.1,'*-----*-----* digital control')
text(.25,.3,'------------- analog control')
text(.35,.6,'  (a)  20 Hz')


hold off

Ws= 40;  % Hz
T=1/Ws;
[numGd,denGd]=c2dm(numG,denG,T,'zoh');
numDd=Ko*[1 -(1-a*T)];
denDd=[1 -(1-b*T)];
numd=conv(numGd,numDd);
dend=conv(denGd,denDd);
[numcld,dencld]=feedback(numd,dend,1,1);

N=tf*Ws;
subplot(2,1,2)
plot(t,yc,'-'),grid
hold on
yd=dstep(numcld,dencld,N);
td=0:T:(N-1)*T;
plot(td,yd,'*')
plot(td,yd,'-')
xlabel('time (sec)')
ylabel('output y')
text(.25,.1,'*-----*-----* digital control')
text(.25,.3,'------------- analog control')
text(.35,.6,'  (a)  40 Hz')

hold off
