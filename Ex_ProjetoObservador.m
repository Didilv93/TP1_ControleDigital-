% ELT013 - Controle Digital
% Projeto de observador de estados
%
% Prof. Bruno Teixeira, DELT/UFMG, 06/2017

%% Sistema: satelite duplo integrador
% Entrada: torque, Saida: posicao angular

close all; clc; clear all; clc

% Funcao de transferencia em Laplace
nGs = 1;
dGs = [1 0 0];
Gs = tf(nGs, dGs)
% Espaco de estados:
[Ac, Bc, Cc, Dc] = tf2ss(nGs, dGs);
SysS = ss(Ac, Bc, Cc, Dc)

% Discretizacao do modelo em tempo continuo:
Ts = 0.1;
SysZ = c2d(SysS, Ts, 'zoh')
[A, B, C, D, Ts] = ssdata(SysZ)
[nGz, dGz] = ss2tf(A, B, C, D);
Gz = zpk(minreal(tf(nGz, dGz, Ts)))

%% Teste de Observabilidade:

% Usando funcao do Matlab:
Ob = obsv(A,C);
% Passo a passo:
Ob = [C; C*A];
if (size(A,1)-rank(Ob)==0)
    disp('O sistema e observavel.');
end

%% Especificacoes de requisito:

% Autovalores desejados em z:
p1z = 0.4 + i*0.4;
p2z = 0.4 - i*0.4;

% Equacao caracteristica desejada:
alphacz = conv([1 -p1z],[1 -p2z])

%% Projeto pelo Metodo 1 - Identidade Polinomial

syms z l1 l2
L = [l1 l2]';

ECmf = z*eye(size(A)) - A + L*C
det(ECmf)
% A partir da inspecao visual da expressao (det(ECmf) e ordenar) acima, temos:
alpha = [1    (l2-2)    (1 - l2 + Ts*l1)]

% Comparar alpha com alphacz e montar sistema de 2a ordem para encontrar l1
% e l2:
L1 = (inv([0 1; Ts -1])*[(2+alphacz(2))  (-1+alphacz(3))]') 

% Conferindo os polos da MF:
eig(A - L1*C)

%% Projeto pelo Metodo 2 - Formula Canonica de Observacao

% Forma canonica observavel:
A2 = [-dGz(2) 1; -dGz(3) 0];
B2 = [nGz(2) nGz(3)]';
C2 = [1 0];
Sys2 = ss(A2, B2, C2, 0, Ts)

L2 = [alphacz(2)-dGz(2)     alphacz(3)-dGz(3)]'

% Conferindo os polos da MF:
eig(A2 - L2*C2)


%% Projeto pelo Metodo 3 - Formula de Ackerman

% Usando funcao do Matlab:
L3 = acker(A', C', [p1z p2z])' 

% Alternativamente: passo a passo:
L3 = ( alphacz(1)*A^2 + alphacz(2)*A + alphacz(3)*eye(size(A)) )*inv(Ob)*[zeros(size(A,1)-1,1);  1]


%% Projeto do Controlador:
% Retirado de Ex_ProjetoRegulador.m

% Controlabilidade
Co = [B A*B];
if (size(A,1)-rank(Co)==0)
    disp('O sistema e controlavel.');
end

% Requisitos do controlador:
zeta = 0.5;
zetawn = 1.8;
wn = zetawn/zeta;
% Polos desejados em s:
p1s = -zetawn + i*wn*sqrt(1-zeta^2);
p2s = -zetawn - i*wn*sqrt(1-zeta^2);
% Polos desejados em z:
pc1z = exp(p1s*Ts)
pc2z = exp(p2s*Ts)
% Arredondando:
pc1z = 0.8 + i*0.25;
pc2z = 0.8 - i*0.25;

% Equacao caracteristica desejada:
alphacontz = conv([1 -p1z],[1 -p2z])

% Alocacao de polos:
%K3 = [zeros(1,size(A,1)-1)  1]*inv(Co)*( A^2 + alphacontz(2)*A + alphacontz(3)*eye(size(A)) )
K3 = acker(A, B, [pc1z pc2z]) 


%% Simulacao:

x0 = [1 1]';
N = 30;
u = 0;
kT = [0:Ts:(N-1)*Ts];
xmf = [x0 x0]; % Estado verdadeiro
ymf = C*x0;
xhat = [10 10]'; % Estado estimado
for k = 2:N
    % Observador de estados:
    xhat(:,k) = A*xhat(:,k-1) + B*u(k-1)  +  L3*(ymf(:,k-1) - C*xhat(:,k-1));
    % Lei de controle:
    u(k) = -K3*xhat(:,k);
    % Processo controlado (simulacao):
    xmf(:,k+1) = A*xmf(:,k) + B*u(k); % Observe que o modelo do mesmo processo usado nesse projeto nao e o mesmo dos outros casos
    ymf(:,k+1) = C*xmf(:,k+1);
end
figure; subplot(311); stairs(kT, xmf(1,1:end-1), 'b'); hold on; stairs(kT, xhat(1,:), 'r'); xlabel('kT (s)'); ylabel('x_1(k)'); legend('x_1(k)','xhat_1(k)')
        subplot(312); stairs(kT, xmf(2,1:end-1), 'b'); hold on; stairs(kT, xhat(2,:), 'r'); xlabel('kT (s)'); ylabel('x_2(k)');
        subplot(313); stairs(kT, u, 'b'); xlabel('kT (s)'); ylabel('u(k)');
        
