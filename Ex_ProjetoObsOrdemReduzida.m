% ELT013 - Controle Digital
% Projeto de observador de estados com ordem reduzida
%
% Prof. Bruno Teixeira, DELT/UFMG, 06/2017

%% Sistema: satelite duplo integrador
% Entrada: torque, Saida: posicao angular

close all; clc; clear all

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

%% Sistema reduzido: {Ar, Br, Cr, Dr}

% ATENCAO!!! Observe que, neste caso, xb = x1 e xa = x2 = y
Aaa = A(2,2); Aab = A(2,1); Aba = A(1,2); Abb = A(1,1);
Ba = B(2,1); Bb = B(1,1);
Ar = Abb
Cr = Aab
Dr = 0;

Obr = Cr;


%% Especificacoes de requisito:

% Polo desejado em z:
p1z = 0.3;
% Equacao caracteristica desejada:
alphacz = [1 -p1z]


%% Projeto de PREDITOR REDUZIDO pela Formula de Ackerman

% Usando funcao do Matlab:
Lr = acker(Ar', Cr', p1z)' 

% Passo a passo:
Lr = ( Ar + alphacz(2)*eye(size(Ar)) )*inv(Obr)*[zeros(size(Ar,1)-1,1);  1]

eig(Ar - Lr*Cr) % Conferindo polo do observador de ordem reduzida

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
% Equacao caracteristica desejada:
alphaccz = conv([1 -pc1z],[1 -pc2z]);

% Alocacao de polos (passo a passo da Formula de Ackermann):
K = [zeros(1,size(A,1)-1)  1]*inv(Co)*( A^2 + alphaccz(2)*A + alphaccz(3)*eye(size(A)) )

%% Simulacao:

x0 = [1 1]';
N = 20;
u = 0;
kT = [0:Ts:(N-1)*Ts];
xmf = [x0 x0]; % Estados verdadeiros
ymf = C*xmf;
x1hat = 0; % Estado estimado

for k = 2:N
    % Observador reduzido de estados:
    x1hat(:,k) = Ar*x1hat(:,k-1) + (Aba*ymf(:,k-1) + Bb*u(k-1))  +  Lr*((ymf(:,k) - Aaa*ymf(:,k-1) - Ba*u(k-1)) - Cr*x1hat(:,k-1));
    % Lei de controle:
    u(k) = -K*[x1hat(:,k) xmf(2,k)]'; % u(k) = -K*xmf(:,k); % (teste)
    % Processo controlado:
    xmf(:,k+1) = A*xmf(:,k) + B*u(k); % Observe que o modelo do mesmo processo usado nesse projeto nao e o mesmo dos outros casos
    ymf(:,k+1) = C*xmf(:,k+1);
end
figure; subplot(311); stairs(kT, xmf(1,1:end-1), 'b'); hold on; stairs(kT, x1hat(1,:), 'r');  xlabel('kT (s)'); ylabel('x_1(k)'); legend('x_1(k)','xhat(k)')
        subplot(312); stairs(kT, xmf(2,1:end-1), 'b'); hold on; xlabel('kT (s)'); ylabel('x_2(k)');
        subplot(313); stairs(kT, u, 'r'); hold on; xlabel('kT (s)'); ylabel('u(k)');
        
