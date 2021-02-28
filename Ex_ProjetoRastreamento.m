% ELT013 - Controle Digital
% Projeto de controlador no espaco de estados: rastreamento
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

% Observe que a saida y(k) = x2(k).
% Assuma que desejamos rastrear yr(k) = x2(k) tal que:
Cr = C;
% Cr = [0.5 0.9];

%% Projeto do OBSERVADOR:

% Teste de observabilidade:

% Usando funcao do Matlab:
Ob = obsv(A,C);
% Passo a passo:
Ob = [C; C*A];
if (size(A,1)-rank(Ob)==0)
    disp('O sistema e observavel.');
end

% Especificacoes de requisito:
% Polos desejados em z:
p1z = 0.4 + i*0.4;
p2z = 0.4 - i*0.4;
% Equacao caracteristica desejada:
alphacz = conv([1 -p1z],[1 -p2z])


% Projeto de PREDITOR pela Formula de Ackerman

% Usando funcao do Matlab:
L = acker(A', C', [p1z p2z])' 
% Passo a passo:
L = ( A^2 + alphacz(2)*A + alphacz(3)*eye(size(A)) )*inv(Ob)*[zeros(size(A,1)-1,1);  1]

% Projeto de FILTRO pela Formula de Ackerman

% Usando funcao do Matlab:
% Obs.: Basta substituir C por C*A
G = acker(A', (C*A)', [p1z p2z])' 
% Passo a passo:
% Obs.: Basta multiplicar a matriz de observabilidade por A
G = ( A^2 + alphacz(2)*A + alphacz(3)*eye(size(A)) )*inv(Ob*A)*[zeros(size(A,1)-1,1);  1]


%% Projeto do CONTROLADOR:

% Teste de Controlabilidade
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
alphaccz = conv([1 -pc1z],[1 -pc2z])

% Alocacao de polos (passo a passo da Formula de Ackermann):
K = [zeros(1,size(A,1)-1)  1]*inv(Co)*( A^2 + alphaccz(2)*A + alphaccz(3)*eye(size(A)) )

%% Obtencao dos Ganhos de Referencia:
Nn = inv([A-eye(size(A)), B; Cr, 0])*([zeros(2,1); 1])
Nx = Nn(1:2)
Nr = Nn(3) % Observe que Nr = 0 pois sistema e tipo 2

%% Simulacao:
% 1 - caso com observador do tipo preditor
% 2 - caso com observador do tipo filtro

x0 = [5 5]';
N = 50;
u1 = 0;
u2 = 0;
kT = [0:Ts:(N-1)*Ts];
xmf1 = [x0 x0]; % Estado verdadeiro
ymf1 = C*x0;
xhat1 = [0 0]'; % Estado estimado
xmf2 = [x0 x0]; % Estado verdadeiro
ymf2 = [C*x0, C*x0];
xhat2 = [0 0]';

% Ruido:
w = 0.05*randn(2,N+1); % de processo
v = 0.5*randn(1,N+1); % de observacao

% Referencia:
r = [4*ones(1,N/2), -4*ones(1,N/2)];

for k = 2:N
    % ------------------- CASO 1: Preditor + regulador --------------------
    % Observador de estados:
    xhat1(:,k) = A*xhat1(:,k-1) + B*u1(k-1)  +  L*(ymf1(:,k-1) - C*xhat1(:,k-1));
    % Lei de controle:
    u1(k) = -K*(xhat1(:,k) - Nx*r(k)) + Nr*r(k);
    % Processo controlado:
    xmf1(:,k+1) = A*xmf1(:,k) + B*u1(k) +  w(:,k);
    ymf1(:,k+1) = C*xmf1(:,k+1) + v(k+1);
    % ------------------- CASO 2: Filtro + regulador ----------------------
    % Observador de estados:
    xhat2k_ = A*xhat2(:,k-1) + B*u2(k-1);
    yhat2k_ = C*xhat2k_;
    xhat2(:,k) = xhat2k_  +  G*(ymf2(:,k) - yhat2k_);
    % Lei de controle:
    u2(k) = -K*(xhat2(:,k) - Nx*r(k)) + Nr*r(k);
    % Processo controlado:
    xmf2(:,k+1) = A*xmf2(:,k) + B*u2(k) + w(:,k);
    ymf2(:,k+1) = C*xmf2(:,k+1) + v(k+1);
end
figure; subplot(311); stairs(kT, xmf1(1,1:end-1), 'b'); hold on; stairs(kT, xhat1(1,:), 'r'); stairs(kT, xhat2(1,:), 'm'); xlabel('kT (s)'); ylabel('x_1(k)'); grid on; legend('x_1(k)','xhat_1(k)','xhat_2(k)')
        subplot(312); stairs(kT, xmf1(2,1:end-1), 'b'); hold on; stairs(kT, xhat1(2,:), 'r'); stairs(kT, xhat2(2,:), 'm'); stairs(kT, r, 'k'); xlabel('kT (s)'); ylabel('x_2(k)'); grid on;
        subplot(313); stairs(kT, u1, 'r'); hold on; stairs(kT, u2, 'm'); xlabel('kT (s)'); ylabel('u(k)'); grid on;
        
