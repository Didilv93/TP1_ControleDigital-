% ELT013 - Controle Digital
% Projeto de regulador
%
% Prof. Bruno Teixeira, DELT/UFMG, 05/2017

%% Sistema: satelite duplo integrador
% Entrada: torque, Saida: posicao angular

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

%% Teste de Controlabilidade:

% Usando funcao do Matlab:
Co = ctrb(A,B);
% Passo a passo:
Co = [B A*B];
if (size(A,1)-rank(Co)==0)
    disp('O sistema e controlavel.');
end

%% Especificacoes de requisito:

zeta = 0.5;
zetawn = 1.8;
wn = zetawn/zeta;
% Polos desejados em s:
p1s = -zetawn + i*wn*sqrt(1-zeta^2);
p2s = -zetawn - i*wn*sqrt(1-zeta^2);
% Polos desejados em z:
p1z = exp(p1s*Ts)
p2z = exp(p2s*Ts)

% Equacao caracteristica desejada:
alphacz = conv([1 -p1z],[1 -p2z])

%% Projeto pelo Metodo 1 - Identidade Polinomial

syms z k1 k2
K = [k1 k2];

ECmf = z*eye(size(A)) - A + B*K
% A partir da inspecao da expressao acima, temos:
alpha = [1    (-2 + Ts*k1 + (k2*((Ts^2)/2)))    (1 - Ts*k1 + (k2*((Ts^2)/2)))]

K1 = (inv([Ts Ts^2/2; -Ts  Ts^2/2])*[(2+alphacz(2))  (-1+alphacz(3))]')' 

% Conferindo os polos da MF:
eig(A - B*K1)

%% Projeto pelo Metodo 2 - Formula Canonica de Controle

% Forma canonica de controle:
[A2, B2, C2, D2] = tf2ss(nGz, dGz)

K2 = [alphacz(2)-dGz(2)     alphacz(3)-dGz(3)]

% Conferindo os polos da MF:
eig(A2 - B2*K2)


%% Projeto pelo Metodo 3 - Formula de Ackerman

% Usando funcao do Matlab:
K3 = acker(A, B, [p1z p2z]) 

% Passo a passo:
K3 = [zeros(1,size(A,1)-1)  1]*inv(Co)*( A^2 + alphacz(2)*A + alphacz(3)*eye(size(A)) )

%% Simulacao:

% Malha aberta (resposta natural apenas):
x0 = [1 1]';
N = 100;
xma(:,1) = x0; yma = C*xma;
u = 0;
kT = [0:Ts:(N-1)*Ts];
for k = 2:N
    xma(:,k) = A*xma(:,k-1) + B*u;
    yma(:,k) = C*xma(:,k);
end
figure; subplot(211); stairs(kT, xma(1,:), 'k'); xlabel('kT (s)'); ylabel('x_1(k)');
        subplot(212); stairs(kT, xma(2,:), 'k'); xlabel('kT (s)'); ylabel('x_2(k)');

% Malha fechada (metodo 2):
xmf2 = x0;
for k = 2:N
    % Lei de controle:
    u2(k-1) = -K2*xmf2(:,k-1);
    % Processo controlado:
    xmf2(:,k) = A2*xmf2(:,k-1) + B2*u2(k-1); % Observe que o modelo do mesmo processo usado nesse projeto nao e o mesmo dos outros casos
    ymf2(:,k) = C2*xmf2(:,k);
end
figure; subplot(311); stairs(kT, xmf2(1,:), 'b'); xlabel('kT (s)'); ylabel('x_1(k)');
        subplot(312); stairs(kT, xmf2(2,:), 'b'); xlabel('kT (s)'); ylabel('x_2(k)');
        subplot(313); stairs(kT(1:end-1), u2, 'b'); xlabel('kT (s)'); ylabel('u(k)');
        
% Malha fechada (metodo 3):
xmf3 = x0;
for k = 2:N
    % Lei de controle:
    u3(k-1) = -K3*xmf3(:,k-1);
    % Processo controlado:
    xmf3(:,k) = A*xmf3(:,k-1) + B*u3(k-1);
    ymf3(:,k) = C*xmf3(:,k);
end
subplot(311); hold on; stairs(kT, xmf3(1,:), 'r'); xlabel('kT (s)'); ylabel('x_1(k)');
subplot(312); hold on; stairs(kT, xmf3(2,:), 'r'); xlabel('kT (s)'); ylabel('x_2(k)');
subplot(313); hold on; stairs(kT(1:end-1), u3, 'r'); xlabel('kT (s)'); ylabel('u(k)');