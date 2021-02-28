% ELT013 - Controle Digital
% Transformacao de similaridade
%
% Prof. Bruno Teixeira, DELT/UFMG, 05/2017

% Controle de azimute de uma antena
% Apendice A.2 do Franklin

%% Funcao de transferencia:
a = 0.1; % B/J
nGs =  1;
dGs = [1/a 1 0];
Gs = zpk(tf(nGs, dGs))

Ts = 1;
Gz = zpk(c2d(Gs, Ts, 'zoh'))
[nGz, dGz] = tfdata(Gz,'v');

%% Espaco de estados
[A, B, C, D] = tf2ss(nGz, dGz);
SysZ = ss(A, B, C, D, Ts)

% Simulacao:
N = 200;
kT = [0:Ts:Ts*(N-1)]';
u = square(0.1*kT); % Torque do motor

x0 = [0 0]';
[y, x] = dlsim(A, B, C, D, u, x0);

figure(1); subplot(311); stairs(kT, x(:,1), 'bo-'); xlabel('kT'); ylabel('x_1(k)');
           subplot(312); stairs(kT, x(:,2), 'bo-'); xlabel('kT'); ylabel('x_2(k)');
           subplot(313); stairs(kT, y, 'bo-'); xlabel('kT'); ylabel('y(k)');
figure(2); plot(x(:,1), x(:,2), 'bo:'); xlabel('x_1(k)'); ylabel('x_2(k)');

%% Transformacao de similaridade:

% Caso 1: escolha arbitraria
P = [2 0.5; 0.3 1.5];
A1 = inv(P)*A*P;
B1 = inv(P)*B;
C1 = C*P;
D1 = D;
SysZ1 = ss(A1, B1, C1, D1, Ts)

[y1, x1] = dlsim(A1, B1, C1, D1, u, x0);

% Caso 2: forma canonica de Jordan
[V, R] = eig(A)
P = V;
A2 = inv(P)*A*P;
B2 = inv(P)*B;
C2 = C*P;
D2 = D;
SysZ2 = ss(A2, B2, C2, D2, Ts)

[y2, x2] = dlsim(A2, B2, C2, D2, u, x0);

figure(1); subplot(311); hold on; stairs(kT, x1(:,1), 'ro-'); stairs(kT, x2(:,1), 'mo-'); xlabel('kT'); ylabel('x_1(k)');
           subplot(312); hold on; stairs(kT, x1(:,2), 'ro-'); stairs(kT, x2(:,2), 'mo-'); xlabel('kT'); ylabel('x_2(k)');
           subplot(313); hold on; stairs(kT, y1, 'rx-'); stairs(kT, y2, 'm*-'); xlabel('kT'); ylabel('y(k)');
figure(2); hold on; plot(x1(:,1), x1(:,2), 'ro:'); plot(x2(:,1), x2(:,2), 'mo:'); xlabel('x_1(k)'); ylabel('x_2(k)');

% Observe que os autovalores das tres matrizes de estado A, A1 e A2 sao
% identicos entre si e aos polos de Gz:
eig(A), eig(A1), eig(A2), roots(dGz)

% Observe que as tres representa?oes no espaco de estados produzem a mesma
% funcao de transferencia:

Gz % FT original

[n1, d1] = ss2tf(A1, B1, C1, D1, Ts);
Gz1 = zpk(minreal(tf(n1, d1, Ts))) % FT do caso 1

[n2, d2] = ss2tf(A2, B2, C2, D2, Ts);
Gz2 = zpk(minreal(tf(n2, d2, Ts))) % FT do caso 2