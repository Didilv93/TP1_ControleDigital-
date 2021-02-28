numGHbar = 0.368*[1 0.717];
denGHbar = conv([1 -1], [1 -0.368]);
T = 0.1; % Periodo de amostragem
GHbar = tf(numGHbar, denGHbar, T) % Funcao de transferencia da eq. carac. 1 + K GHbar(z) = 0

K = [0:0.001:30]; % Escolhendo valores de K pros quais o lugar das raizes sera tracado

rlocus(GHbar, K); zgrid; % Traca o lugar das raizes

[Kc, raizes] = rlocfind(GHbar) % Para encontrar com cursor ganho associado a alguma raiz