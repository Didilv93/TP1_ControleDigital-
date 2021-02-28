% ELT013 - Controle Digital
%
% Considere o sistema din?mico descrito pela equa??o de diferen?as
% y[k] ? y[k-1] + 0,25y[k-2] = 2x[k-1].
% Usando o m?todo da transformada Z, encontre a resposta ao degrau deste sistema. 
%
% UFMG, Bruno Teixeira, 03/2017

clear all;
close all;

% Funcao de transferencia:
numH = [0 2 0]
denH = [1 -1 0.25]

% Resposta ao degrau:
numU = [1 0];
denU = [1 -1];
% Obs: A fim de obter a decomposicao de S(z)/z, observe que numSm abaixo e
% modificado*** :
numS = conv(numH, numU);
numSm = conv(numH, [0 1]); % ***
denS = conv(denH, denU);
[R, P, K] = residue(numS, denS)
[R, P, K] = residue(numSm, denS) % *** --> Use essa forma!!!
% Conforme help residue:
%      num(s)       R(1)       R(2)             R(n)
%      ------  =  -------- + -------- + ... + -------- + K(s)
%      den(s)     s - P(1)   s - P(2)         s - P(n)

% Simulacao:
figure;
dstep(numH, denH);

sk(1) = 0;
s2k(1) = 0;
s3k(1) = 0;
for j = 2:5000
    k = j-1;
    sk(j) = R(1)*P(1)^k + R(2)*P(2)^k + R(3)*k*(1/P(3))*(P(3)^k); % Observe que um fator 1/P(3) foi colocado aqui para fazer uso da Tabela k*a^k*u[k] ---> az / z - a
    s2k(j) = 8 - 8*(0.5^k) - 4*k*(0.5^k); % Resultado obtido analiticamente em aula
    
end
hold on; stairs([0:(length(sk)-1)], sk,'r*');
stairs([0:(length(s2k)-1)], s2k,'gx');


