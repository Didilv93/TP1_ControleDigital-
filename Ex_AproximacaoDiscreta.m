
close all; clc;

% Compensador avanco-atraso analogico:
a = 10;
b = 0;
Ds = tf([1 a],[1 b])
bode(Ds);

% Aproximacao discreta:
T = 0.1;

% Mapeamento de polos e zeros:
Dz1 = c2d(Ds, T, 'matched')
hold on; bode(Dz1); pause;

% Trapezoidal ou Tustin:
Dz2 = c2d(Ds, T, 'tustin')
hold on; bode(Dz2); pause;

% Equivalencia de segurador ZOH:
Dz3 = c2d(Ds, T, 'zoh')
hold on; bode(Dz3); pause;

% Equivalencia de segurador FOH:
Dz4 = c2d(Ds, T, 'foh')
hold on; bode(Dz4);
legend('Analog','Matched','Tustin','ZOH','FOH')

