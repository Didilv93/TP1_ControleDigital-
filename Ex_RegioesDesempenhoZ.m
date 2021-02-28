T = 0.2;
zeta = [0.5 0.8];
omegan = T*[4 8];
raio = exp(-T.*zeta.*omegan)
figure;
%zgrid; %hold on;
zgrid(zeta,omegan,'new'); hold on;
plotcircle(0,0,raio(1),'k-.'); hold on; plotcircle(0,0,raio(2),'k--');

%%
zgrid([0.1:0.1:0.9],'new')

%%
figure; zgrid([],[0.1:0.1:1]*pi,'new')

%% 
figure;
zgrid; hold on; 
plotcircle(0,0,0.1,'b-.'); plotcircle(0,0,0.5,'b--'); plotcircle(0,0,0.9,'b-.');