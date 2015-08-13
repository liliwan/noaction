
close all
LayoutDir = 'E:\Gruvi\mess_it_up - noaction\data\LayoutData\'; %'C:\Graphics\Scene_Code\SceneDB\Stanford\LayoutData\';

Supporter = 'desk';
fisrtObj = 'book';
secondObj = 'lamp';
thirdObj = 'bottle';

% desk_TV  desk_keyboard
M = csvread([LayoutDir Supporter, '_' fisrtObj '.csv']);
LabelM = ones(size(M,1),1);
N = csvread([LayoutDir, Supporter,'_' secondObj '.csv']);
LabelN = 4*ones(size(N,1),1);

P = csvread([LayoutDir, Supporter, '_' thirdObj '.csv']);
LabelP = 2*ones(size(P,1),1);

Label = cat(1,LabelM,LabelN, LabelP);
A = cat(1,M,N,P);
%scatter(A(:,1), A(:,2),30, Label');

% fit
options = statset('Display','final');
gm1 = gmdistribution.fit(M,1,'Options',options);
gm2 = gmdistribution.fit(N,2,'Options',options);
%gm3 = gmdistribution.fit(P,1,'Options',options);

figure
hold on
ezcontour(@(x,y)pdf(gm1,[x y]),[0 1],[0 1]);
ezcontour(@(x,y)pdf(gm2,[x y]),[0 1],[0 1]);
%ezcontour(@(x,y)pdf(gm3,[x y]),[0 1],[0 1]);
legend(fisrtObj,secondObj, thirdObj);
hold off
title('Gaussion Mixtures')

Y = random(gm1, 1000);

% figure
% hold on
% scatter(M(:,1), M(:,2),30);
% scatter(N(:,1), N(:,2),30,'x');
% scatter(P(:,1), P(:,2),50,'+')
% scatter(Y(:,1), Y(:,2), 50);
% hold off

legend(fisrtObj,secondObj, thirdObj);
title('Observations')

% figure
% hold on
% scatter(M(:,1), M(:,2),30);
% %scatter(N(:,1), N(:,2),30,'x');
% %scatter(P(:,1), P(:,2),50,'+')
% scatter(Y(:,1), Y(:,2),50,'rx');
% hold off
% legend(fisrtObj,'samplings');
% title('Samplings')