LayoutDir = 'C:\Graphics\Scene_Code\SceneDB\Stanford\LayoutData\';

% desk_TV  desk_keyboard
M = csvread([LayoutDir 'desk_tv.csv']);
LabelM = ones(size(M,1),1);
N = csvread([LayoutDir, 'desk_keyboard.csv']);
LabelN = 4*ones(size(N,1),1);

P = csvread([LayoutDir, 'desk_mouse.csv']);
LabelP = 2*ones(size(P,1),1);

Label = cat(1,LabelM,LabelN, LabelP);
A = cat(1,M,N,P);