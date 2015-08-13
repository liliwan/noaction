function Pos = LearnGM(fileName)

M = csvread(fileName);

numM = size(M,1);
minDataNum = 5;
numSampling = 10;

if numM > minDataNum
    
    % fit
    options = statset('Display','final');
    gm = gmdistribution.fit(M,1,'Options',options);
    
    % hold on
    % ezcontour(@(x,y)pdf(gm,[x y]),[0 1],[0 1]);
    % hold off
    
    % sampling
    Pos = random(gm, numSampling);
else
    randSamp = randi(numM, numSampling, 1);
    Pos = M(randSamp,:);
    
end
end