function ComputeLayout(path, SceneName, SuppGiverName)

fileName = [path SceneName '_' SuppGiverName '.txt'];
fileID = fopen(fileName);

currLine = fgetl(fileID);
sceneID = -1;
while ~feof(fileID)
    switch currLine(1)
        case 'S'
        case 'N'
            sceneID = sceneID+1;
        case 'L'            
        otherwise
            modelName = currLine;
            pos = LearnGM([path SuppGiverName '_' modelName '.csv']);
            csvwrite([path 'SynPos\' SceneName '_' SuppGiverName '_' modelName '.pos'], pos);
            currLine = fgetl(fileID);
            currLine = fgetl(fileID); 
    end
    currLine = fgetl(fileID);
end
fclose(fileID);
end