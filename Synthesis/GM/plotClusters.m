% Function that takes data points, as well as which clusters they belong
% to, and plots this data via a scatterplot.  
% The data is colour coded so that points that belong to the same cluster 
% share the same colour.
%
% Note - This script only works for visualizing data in 1D, 2D and 3D.
%
% Written by Raymond Phan, Ryerson University
% Version 1.0 - Initial Creation, Sept. 5, 2011
%
% Inputs - 
% X - A N x D matrix, where N represents the total number of sample points,
% while D represents the dimension of each point
%
% IDX - An N x 1 array, where the i'th row corresponds to what cluster the
% i'th point belongs to from the data matrix X (each point occupies 1 row
% in X)
%
% Outputs
% (Optional) - h: The figure handle that contains this plot
%
% To see an example of how this is used, please refer to the testKMeans.m
% code

function h = plotClusters(X,IDX)

% 7 possible colours when plotting
colours = 'bgrcmyk';
N = max(IDX); % Determine number of clusters
if(N > 7)
    error('Cannot visualize for more than 7 clusters');
end

% Create the figure
figure;
hold on;

% For each cluster...
for i = 1 : N
    % Grab those data points that belong to the i'th cluster
    idx = IDX == i;
    pts = X(idx,:);
    
    % Choose our colour
    str = [colours(i) '.'];
    
    % Determine what type of data we're drawing (1D, 2D, or 3D)
    % For 1D, make the y-axis all zero, and plot the points along the
    % x-axis accordingly
    if(size(X,2) == 1)
       h = plot(pts,zeros(1,length(idx)),str);
    % 2D Plot
    elseif(size(X,2) == 2)
       h = plot(pts(:,1),pts(:,2),str);
	% 3D Plot
    elseif(size(X,2) == 3)
       h = plot3(pts(:,1),pts(:,2),pts(:,3),str);
    else %.. and we can't do anything else other than the above.
        error('plotClusters: Cannot Visualize');
    end
end

grid;
axis tight;