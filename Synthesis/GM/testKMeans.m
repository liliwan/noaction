% Test code to test out the KMeans algorithm I implemented
clear all;
close all;

% Generate 4 sets of 20 random points in 2D
A = randn(20,2);
B = randn(20,2);
C = randn(20,2);
D = randn(20,2);

% Make it so that one cluster is centered around (10,10), the other at
% (-10,10), one more at (-10,10), and finally one at (10,-10)
A = A + 10;
B = B - 10;
C = [C(:,1)-10 C(:,2)+10];
D = [D(:,1)+10 D(:,2)-10];

% Create one final matrix of points, and re-arrange the points in a random
% fashion to ensure no bias
pts = [A;B;C;D];
pts = pts(randperm(80),:);

% Run the algorithm, and plot the points, colour coding what cluster each
% point belongs to.
[CL,IDX,dist,iter] = raykmeans(pts,4);
plotClusters(pts,IDX);
plot(CL(:,1),CL(:,2),'kx','MarkerSize', 14, 'LineWidth',2);
axis([-15 15 -15 15]);