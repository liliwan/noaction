% Some data that I generated drawing from a 2D multivariate
% normal distribution, stored in the array pts
%
% I already pre-computed the mean vectors and covariance
% matrices before hand
load gmm_data;

% Determine the membership of each point, stored in IDX
IDX = clusterGMM(pts,MU,SIGMA);

% Plot where each point belongs to, and colour coded.
close all;
plot(pts(IDX == 1,1),pts(IDX==1,2),'b.',pts(IDX==2,1),pts(IDX==2,2),'r.');
legend('Class 1', 'Class 2');
hold on;
plot2dgauss(MU,SIGMA);
grid;
xlabel('x1');
ylabel('x2');
title('Example of GMM Clustering - 2 classes');