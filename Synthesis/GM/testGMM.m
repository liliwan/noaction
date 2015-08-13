% Test code to test out GMM Mixture model learning
% Going to learn a model using two mixtures

clear all;
close all;

% Pulled from MATLAB GMM help
% Create mean and covariance matrices, and generate
% a random set of points from both multivariate distributions
mu1 = [1 2];
sigma1 = [3 .2; .2 2];
mu2 = [-1 -2];
sigma2 = [2 0; 0 1];
pts = [mvnrnd(mu1,sigma1,200);mvnrnd(mu2,sigma2,100)];

% Create our Gaussian mixture model
[weights,MU,SIGMA,dist,iter] = createGMM(pts, 2);

% Plot the points, with covariance ellipses
close all;
figure;
plot(pts(:,1),pts(:,2),'b.');
hold on;
plot2dgauss(MU,SIGMA);    
plot(MU(:,1),MU(:,2),'kx','MarkerSize',14,'LineWidth',2);
grid;
xlabel('x1');
ylabel('x2');
title('Example of GMM learning - 2 clusters');
