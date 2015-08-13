% This code determines which points in a given data set belong to which
% Gaussian mixture.  Because this is purely a classification problem, the
% weights are not required.
%
% Written by Raymond Phan, Ryerson University
% Version 1.0 - Sept. 8, 2012 - Initial creation
%
% The distance used is the Mahalanobis distance, as this takes the shape of
% the distributions into account.  For a given multivariate data point, X,
% that is N-dimensional, and a covariance matrix S that describes a 
% distribution of points, the distance is defined as:
%
% D(X) = sqrt((X-mu)' * S^{-1} * (X-mu));
%
% For each point in the data set, we determine the Mahalanobis distance to
% each mixture, and classify the point to be from the mixture that yields
% the smallest distance.
%
% Inputs - 
% X - A N x D matrix, where N represents the total number of sample points,
% while D represents the dimension of each point.
%
% MU - A K x D matrix, where the i'th row corresponds to the mean of the 
% i'th mixture, and is D dimensional.  K is the number of mixtures in
% the data set.
%
% SIGMA - A D x D x K matrix, where the i'th 2D slice is the 
% covariance matrix for the i'th mixture.
%
% Outputs - 
% IDX - An N x 1 matrix, where the i'th element denotes the mixture
% membership for the i'th data point.

function [IDX] = clusterGMM(X,MU,SIGMA)
% Number of mixtures
K = size(MU,1);

% Number of data points
N = size(X,1);

% Dimension of data points
dim = size(MU,2);

% Distance matrix - Each row denotes the distance between this point and
% all the mixtures.  The j'th column is the distance from the i'th data
% point, to the j'th mixture.
dist = zeros(N,K);

% For each mixture...
for i = 1 : K
    % Compute the Mahalanobis distance between this mixture, and all of the
    % points
    mu = MU(i,:);
    mu = mu(ones(N,1),:);
    dif = X - mu;
    INV_SIGMA = SIGMA(:,:,i)\eye(dim);
    dist(:,i) = diag(dif*INV_SIGMA*dif');
end

% For each data point, find the point that is the closest to a mixture.
[~,IDX] = min(dist,[],2);