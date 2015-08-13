% Implementation of Lloyd's algorithm / K-Means clustering algorithm
%
% Written by Raymond Phan, Ryerson University
% Version 1.0 - Initial creation, Sept. 5, 2012
%
% Given a data set, and the amount of clusters desired for grouping the
% data, this program will determine how to best classify each data point in
% the data set to belong to one of these clusters.
%
% The proximity criterion used to determine how close two points are
% in space is the squared Euclidean distance, as this maintains rotational
% invariance
%
% For more information on how the algorithm works, please consult:
% 1)  http://www.slideshare.net/petitegeek/expectation-maximization-and-gaussian-mixture-models
% 2)  http://en.wikipedia.org/wiki/K-means_clustering
%
% Inputs:
% X - A N x D matrix, where N represents the total number of sample points,
% while D represents the dimension of each point
%
% K - The total number of desired clusters for k-means
%
% min_dist - The distance between the cluster centres of the two most
% current iterations such that if this distance is below this threshold,
% the algorithm stops.
%
% Theoretically, the algorithm should converge when the cluster centres
% remain unchanged, but we'll do this in case the cluster centres are
% slowly converging towards the optimum locations.
%
% The criterion I'm using is the following, assuming that C_i and C_(i-1)
% are K x D matrices, where the j'th row is the cluster centre for cluster
% j.  i and i-1 denote the current and previous iterations respectively.
%
% dist = inf_norm(C_i - C_(i-1))
%
% numIter - The maximum allowed number of iterations
%
% Note - By omitting min_dist and numIter upon calling this function, the
% default parameters are 1e-5 and 100 respectively.
%
% Outputs:
% C - The final cluster centres, which is a K x D matrix.  
% The i'th row denotes the final cluster centre for cluster i.
%
% IDX - An N x 1 array, where the i'th row corresponds to what cluster the
% i'th point belongs to from the data matrix X (each point occupies 1 row
% in X)
%
% dist - The difference of infinite norms between the cluster centre
% matrices of two most current iterations
% 
% iter - The total number of iterations performed once the algorithm
% converged
%
% To see an example of how this is used, please refer to the testKMeans.m
% code

function [C,IDX,dist,iter] = raykmeans(X, K, min_dist, numIter)

if(nargin == 2)
    min_dist = 1e-5;
    numIter = 100;
end

% Total number of samples
N = size(X,1);

% Run k-means++ algorithm to determine initial cluster centres
C = kmeanspp(X,K);
IDX = zeros(N,1);

for i = 1 : numIter
    % Save previous iteration to check for convergence    
    C_before = C;
    
    % Determine the membership for each point to each cluster
    for j = 1 : N
        cdist = sum((X(j*ones(1,K),:) - C).^2,2);        
        [~,IDX(j)] = min(cdist);        
    end
    
    % Update the means
    for j = 1 : K        
        C(j,:) = mean(X(IDX == j,:),1);
    end
    
    % Check for convergence
    dist = norm(C_before - C,'inf');
    if(dist < min_dist)
        break;
    end
end

iter = i;