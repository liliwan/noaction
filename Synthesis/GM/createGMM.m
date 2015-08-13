% Implementation of learning a Gaussian mixture model on a data set
%
% Written by Raymond Phan, Ryerson University
% Version 1.0 - Initial creation, Sept. 5, 2012
% Version 2.0 - Made changes so that function was more vectorized, Sept. 6,
%               2012
%
% Given a data set, and the total number of desired clusters, this program
% will determine a Gaussian mixture model that will best fit this data
%
% This essentially implements a special case of the
% Expectation-Maximization algorithm (EM) for determining the model.  For
% more details on how the algorithm works, this presentation is a very good
% primer on how to implement it:
%
% http://www.slideshare.net/petitegeek/expectation-maximization-and-gaussian-mixture-models
%
% Inputs:
% X - A N x D matrix, where N represents the total number of sample points,
% while D represents the dimension of each point
%
% K - The total number of desired clusters to fit the model to
%
% min_dist - The distance used to check for convergence.  If the log
% likelihood difference between two successive iterations falls below
% min_dist, we will exit the program
%
% numIter - The maximum allowed number of iterations
%
% Note - By omitting min_dist and numIter upon calling this function, the
% default parameters are 1e-5 and 100 respectively.
%
% Outputs:
% weights - The weights for each Gaussian mixture
%
% MU - The mean matrix, which is a K x D matrix.  Each row denotes the 
% mean for the particular mixture
%
% SIGMA - The covariance matrices, which is a D x D x K matrix.  The 
% i'th 2D slice is the covariance matrix for the i'th cluster
%
% dist - The absolute log likeliehood difference between the two most
% current iterations
% 
% iter - The total number of iterations performed once the algorithm
% converged
%
% To see an example of how this is used, please refer to the testGMM.m code
function [weights,MU,SIGMA,dist,iter] = createGMM(X, K, min_dist, numIter)

if(nargin == 2)
    min_dist = 1e-5;
    numIter = 100;
elseif(nargin == 3)
    numIter = 100;
elseif(nargin < 2 || nargin >= 5)
    error('createGMM: Please ensure you specify the right amount of parameters');
end

% Just in case...
if(numIter <= 0)
    error('createGMM: Please specify a non-negative number of iterations');
end

if(min_dist < 0)
    error('createGMM: Please specify a non-negative minimum distance');
end

if(K <= 1)
    error('createGMM: Please be sure that you specify at least 2 clusters or more');
end

% Some useful variables
total_pts = size(X,1);
N = size(X,2);

% Initialize GMM
[MU,IDX] = raykmeans(X,K);
SIGMA = zeros(N,N,K);
weights = zeros(1,K);
for i = 1 : K    
    pts = X(IDX == i,:);
    SIGMA(:,:,i) = cov(pts);
    weights(i) = size(pts,1)/total_pts;
end

% For each iteration...
for i = 1 : numIter
    % Store to check for convergence
    MU_BEFORE = MU;
    SIGMA_BEFORE = SIGMA;
    [DET_SIGMA_BEFORE INV_SIGMA_BEFORE] = GenerateCovDetInv(SIGMA_BEFORE);
    [DET_SIGMA INV_SIGMA] = GenerateCovDetInv(SIGMA);    
    weights_before = weights;
    
    % Perform E-step
    % Determine responsibility of each point to each Gaussian
    % cluster
%%%% OLD CODE    
%    for j = 1 : total_pts
%        pt = X(j,:);
%        % Determine normalization factors
%        accum = weights.*mvnpdf(pt(ones(1,K),:),MU,SIGMA);
%        accum = weights.*raymvnpdf(pt(ones(1,K),:),MU,DET_SIGMA,INV_SIGMA);
%        % Determine responsibility
%        gamma(j,:) = (accum / sum(accum))';
%    end

    weights_rep_before = weights_before(ones(total_pts,1),:);
    weights_rep = weights(ones(total_pts,1),:);
    gamma_before = weights_rep.*raymvnpdf(X,MU,DET_SIGMA,INV_SIGMA);
    gamma_factor = sum(gamma_before,2);
    gamma = gamma_before ./ gamma_factor(:,ones(1,K));
    
    % Perform M-step for the means
    NK = sum(gamma,1);         
    for k = 1 : K
        gammak = gamma(:,k);
        MU(k,:) = (1/NK(k)) * sum(gammak(:,ones(1,N)).*X,1);
    end
    
    % Perform M-step for the covariance matrices
    for k = 1 : K        
        gammak = gamma(:,k);
        uknew = MU(k,:);
%%%% OLD CODE        
%        sigma = zeros(N,N);        
%        for j = 1 : total_pts
%            vect = (X(j,:) - uknew);
%            vec = vect';
%            sigma = sigma + gammak(j)*(vec*vect);
%        end
        dif = X - uknew(ones(total_pts,1),:);
        % Can also do (dif')*diag(gammak)*dif, but below is faster
        sigma = (dif')*(gammak(:,ones(1,N)).*dif);
        SIGMA(:,:,k) = sigma / NK(k);
    end
    
    % Perform M-step for the mixing coefficients
    weights = NK ./ total_pts;
%%%% OLD CODE
%    dist_before = 0; dist_after = 0;
%    for j = 1 : total_pts
%        pt = X(j,:);
%        dist_before = dist_before + ...
%            log(sum(weights_before.*raymvnpdf(pt(ones(1,K),:),...
%            MU_BEFORE,DET_SIGMA_BEFORE,INV_SIGMA_BEFORE)));
%        dist_after = dist_after + ...
%            log(sum(weights.*raymvnpdf(pt(ones(1,K),:),MU,DET_SIGMA,...
%            INV_SIGMA)));
%    end
    
    % Check for convergence    
    dist_before = sum(log(sum(weights_rep_before.*raymvnpdf(X,...
        MU_BEFORE,DET_SIGMA_BEFORE,INV_SIGMA_BEFORE))));
    dist_after = sum(log(sum(weights_rep.*raymvnpdf(X,MU,DET_SIGMA,...
        INV_SIGMA))));
        
    dist = abs(dist_before-dist_after);
    if(dist < min_dist)
        break;
    end
end

iter = i;