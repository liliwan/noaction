% My own implementation of the Multivariate Normal
% PDF function
%
% Raymond Phan, Ryerson University - Sept. 6, 2012
%
% The built-in mvnpdf given in MATLAB has
% a lot of filler and error checking code that
% eats up a lot of time when invoking the function
% a large amount of times.  This is also used
% to help speed up the code for estimating
% the GMM parameter estimation via EM.
%
% The code is designed to take in observation
% vectors (N-dimensional), and computes the
% probability of occurrence over all populations
% 
% i.e. Given M samples, that are each N-dimensional,
% compute the probability of occurrence between each
% sample with ALL of the populations
% 
% This is essentially a stripped down version of
% that code.  The PDF of a MVN PDF with a mean
% vector U, and covariance matrix C, is the following:
% (using MATLAB syntax to describe it)
%
% p(X) = (1 / ((2*pi)^(K/2))*(sqrt(det(C)))) * ...
%        exp(-0.5*(X-U)' * inv(C) * (X-U));
%
% Reference: 
% http://en.wikipedia.org/wiki/Multivariate_normal_distribution#Non-degenerate_case
%
% To increase numerical stability, I computed log(p(X)), to break up
% the products in the expression as a summation.  After computation,
% I take the exp() operator.
%
% Inputs: 
% X - An N x D matrix, where N is the total number of
% points, and D is the dimension of each point
%
% MU - A K x D matrix, where K is the number
% of normally distributed populations within
% the data set.
%
% DET_SIGMA - A K x 1 array, where each element
% is the determinant of the covariance matrix for 
% the particular population
%
% INV_SIGMA - A D x D x K matrix, where each
% 2D slice is the inverse of the covariance matrix
% for the particular population
%
% Outputs:
% Y - A N x K matrix.  Each i'th row corresponds to
% a single point in the data set.  Each j'th column
% in the i'th row is the probability that the
% i'th data point appears in the j'th population
%
%
function [Y] = raymvnpdf(X,MU,DET_SIGMA,INV_SIGMA)
n = size(X,1); % total number of points
d = size(X,2); % # of dimensions
K = size(MU,1); % # of clusters

% Constant factor outside the exponential, but
% becomes a log sum.
ln_pi = (-0.5*d*log(2*pi));
ln_det = (-0.5*log(DET_SIGMA));

Y = zeros(n,K);

% For each population...
for k = 1 : K
    % Compute the likelihood that each data set point
    % belongs to the k'th population
    mu = MU(k,:);
    mu = mu(ones(n,1),:);        
    dif = X - mu;
    
    quadform = (-0.5)*diag((dif*INV_SIGMA(:,:,k)*dif'));
    Y(:,k) = exp(quadform' + ln_det(k) + ln_pi);
end