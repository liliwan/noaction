% Helper function that calculates the determinant and
% the inverse for a set of covariance matrices for use
% in GMM parameter estimation
%
% SIGMA should be a D x D x K matrix, where K denotes
% the number of mixture models we desire, and 
% the i'th 2D slice is the covariance of the i'th
% mixture.
%
% D is the number of features per point in the
% feature space
%
% DET_SIGMA and INV_SIGMA return the determinants and
% the inverses of each covariance matrix
%
% DET_SIGMA is a K x 1 array, where the i'th element
% is the determinant for the i'th covariance matrix
%
% INV_SIGMA is a D x D x K matrix, where the i'th
% 2D slice is the inverse for the i'th covariance
% matrix
%
% Raymond Phan, Ryerson University - September 6, 2012

function [DET_SIGMA INV_SIGMA] = GenerateCovDetInv(SIGMA)
n = size(SIGMA,3); % number of mixtures
r = size(SIGMA,2); % 
DET_SIGMA = zeros(n,1);
INV_SIGMA = zeros(size(SIGMA));
for i = 1 : n
    sigma = SIGMA(:,:,i);
    DET_SIGMA(i) = det(sigma);
    INV_SIGMA(:,:,i) = sigma\eye(r,r);
end
end

