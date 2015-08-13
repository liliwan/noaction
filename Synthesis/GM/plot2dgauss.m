% plot2dgauss, based on code by Mark Paskin
% function h=plot2dgauss(Mu, Sigma, color)
% Plot an ellipse representing the covariance matrix of a Gaussian
%
% This function was taken from Kevin Murphy's PDF - 
% http://www.cs.ubc.ca/~murphyk/Teaching/CS340-Fall06/reading/gaussians.pdf
%
% Modified by Raymond Phan to plot more than one ellipse - Sept. 5, 2012
%
% Inputs: 
% Mu - An K x N matrix, where K represents the number of Gaussian mixtures
% N represents the dimension of the data.  Each row represents the mean for
% that particular Gaussian mixture
%
% Sigma - An N x N x K 3D covariance matrix.  Each i'th 
% 2D slice represents the covariance matrix for the i'th Gaussian mixture
%
% color - A single character string, specifying what colour to colour in
% the ellipse
function h=plot2dgauss(Mu, Sigma, color)

if (size(Sigma,1) ~= 2 || size(Sigma,2) ~= 2)
    error('Sigma must be a 2 by 2 matrix'); 
end

if (nargin < 3)
    color = 'r'; 
end

if(size(Sigma,3) ~= size(Mu,1))
    error('Ensure that the same number of means and covariances are specified');
end

n = 100;
t = linspace(0, 2*pi, n);
xy = [cos(t); sin(t)];

% Removed internal function conf2mahal and simply replaced with chi2inv
% to avoid confusion
k = sqrt(chi2inv(0.95, 2));

% For each Gaussian mixture...
for i = 1 : size(Sigma,3)
    % Plot the ellipse
    mu = (Mu(i,:))';
    [U, D] = eig(Sigma(:,:,i));
    w = (k * U * sqrt(D)) * xy;
    z = repmat(mu, [1 n]) + w;
    h = plot(z(1, :), z(2, :), color);
end