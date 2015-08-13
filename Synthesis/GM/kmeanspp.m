% Implementation of the k-means++ algorithm
%
% Written by Raymond Phan, Ryerson University
% Version 1.0 - Initial creation, Sept. 5, 2012
% Version 1.1 - Bug fix and modified slightly to use a double greedy
%               procedure, Sept. 6, 2012
%
% Given a data set, this algorithm will determine what the best initial
% clusters for use in the initial step of the k-means clustering algorithm
%
% Randomly choosing points within a dataset as the initial clusters will
% inevitably be prone to errors, especially with the chance of choosing the
% same point more than once.
%
% This algorithm was created to minimize the error associated with k-means
% clustering.  Specifically, the end result is entirely dependent on what
% initial clusters you choose.  This algorithm has been shown to
% significantly increase accuracy.
%
% For more information on how the algorithm works, please consult:
% 1)  http://ilpubs.stanford.edu:8090/778/1/2006-13.pdf
% 2)  http://en.wikipedia.org/wiki/K-means%2B%2B
%
% Inputs:
% X - A N x D matrix, where N represents the total number of sample points,
% while D represents the dimension of each point
%
% K - The total number of desired clusters for k-means
%
% Outputs:
% C - The initial cluster centres, which is a K x N matrix.  
% The i'th row denotes the initial cluster centre for cluster i.

function [C] = kmeanspp(X,K)

% Important variables
N = size(X,1); % Number of points
dim = size(X,2); % dimension of each sample
C = zeros(K,dim); % Output initial cluster centres

% Step #1 - Choose an initial sample uniformly at random from among 
% data points
ind = randi(N,1);

% Add this to cluster set
C(1,:) = X(ind,:);

% This variable keeps track of which clusters are still available to
% be chosen.  When each cluster is selected, it is removed from the overall
% list to ensure that we don't select the same cluster twice
Xlist = X;

% Variable used to keep track of the cluster chosen in the previous
% iteration
final_ind = ind;

% While we still have clusters to choose from... (Step #4)
for i = 2 : K
    % Step #2 - For each data point x, compute D(x)^2, the 
    % squared distance between the current cluster centre and this 
    % data point
    clust = Xlist(final_ind,:);
    dist = sum((Xlist - clust(ones(1,(N-i+2)),:)).^2,2);
    
    % Remove self entry
    [~,ind_remove] = min(dist);
    dist(ind_remove) = [];
    Xlist(ind_remove,:) = [];
    
    % Compute weighted probability
    dist = dist / sum(dist);
    
    % Step #3 - Choose one new data point, at random as a new centre, using a
    % weighted probability distribution where a point x is chosen with
    % probability proportional to D(x)^2
    
    % Slight modification - Double greedy procedure
    % We choose ten random points as the new centres.  With these,
    % examine their corresponding probabilities and finally choose the one
    % that has the biggest probability as the new centre
    
    % Reason: Because the sampling is probabilistic, there is a small (but
    % non-zero) probability that the next centre in line will be very close
    % to the previous centre, even when there are better centres around.
    % This way, this biases the distribution to the further away points.
    ind = randsample(N-i+1,10,true,dist);
    
    % Examine corresponding probabilities of these ten points and grab
    % where these centres are located within the list
    prob = dist(ind);
    
    % Determine which of these ten points has the maximum probability,
    % then choose this as the final initial centre.
    [~,m] = max(prob);
    final_ind = ind(m);
    
    % Add this new point to cluster list
    C(i,:) = Xlist(final_ind,:);
end

% Step #5 - Algorithm finished - Use these as initial cluster centres