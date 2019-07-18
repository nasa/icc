function [observed_points, priority] = single_agent_points_optimizer(observable_points_map, reward_map)
%SINGLE_AGENT_POINTS_OPTIMIZER chooses the best set of observation points
%given the set of available points (observable_points_map) and a
%specification on their value (reward_map)

tic 
Nv = size(reward_map,1); % number of vertices
K  = size(reward_map,2); % number of time steps
M = sum(sum(observable_points_map==1)); % size of decision variable

% Create pointer matrix P such that P(:, j) returns the vertex and time of
% the jth observable point
P = zeros(2,M);
p = 0;clc
for k = 1:K
    for i_v = 1:Nv
        if observable_points_map(i_v, k)==1 % if vertex i_v is observable at time k
            p = p+1;
            P(:,p) = [i_v; k]; 
        end
    end
end

% Define sets of indicies for contemporaneous observations
sigma_k = cell(1,K);
for k = 1:K
    sigma_k{k} = find(P(2,:)==k);
end

% Define sets of indicies for observations of the same vertex at different times
p = 0;
for i_v = 1:Nv
    inds = find(P(1,:)==i_v);
    if ~isempty(inds)
        p=p+1; %observable vertex found
        sigma_v{p} = inds;
    end
end
n_observed_vertices = p;

% Define Constraints
A = sparse(zeros(n_observed_vertices+K, M));
for k = 1:K
    A(k,sigma_k{k}) = 1;
end
for p=1:n_observed_vertices
    A(K+p, sigma_v{p}) = 1;
end
b = ones(n_observed_vertices+K, 1);

% Vectorize reward matrix =
w = reward_map(:);
w = w(observable_points_map(:)==1); % remove indicies for unobservable rewards

% Set bounds
UB = ones(M,1);
LB = zeros(M,1);
toc 

% Define problem
tic
X = intlinprog(-w, 1:M, A, b, [], [], LB, UB);
toc

% Fill out observed points and reward vectors
observations = find(X==1);
observed_points = zeros(1,K);
priority =  zeros(1,K);
for i = 1:length(observations)
    observed_vertex = P(1,observations(i));
    time_of_observation = P(2,observations(i));
    observed_points(time_of_observation) = observed_vertex;
    priority(time_of_observation) = w(observations(i)); % reward translates to priority
end

end

