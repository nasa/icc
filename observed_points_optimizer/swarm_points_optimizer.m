function [observed_points, priority] = swarm_points_optimizer(observable_points_map, reward_map, sc_optimized)
%SWARM_POINTS_OPTIMIZER Optimizes the observed points of the agents in
%sc_optimized

%% Setup 
tic

N = length(observable_points_map);
Nv = size(reward_map{sc_optimized(1)},1); % number of vertices
K  = size(reward_map{sc_optimized(1)},2); % number of time steps

% Total number of potential observations = size of decision variable = M
M = 0;
for i_sc = sc_optimized
    M = M + sum(sum(observable_points_map{i_sc}==1));
end

% Extract list of observed vertices:
observable_vertices = [];
for i_sc = sc_optimized
    for k = 1:K
        observable_vertices = unique([observable_vertices; find(observable_points_map{i_sc}(:,k)==1)]);
    end
end
n_observed_vertices = length(observable_vertices);

n_constraints = N*K + n_observed_vertices;

% Create pointer vectors to keep track of the time, vertex, and agent
% corresponding to the pth position in the decision variable
p = 0; p_k=zeros(1,M); p_v=zeros(1,M); p_s=zeros(1,M); w=zeros(1,M);
for k = 1:K
    for i_v = 1:Nv
        for i_sc = sc_optimized
            if observable_points_map{i_sc}(i_v, k)==1 % if vertex i_v is observable at time k
                p = p+1;
                p_k(p) = k;
                p_v(p) = i_v;
                p_s(p) = i_sc;
                w(p) = reward_map{i_sc}(i_v,k); % rewards corresponding to each potential observation
            end
        end
    end
end

% Store sets of indicies for observations by same spacecraft at same time
sigma_ik = cell(N,K);
for k = 1:K
    for i_sc = sc_optimized
        sigma_ik{i_sc,k} = intersect(find(p_k==k),find(p_s==i_sc));
    end
end

% Store sets of indicies for observations of the same vertex at different times (and by different spacecraft)
sigma_v = cell(1,n_observed_vertices);
for p = 1:n_observed_vertices
    sigma_v{p} = find(p_v==observable_vertices(p));
end

%% Define Optimization Input Arrays
A = zeros(n_constraints, M); 
i_constraint = 0;
for k = 1:K
    for i_sc = sc_optimized
        i_constraint = i_constraint+1;
        A(i_constraint , sigma_ik{i_sc,k}) = 1;
    end
end
for p=1:n_observed_vertices
    i_constraint = i_constraint+1;
    A(i_constraint, sigma_v{p}) = 1;
end
A = sparse(A);
b = ones(n_constraints, 1);

% Set bounds
UB = ones(M,1);
LB = zeros(M,1);
fprintf('Total time to setup observation points problem: %0.2f\n',toc)

%% Solve 
% Define problem
tic
X = intlinprog(-w, 1:M, A, b, [], [], LB, UB);
fprintf('Total time to solve observation points problem: %0.2f\n',toc)

% Fill out observed points and reward vectors
observations = find(X==1);
observed_points = zeros(N,K);
priority =  zeros(N,K);
for i = 1:length(observations)
    i_obs = observations(i);
    observed_points(p_s(i_obs), p_k(i_obs)) = p_v(i_obs);
    priority(p_s(i_obs), p_k(i_obs)) = w(i_obs); % reward translates to priority
end

end

