function [observed_points, priority] = swarm_points_optimizer(observable_points_map, reward_map, sc_optimized)
%SWARM_POINTS_OPTIMIZER Optimizes the observed points of the agents in
%sc_optimized

% observable_points_map is a cell of size N (number of spacecraft)
% Every entry in the cell is a sparse matrix of size Nv by K, where Nv is
% the number of vertices and K is the number of timesteps.
% observable_points_map{i}(v,t) is 1 iff point v is observable by agent i
% at time t.

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
for i_sc = sc_optimized
    % observable_points_list is a vector of size n_nonzero_entries*3.
    % the first column is the row location, in this case i_v.
    % the second column is the column location, in this case k
    % the third column is the value, in this case always 1
    [i_v_vec, k_vec, ~] = find(observable_points_map{i_sc});
    for observable_point_index = 1:length(i_v_vec)
        i_v = i_v_vec(observable_point_index);
        k = k_vec(observable_point_index);
        p = p+1;
        p_k(p) = k;
        p_v(p) = i_v;
        p_s(p) = i_sc;
        w(p) = reward_map{i_sc}(i_v,k); % rewards corresponding to each potential observation
    end
% Older code loops through all entries of the sparse matrix, not ideal
%     for k = 1:K
%         for i_v = 1:Nv
%             if observable_points_map{i_sc}(i_v, k)==1 % if vertex i_v is observable at time k
%                 p = p+1;
%                 p_k(p) = k;
%                 p_v(p) = i_v;
%                 p_s(p) = i_sc;
%                 w(p) = reward_map{i_sc}(i_v,k); % rewards corresponding to each potential observation
%             end
%         end
%     end
end

% Store sets of indicies for observations by same spacecraft at same time
num_constraint_entries = 0;
sigma_ik = cell(N,K);
for k = 1:K
    for i_sc = sc_optimized
        sigma_ik{i_sc,k} = intersect(find(p_k==k),find(p_s==i_sc));
        num_constraint_entries = num_constraint_entries+length(sigma_ik{i_sc,k});
    end
end

% Store sets of indicies for observations of the same vertex at different times (and by different spacecraft)
sigma_v = cell(1,n_observed_vertices);
for p = 1:n_observed_vertices
    sigma_v{p} = find(p_v==observable_vertices(p));
    num_constraint_entries = num_constraint_entries+length(sigma_v{p});
end

%% Define Optimization Input Arrays
% Build A as a natively sparse matrix
A = zeros(num_constraint_entries,3);

i_constraint = 0;
i_entry = 0;
for k=1:K
    for i_sc = sc_optimized
        i_constraint = i_constraint + 1;
        for entry = sigma_ik{i_sc,k}
            i_entry = i_entry +1;
            A(i_entry, :) = [i_constraint, entry, 1];
        end
    end
end
for p=1:n_observed_vertices
    i_constraint = i_constraint+1;
    for entry= sigma_v{p}
        i_entry = i_entry+1;
        A(i_entry,:) = [i_constraint, entry, 1];
    end
end
A = sparse(A(:,1), A(:,2), A(:,3), n_constraints, M);

% A = zeros(n_constraints, M); 
% i_constraint = 0;
% for k = 1:K
%     for i_sc = sc_optimized
%         i_constraint = i_constraint+1;
%         A(i_constraint , sigma_ik{i_sc,k}) = 1;
%     end
% end
% for p=1:n_observed_vertices
%     i_constraint = i_constraint+1;
%     A(i_constraint, sigma_v{p}) = 1;
% end
% A = sparse(A);
b = ones(n_constraints, 1);

% Set bounds
UB = ones(M,1);
LB = zeros(M,1);
fprintf('Total time to setup observation points problem: %0.2f\n',toc)

%% Solve 
% Define problem
tic
% Mosek has some undesirable text input due to an uncommented "options".
% We suppress it by wrapping it in evalc.
[~,X] = evalc("intlinprog(-w, 1:M, A, b, [], [], LB, UB);");
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

