%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%     CVX implementation of a network flow-based communication optimizer. %
%     Based on the model developed by CSM's Sam Friedman and JPL's        %
%     Federico Rossi.                                                     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [swarm, goal] = observation_and_communication_optimizer(asteroid_model, swarm, bandwidth_model, data_scaling_factor)

% How to best collect observations and route data from a number of science
% spacecraft to a carrier s/c, given an observation model and a network
% bandwidth model.
% Solves the problem as a single-commodity flow, leveraging CPLEX or MOSEK.
% Syntax:
%  [swarm, goal] = observation_and_communication_optimizer(asteroid_model, swarm, bandwidth_model, data_scaling_factor)
% Inputs:
% * asteroid_model is the asteroid model created by SphericalHarmonicsGravityIntegrator_SBDT
% * swarm is the object defined in SpacecraftSwarm.m
% * bandwidth_model is a function. bandwidth_model takes in a pair of
% locations x1, x2 and returns a bandwidth (in bits) between the
% spacecraft. If the bandwidth is not specified, a quadratic model
% providing 250 kbps at 100 km is used.
% * data_scaling_factor is a factor by which the data flows are scaled
% internally. Useful to ensure the problem is well-conditioned.
% Outputs:
% swarm, the updated Swarm object
% goal, the optimal goal
prelim_tic = tic;

if nargin<4
    data_scaling_factor = NaN; % Just a placeholder, will fix once we have instrument data rates
end
if nargin<3
    bandwidth_model = @(x1,x2) quadratic_comm_model(x1,x2);
end

% The problem is TUM, so observations are supposed to be 0-1. But sometimes
% we have numerical inaccuracies. This is a ugly hack to get around that.
observation_threshold = 1e-4;

K = swarm.get_num_timesteps();
N = swarm.get_num_spacecraft();

%% Get observability and reward
% First, let's find out what points are observable
% observable_points is a N by K cell matrix. observable_points(i,k) is the
% list of points that spacecraft i can observe at time k

op_tic = tic;
observable_points = swarm.Observation.observable_points; % Get an empty container of the right size from the constructor

for i_sc = swarm.which_trajectories_set()
    for i_time = 1:K
        observable_points{i_sc, i_time} = get_observable_points(asteroid_model, swarm, i_time, i_sc) ;
    end
end
swarm.Observation.observable_points = observable_points;

observability_time = toc(op_tic);
 
% Next, let's define the coverage reward map, i.e. the reward for a point
% (if it can be observed)
% Reward_map is a list of N cells. Each cell contains a Nv x K matrix
% listing the reward for observing vertex v at time t.
rtic = tic;
reward_map = get_coverage_reward_map(asteroid_model, swarm);
reward_time = toc(rtic);

% We also need the instrument data rates
drtic = tic;

data_rates = zeros(N,K);
for i_sc=1:N
    data_rate_per_point = get_data_rates(swarm.Parameters.types{i_sc},asteroid_model);
    
    if ~isempty(data_rate_per_point)
        data_rates(i_sc,1:end-1) = data_rate_per_point*ones(1,K-1);
        % Cheat at the end - information collected there can't be
        % transmitted anyway
        data_rates(i_sc,end) = data_rates(i_sc,end-1);
    end
    
end
datarate_time = toc(drtic);

% Numerical conditioning is critical to get a good solution from the
% communication optimizer. Mosek 9 will deal with ill-conditioned problems,
% but just about every free solver (and Mosek 8) will report infeasibility
% if appropriate scaling is not applied.
% Here, we attempt to make a guess at the "mean" information flow by
% taking the mean instrument data rate. We use the mean, as opposed
% to the median, because we do _not_ want to throw out outlier instruments
% generating a disproportionate amount of data.

if isnan(data_scaling_factor)
    data_scaling_factor = mean(mean(data_rates(data_rates>0)));
end

% We also make a list of all the vertices that can be observed ever. We
% will have one constraint per vertex saying "only observe this once"
observabletic = tic;
observable_vertices = [];
for i_sc = 1:N
    for k = 1:K
        observable_vertices = unique([observable_vertices, observable_points{i_sc, k}]);
    end
end
n_observed_vertices = length(observable_vertices);
observable_points_time = toc(observabletic);

%% Compute bandwidths

% We will never need more than this memory
max_memory = sum(sum(swarm.Observation.flow));

if isa(bandwidth_model,'function_handle')
    bandwidths_and_memories = zeros(K,N,N);
    for k=1:K-1
        for i=1:N
            for j=1:N
                if j~=i
                    bandwidths_and_memories(k,i,j) = bandwidth_model(swarm.abs_trajectory_array(k,1:3,i), swarm.abs_trajectory_array(k,1:3,j))* (swarm.sample_times(k+1)-swarm.sample_times(k));
                elseif ~isnan(swarm.Parameters.available_memory(i)) && ~isinf(swarm.Parameters.available_memory(i))
                    bandwidths_and_memories(k,i,j) = swarm.Parameters.available_memory(i);
                else
                    bandwidths_and_memories(k,i,j) = max_memory;
                end
            end
        end
    end
elseif isa(bandwidth_model,'double')
    bandwidths_and_memories = bandwidth_model;
    assert(size(bandwidths_and_memories,1)==K)
    assert(size(bandwidths_and_memories,2)==N)
    assert(size(bandwidths_and_memories,3)==N)
else
    error("Bandwidth model type not recognized (should be function handle or double)")
end

%% Scale the problem
bandwidths_and_memories = bandwidths_and_memories/data_scaling_factor;
data_rates = data_rates./data_scaling_factor;

%% Sanity check: how many carriers are there?
num_carriers = 0;
carrier_id = zeros(N,1); % This is a map from spacecraft_number to carrier_number
carrier_index = 1;
for j=1:N
    if swarm.Parameters.types{j} == 0
        num_carriers = num_carriers + 1;
        carrier_id(j) = carrier_index;
        carrier_index = carrier_index + 1;
    end
end
assert(num_carriers>0, "ERROR: no designated carrier spacecraft. Communication optimization problem will fail")
if num_carriers>1
    disp("There is more than one carrier spacecraft. Is this intended?")
end

problem_prelim_time = toc(prelim_tic);
%% Pose the problem
setup_tic = tic;

% The code below merges relay optimization code from
% communication_optimizer and observation optimization code from
% swarm_points_optimizer. Style somewhat suffers. To aid in reading the
% code, keep in mind that:
% All variables are flattened in a linear index. Observation decision
% variables come first, followed by flows and delivered science. 
% For observations, we have a mapping from decision variable to (time,
% spacecraft, vertex) encoded in the vectors p_k, p_v, and p_s.
% Conversely, for flows and delivered_science, we provide a mapping from
% (time, spacecraft, spacecraft) (resp. (time)) to the index of the
% decision variable through flows_finder() and delivered_science_finder().
% When indexing observations, we liberally use find() on the vectors p_* .
% When indexing flows and delivered science, we loop through the indices
% and call the *_finder functions.

% First we define the index for the observation variables.

% Total number of potential observations = observation decision variable = M
M = 0;
for i_time = 1:K-1
    for i_sc = swarm.which_trajectories_set()
        M = M+length(observable_points{i_sc, i_time});
    end
end

% Create pointer vectors to keep track of the time, vertex, and agent
% corresponding to the pth position in the decision variable
p = 0; % This index will run over all possible observations
p_k=zeros(1,M); % Decision variable m corresponds to time p_k(m)
p_v=zeros(1,M); % Decision variable m corresponds to vertex p_v(m)
p_s=zeros(1,M); % Decision variable m corresponds to spacecraft p_s(m)
w=zeros(1,M);   % w(m) is the reward for decision variable m

for k = 1:K-1
    for i_sc = swarm.which_trajectories_set()
        for i_v = observable_points{i_sc, k}
            p = p+1;
            p_k(p) = k;
            p_v(p) = i_v;
            p_s(p) = i_sc;
            w(p) = reward_map{i_sc}(i_v,k); 
        end
    end
end

% assert(p==M, "ERROR: mismatch in length of observation variables")

% In the interest of speed, we also build an index for the variables
% corresponding to time k and spacecraft j. We will use this in two
% constraints below.
sc_time_observation_opportunities = cell(N,K);
for j=1:N
    for k=1:K
        sc_time_observation_opportunities{j,k} = intersect(find(p_k==k), find(p_s==j));
    end
end

% Next, we define how to find flows and delivered science

flows_finder = @(k,i,j) M + N*N*(k-1)+N*(i-1)+j;

delivered_science_finder = @(k,c) M+ K*N*N + num_carriers*(k-1) + c;

% The overall number of variables is
num_variables = M +K*N*N+num_carriers*K;

% Objective - remember we MINIMIZE
f = zeros(num_variables,1);
f(1:M) = -w;
% sum(sum(sum(reward_with_observability_matrix.*observations)))

% Initialization of upper and lower bounds
ub= Inf*ones(num_variables,1);
lb= zeros(num_variables,1);

% Number of constraints and entries:
num_equality_constraints = ...
    N*(K-1); % continuity

num_inequality_constraints = ...
    N*K + ... % Every sc observes at most one point per unit time
    n_observed_vertices; % Every vertex is observed at most once

num_equality_entries = ...
    2*N*N*(K-1) + ...% flows appear twice in continuity, except at first and last step
    M + ... % Every observation appears once
    num_carriers*(K-1); % Delivered science. END of continuity

num_inequality_entries = ...
    M + ... % Every sc observes at most one point per unit time
    M ; % Every vertex is observed at most once

% Preallocate sparse representation of equality matrices. (Row, column,
% value)
A_eq_sparse = zeros(num_equality_entries,3);
b_eq = zeros(num_equality_constraints,1);
A_ineq_sparse = zeros(num_inequality_entries,3);
b_ineq = zeros(num_inequality_constraints,1);

% Indices to keep track of constraint and entry.
eq_constraint_ix = 1;
eq_entry_ix = 1;
ineq_constraint_ix = 1;
ineq_entry_ix = 1;

% Inequality constraint 1: Observe every point at most once
%     sum(sum(observations(v,:,:))) <=1
for v_ix = 1:length(observable_vertices)
    v = observable_vertices(v_ix);
    v_observation_opportunities = find(p_v==v);  % Indices of the observation decision variables for v
    % Above, we use the fact that observation variables are at the head
    % of the decision vector.
    for v_obs_ix = 1:length(v_observation_opportunities)
        v_obs = v_observation_opportunities(v_obs_ix);
        A_ineq_sparse(ineq_entry_ix,:) = [ineq_constraint_ix, v_obs, 1];
        ineq_entry_ix = ineq_entry_ix +1;
    end
    b_ineq(ineq_constraint_ix) = 1;
    ineq_constraint_ix = ineq_constraint_ix+1;
end

% Inequality constraint 2: every spacecraft observes at most one point per
% unit time
for j=1:N
    for k=1:K
%         sum(observations(:,j,k)) <=1
        for obs = sc_time_observation_opportunities{j,k}
            A_ineq_sparse(ineq_entry_ix, :) = [ineq_constraint_ix, obs, 1];
            ineq_entry_ix = ineq_entry_ix + 1;
        end
        b_ineq(ineq_constraint_ix) = 1;
        ineq_constraint_ix = ineq_constraint_ix+1;
    end
end
    
% Equality constraint 1: continuity
for k=1:K-1
    for j=1:N
        % sum(flows(k,:,j)) + sum(observations(:,j,k))*data_rates(j,k) == sum(flows(k+1,j,:))
        % sum(flows(k,:,j)) + sum(observations(:,j,k))*data_rates(j,k) == sum(flows(k+1,j,:))+delivered_science(k+1, carrier_id(j));
        % Flows in and out
        for i=1:N
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, flows_finder(k,i,j), 1];
            eq_entry_ix = eq_entry_ix +1;
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, flows_finder(k+1,j,i), -1];
            eq_entry_ix = eq_entry_ix +1;
        end
        
        % Add the observations at this time
        for obs = sc_time_observation_opportunities{j,k}
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, obs, data_rates(j,k)];
            eq_entry_ix = eq_entry_ix +1;
        end        
        
        % Sink at the carrier
        if swarm.Parameters.types{j}==0
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, delivered_science_finder(k+1,carrier_id(j)), -1];
            eq_entry_ix = eq_entry_ix +1;
        end
        
        b_eq(eq_constraint_ix) = 0;
        eq_constraint_ix = eq_constraint_ix+1;
    end
end

% Constraint 2: No science delivered at t=0
% delivered_science(1, :) == 0;
for c=1:num_carriers
    ub(delivered_science_finder(1,c)) = 0.;
end

% Constraint 5: Do not violate bandwidth and memory constraints
% dual_bandwidth_and_memory: flows<=bandwidths_and_memories;
for k=1:K
    for i=1:N
        ub(flows_finder(k,i,1:N)) = bandwidths_and_memories(k,i,1:N);
    end
end

% Constraint 3: Initial flows are nil - don't make up information
% flows(1,:,:) == 0;
for i=1:N
%     for j=1:N
        ub(flows_finder(1,i,1:N)) = 0;
%     end
end

% Constraint 4: No need for carrier to memorize
for j=1:N
    if swarm.Parameters.types{j}==0      
        for k=1:K      
            ub(flows_finder(k,j,j)) = 0;
        end
    end
end

% Constraint 6: Empty memory at the end
for i=1:1:N
    if ~isnan(swarm.Parameters.available_memory(i)) && ~isinf(swarm.Parameters.available_memory(i))
%         flows(K,i,i) == 0;
        ub(flows_finder(K,i,i)) = 0;
    end
end
  
%% Build the equality matrix
% First, sanity checks
if eq_constraint_ix-1 ~= num_equality_constraints
    fprintf("ERROR: number of equality constraints is unexpected (expected %d, actual %d)\n", num_equality_constraints, eq_constraint_ix-1);
end
if eq_entry_ix-1 ~= num_equality_entries
    fprintf("ERROR: number of equality entries is unexpected (expected %d, actual %d)\n", num_equality_entries, eq_entry_ix-1);
end

if ineq_constraint_ix-1 ~= num_inequality_constraints
    fprintf("ERROR: number of inequality constraints is unexpected (expected %d, actual %d)\n", num_inequality_constraints, ineq_constraint_ix-1);
end
if ineq_entry_ix-1 ~= num_inequality_entries
    fprintf("ERROR: number of inequality entries is unexpected (expected %d, actual %d)\n", num_inequality_entries, ineq_entry_ix-1);
end

A_eq = sparse(A_eq_sparse(:,1), A_eq_sparse(:,2), A_eq_sparse(:,3), num_equality_constraints, num_variables);

A_ineq = sparse(A_ineq_sparse(:,1), A_ineq_sparse(:,2), A_ineq_sparse(:,3), num_inequality_constraints, num_variables);

problem_setup_time = toc(setup_tic);
%% Solve the problem
solve_tic = tic;
[X, goal, exitflag, output, lambdas] = cplexlp(f, A_ineq, b_ineq, A_eq, b_eq, lb, ub);
% can also use linprog (esp. the MOSEK version) with identical syntax
problem_solve_time = toc(solve_tic);
%% Unpack the variables
unpack_tic = tic;
observations_flat = X(1:M);

flows = zeros(K,N,N);
for k=1:K
    for i=1:N
      flows(k,i,:) = X(flows_finder(k,i,1):flows_finder(k,i,N));
% reshape(X(flows_finder(1,1,1):flows_finder(K,N,N)),[N,N,K])
    end
end

% delivered_science = reshape(X(delivered_science_finder(1,1):delivered_science_finder(K,num_carriers)), [num_carriers, K])';
dual_bandwidth_and_memory = zeros(K,N,N);
for k=1:K
    for i=1:N
        dual_bandwidth_and_memory(k,i,1:N) = lambdas.upper(flows_finder(k,i,1):flows_finder(k,i,N));
    end
end

%% Set the output

% Observations

swarm.Observation.observed_points = zeros(N,K); % Points observed
swarm.Observation.flow = zeros(N,K);  % Bits generated by observations.
swarm.Observation.priority = zeros(N,K);  % Reward


swarm.Observation.flow = zeros(N,K);
swarm.Observation.priority = zeros(N,K);
for obs_index = 1:M
    k = p_k(obs_index); % Observation time
    v = p_v(obs_index); % Observed point
    j = p_s(obs_index); % Observing spacecraft
    reward = w(obs_index); % Reward
    if observations_flat(obs_index)>observation_threshold
        if swarm.Observation.observed_points(j,k) ~= 0
            fprintf("SC %d, time %d: more than one point observed! (old: %d w. total flow %f, new: %d with flow %f))\n",j,k,swarm.Observation.observed_points(j,k),swarm.Observation.flow(j,k),v,observations_flat(obs_index)*(data_rates(j,k)*data_scaling_factor))
        end
        swarm.Observation.observed_points(j,k) = v;
    end
    % Add to observation flow
    swarm.Observation.flow(j,k) = swarm.Observation.flow(j,k)+ observations_flat(obs_index)*(data_rates(j,k)*data_scaling_factor);
    swarm.Observation.priority(j,k) = swarm.Observation.priority(j,k)+ observations_flat(obs_index)*reward;
    % There should be only one nonzero entry per (sc, time), due to TUM
end

% Flows

swarm.Communication.flow = flows*data_scaling_factor;
swarm.Communication.effective_source_flow = swarm.Observation.flow;
swarm.Communication.bandwidths_and_memories = bandwidths_and_memories*data_scaling_factor;
swarm.Communication.dual_bandwidths_and_memories = dual_bandwidth_and_memory;  % Effective_flow is reduced by data_scaling_factor, but so is the cost.

problem_unpack_time = toc(unpack_tic);

goal = -goal;

% % Blueprint for recovering science_delivered. 
% flows_to_carrier = flows(:,:,end);
% flows_from_carrier = squeeze(flows(:,end,:));
% delivered_science_recovered = zeros(K,1);
% delivered_science_recovered(2:end) = sum(flows_to_carrier(1:end-1,:),2)-sum(flows_from_carrier(2:end,:),2)+effective_science(end,1:end-1)';
% 
%assert(norm(delivered_science_recovered-delivered_science)<1e-3)

% TODO BUGFIX: What are the units for the data rates? Definitely not bits
% per second!! I don;t think we will be collecting GBPS. More like
% GBPobservation, so maybe we do not need to multiply by time.
