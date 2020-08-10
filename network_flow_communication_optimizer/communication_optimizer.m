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

function [swarm, goal] = communication_optimizer(swarm, bandwidth_model, data_scaling_factor)

% How to best get data from a number of science spacecraft to a carrier
% s/c, given a network bandwidth model. Solves the problem as a
% single-commodity flow, leveraging MOSEK.
% Syntax:
%  communication_optimizer(swarm, bandwidth_model, data_scaling_factor)
% Inputs:
% * swarm is the object defined in swarm.m
% * bandwidth_model is a function. bandwidth_model takes in a pair of
% locations x1, x2 and returns a bandwidth (in bits) between the
% spacecraft. If the bandwidth is not specified, a quadratic model
% providing 250 kbps at 100 km is used.
% * data_scaling_factor is a factor by which the data flows are scaled
% internally. Useful to ensure the problem is well-conditioned.
% Outputs:
% * flows, a matrix of size K by N by N.
%   flows(k,i,j) contains the data flow from s/c i to s/c j at time k.
% * effective_science
% * delivered_science
% * bandwidths

if nargin<3
    data_scaling_factor = 1;
end
if nargin<2
    bandwidth_model = @(x1,x2) quadratic_comm_model(x1,x2);
end

K = swarm.get_num_timesteps();
N = swarm.get_num_spacecraft();

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
observation_flows = swarm.Observation.flow/data_scaling_factor;


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


%% Pose the problem

effective_science_finder = @(j,k) K*(j-1)+k;
flows_finder = @(k,i,j) K*N + N*N*(k-1)+N*(i-1)+j;
delivered_science_finder = @(k,c) K*N+ K*N*N + num_carriers*(k-1) + c;

num_variables = K*N+K*N*N+num_carriers*K;

% Objective
f = zeros(num_variables,1);
% swarm.Observation.priority(j,k)*effective_science(j,k).
f(1:K*N) = -reshape(swarm.Observation.priority',[1,K*N]);

ub= Inf*ones(num_variables,1);
lb= zeros(num_variables,1);

% Equality constraints:
num_equality_constraints = ...
    N*(K-1); % continuity
%    + num_carriers ... % No science delivered at t=0 - enforced by UB
%    + N*N ... % Initial flows are zero - enforced by UB
%    + num_carriers*K ... % No need for carrier to memorize - enforced by UB
%    + N; % Empty memory at the end - enforced by UB

% All inequality constraints enforced by upper bound
% num_inequality_constraints = ...
%     N*N*K + ... % Flows<bandwidths
%     K*N; % Don't do more science than is available

num_equality_entries = ...
    2*N*N*(K-1) + ...% flows appear twice in continuity, except at first and last step
    N*(K-1) + ... % Effective science appears once
    num_carriers*(K-1); % Delivered science. END of continuity
%     num_carriers + ... % Initially delivered science is zero - enforced by UB
%     N*N + ... % Initial flows are zero  - enforced by UB
%     K*num_carriers + ... % memory at carriers is zero - enforced by UB
%     N*N; % Final flows are zero - enforced by UB

% All inequality constraints enforced by upper bound
% num_inequality_entries = ...
%     K*N*N + ... % Don't exceed bandwidth constraint
%     K*N; % Don't do more science than is available

% Preallocate sparse representation of equality matrices. (Row, column,
% value)
A_eq_sparse = zeros(num_equality_entries,3);
b_eq = zeros(num_equality_constraints,1);

% A_ineq_sparse = zeros(num_inequality_entries,3);
% b_ineq = zeros(num_inequality_constraints,1);

% Indices to keep track of constraint and entry.
eq_constraint_ix = 1;
eq_entry_ix = 1;
% ineq_constraint_ix = 1;
% ineq_entry_ix = 1;

% Constraint 1: continuity

for k=1:K-1
    for j=1:N
%             sum(flows(k,:,j)) + effective_science(j,k) == sum(flows(k+1,j,:))
%             sum(flows(k,:,j)) + effective_science(j,k) == sum(flows(k+1,j,:))+delivered_science(k+1, carrier_id(j));
        for i=1:N
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, flows_finder(k,i,j), 1];
            eq_entry_ix = eq_entry_ix +1;
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, flows_finder(k+1,j,i), -1];
            eq_entry_ix = eq_entry_ix +1;
        end
        A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, effective_science_finder(j,k), 1];
        eq_entry_ix = eq_entry_ix +1;
        if swarm.Parameters.types{j}==0
            A_eq_sparse(eq_entry_ix, :) = [eq_constraint_ix, delivered_science_finder(k+1,carrier_id(j)), -1];
            eq_entry_ix = eq_entry_ix +1;
        end
        b_eq(eq_constraint_ix) = 0;
        eq_constraint_ix = eq_constraint_ix+1;
    end
end

% Constraint 5: Do not violate bandwidth and memory constraints
% Imposed _before_ the other, more stringend constraints on UB.
% dual_bandwidth_and_memory: flows<=bandwidths_and_memories;
for k=1:K
    for i=1:N
        ub(flows_finder(k,i,1:N)) = bandwidths_and_memories(k,i,1:N);
    end
end

% Constraint 2: No science delivered at t=0
% delivered_science(1, :) == 0;
for c=1:num_carriers
    ub(delivered_science_finder(1,c)) = 0.;
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
    
% Constraint 7: Don't do more science than is available
for j=1:N
    ub(effective_science_finder(j,1:K)) = observation_flows(j,1:K);
end
%     effective_science<=observation_flows;
  
%% Build the equality matrix
% First, sanity checks
if eq_constraint_ix-1 ~= num_equality_constraints
    fprintf("WARNING: number of equality constraints is unexpected (expected %d, actual %d)\n", num_equality_constraints, eq_constraint_ix-1);
end
if eq_entry_ix-1 ~= num_equality_entries
    fprintf("WARNING: number of equality entries is unexpected (expected %d, actual %d)\n", num_equality_entries, eq_entry_ix-1);
end

A_eq = sparse(A_eq_sparse(:,1), A_eq_sparse(:,2), A_eq_sparse(:,3), num_equality_constraints, num_variables);


%% Solve the problem

[X, tgoal, exitflag, output, lambdas] = cplexlp(f, [], [], A_eq, b_eq, lb, ub);

%% Unpack the variables

effective_science = reshape(X(effective_science_finder(1,1):effective_science_finder(N,K)), [K,N])';

flows = zeros(K,N,N);
for k=1:K
    for i=1:N
      flows(k,i,:) = X(flows_finder(k,i,1):flows_finder(k,i,N));
% reshape(X(flows_finder(1,1,1):flows_finder(K,N,N)),[N,N,K])
    end
end

delivered_science = reshape(X(delivered_science_finder(1,1):delivered_science_finder(K,num_carriers)), [num_carriers, K])';

dual_bandwidth_and_memory = zeros(K,N,N);
for k=1:K
    for i=1:N
        dual_bandwidth_and_memory(k,i,1:N) = lambdas.upper(flows_finder(k,i,1):flows_finder(k,i,N));
    end
end


%% Set the output

swarm.Communication.flow = flows*data_scaling_factor;
swarm.Communication.effective_source_flow = effective_science*data_scaling_factor;
swarm.Communication.bandwidths_and_memories = bandwidths_and_memories*data_scaling_factor;
swarm.Communication.dual_bandwidths_and_memories = dual_bandwidth_and_memory;  % Effective_flow is reduced by data_scaling_factor, but so is the cost.

goal = sum(sum(swarm.Observation.priority.*swarm.Communication.effective_source_flow));

% % Blueprint for recovering science_delivered. 
% flows_to_carrier = flows(:,:,end);
% flows_from_carrier = squeeze(flows(:,end,:));
% delivered_science_recovered = zeros(K,1);
% delivered_science_recovered(2:end) = sum(flows_to_carrier(1:end-1,:),2)-sum(flows_from_carrier(2:end,:),2)+effective_science(end,1:end-1)';
% 
%assert(norm(delivered_science_recovered-sum(delivered_science'))<1e-3)
