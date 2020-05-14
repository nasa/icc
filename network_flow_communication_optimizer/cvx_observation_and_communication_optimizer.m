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

function [swarm, goal] = cvx_observation_and_communication_optimizer(asteroid_model, swarm, bandwidth_model, data_scaling_factor)

% How to best collect observations and route data from a number of science
% spacecraft to a carrier s/c, given an observation model and a network
% bandwidth model.
% Solves the problem as a single-commodity flow, leveraging CVX.
% A _much_ faster version that uses CPLEX or MOSEK directly is available as
% observation_and_communication_optimizer.m, with the same syntax.
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

if nargin<4
    data_scaling_factor = 1;
end
if nargin<3
    bandwidth_model = @(x1,x2) quadratic_comm_model(x1,x2);
end

% The problem is TUM, so observations are supposed to be 0-1. But sometimes
% we have numerical inaccuracies. This is a ugly hack to get around that.
observation_threshold = 1e-4;

K = swarm.get_num_timesteps();
N = swarm.get_num_spacecraft();
Nv = size(asteroid_model.BodyModel.shape.faceCenters,1);

%% Get observability and reward
% First, let's find out what points are observable
% observable_points is a N by K cell matrix. observable_points(i,k) is the
% list of points that spacecraft i can observe at time k

observable_points = swarm.Observation.observable_points;
for i_time = 1:K
    for i_sc = swarm.which_trajectories_set()
        if ismember(0,swarm.Parameters.types{i_sc})
            observable_points{i_sc, i_time} = []; % carrier spacecraft does not observe anything
        else
            observable_points{i_sc, i_time} = get_observable_points(asteroid_model, swarm, i_time, i_sc) ;
        end
    end
end

% Create observable points map, same data as observable_points but in more
% convenient format.
% observable_points_map is a list of N cells. Each cell contains a Nv x K
% sparse matrix with value 1 iff the vertex v can be observed by sc i at
% time k.
observable_points_map = cell(1,N);
for i_sc = swarm.which_trajectories_set()
    observable_points_map{i_sc} = sparse(zeros(Nv,K));
    for i_time = 1:K
        observable_points_map{i_sc}(observable_points{i_sc, i_time}, i_time) = 1;
    end
end

swarm.Observation.observable_points = observable_points;

% Next, let's define the coverage reward map, i.e. the reward for a point
% (if it can be observed)
% Reward_map is a list of N cells. Each cell contains a Nv x K matrix
% listing the reward for observing vertex v at time t.
reward_map = get_coverage_reward_map(asteroid_model, swarm);

% Finally, we define the reward including observability.

% reward_with_observability_map = cell(1,N);
% for i = 1:N
%     reward_with_observability_map{i} = sparse(observable_points_map{i}.*reward_map{i});
% end
reward_with_observability_matrix = zeros(Nv,N,K);
for i = 1:N
    reward_with_observability_matrix(:,i,:) = (observable_points_map{i}.*reward_map{i});
%     if sum(sum(reward_with_observability_matrix(:,i,:)))>0
%         fprintf("Total available rewards for SC %d: %f!\n", i,sum(sum(reward_with_observability_matrix(:,i,:))))
%     else
%         fprintf("Nothing to observe ever for SC %d!\n", i)
%     end
end

reward_with_observability_matrix = reward_with_observability_matrix(:,:,1:end-1);

% We also need the data rates

data_rates = zeros(N,K);
for i_sc=1:N
    [~, ~, ~, dr] = get_instrument_constraints(swarm.Parameters.types{i_sc});
    if ~isempty(dr)
        data_rates(i_sc,1:end-1) = dr*diff(swarm.sample_times);
        % Cheat at the end - information collected there can't be
        % transmitted anyway
        data_rates(i_sc,end) = data_rates(i_sc,end-1);
    end
end

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

%% Pose the problem
cvx_begin
    variable observations(Nv,N,K-1) nonnegative
    variable flows(K,N,N) nonnegative
    variable delivered_science(K, num_carriers) nonnegative
    dual variable dual_bandwidth_and_memory
    
    maximize sum(sum(sum(reward_with_observability_matrix.*observations)))
    %maximize sum(delivered_science)
    
    subject to
    % Observe every point at most once
    for v=1:Nv
        sum(sum(observations(v,:,:))) <=1
    end
%     Observe at most one point per unit time
    for j=1:N
        for k=1:K-1
            sum(observations(:,j,k)) <=1
        end
    end
    
    % Continuity for all except carrier
    for k=1:K-1
        for j=1:N
            if swarm.Parameters.types{j}~=0
                sum(flows(k,:,j)) + sum(observations(:,j,k))*data_rates(j,k) == sum(flows(k+1,j,:))
            else
                sum(flows(k,:,j)) + sum(observations(:,j,k))*data_rates(j,k) == sum(flows(k+1,j,:))+delivered_science(k+1, carrier_id(j));
            end
        end
    end
    
    % No communicating beyond time limit unless it is with carrier -
    % enforced by bandwidth limits at K.
    
    % No science delivered at t=0
    delivered_science(1) == 0;
    
    % Initial flows are nil - don't make up information
    flows(1,:,:) == 0;
    
    % No need for carrier to memorize
    for k=1:K
        for j=1:N
            if swarm.Parameters.types{j}==0            
                flows(k,j,j) == 0;
            end
        end
    end
    
    % Do not violate bandwidth and memory constraints

    dual_bandwidth_and_memory: flows<=bandwidths_and_memories;
    
    % Empty memory at the end
    for i=1:1:N
        if ~isnan(swarm.Parameters.available_memory(i)) && ~isinf(swarm.Parameters.available_memory(i))
            flows(K,i,i) == 0;
        end
    end
    
cvx_end

swarm.Observation.observed_points = zeros(N,K); % Points observed
swarm.Observation.flow = zeros(N,K);  % Bits generated by observations.
swarm.Observation.priority = zeros(N,K);  % Reward



for j=1:N
    for k=1:K-1
        observed_pt = find(observations(:,j,k)>observation_threshold);
        swarm.Observation.flow(j,k) = data_rates(j,k)*data_scaling_factor*sum(observations(:,j,k));
        if ~isempty(observed_pt)
            if length(observed_pt)>1
                for opt =1:length(observed_pt)
                    fprintf("SC %d, time %d, point %d: flow %d\n",j,k,observed_pt(opt), observations(observed_pt(opt),j,k));
                end
            end
%             assert(length(observed_pt)==1, "ERROR: multiple points observed per time step")
            swarm.Observation.priority(j,k) = reward_with_observability_matrix(observed_pt(1),j,k);
            swarm.Observation.observed_points(j,k) = observed_pt(1);
        end
    end
end

swarm.Communication.flow = flows*data_scaling_factor;
swarm.Communication.effective_source_flow = swarm.Observation.flow;
swarm.Communication.bandwidths_and_memories = bandwidths_and_memories*data_scaling_factor;
swarm.Communication.dual_bandwidths_and_memories = dual_bandwidth_and_memory;  % Effective_flow is reduced by data_scaling_factor, but so is the cost.

goal = sum(sum(sum(reward_with_observability_matrix.*observations)));

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

%% TODO for a faster V2
% build the matrix by hand
% Use smarter indexing: only consider vertices with nonzero reward
% Also only use comm links with nonzero bandwidth.
% You may want to make liberal use of find()
% For instance, find(reward_with_observability_matrix) returns a set of
% column vectors for the indices and values of
% reward_with_observability_matrix. Then iterating through the column
% vectors gives a natural index for the feasible observations.
% What you can do is: find(), then define a linear index 1:1:length(...),
% then build a new sparse matrix where the matrix values correspond to the
% index element. Like
% [rvec, nvec, tvec, content] = find(reward_with_observability_matrix)
% obs_index_linear = 1:1:length(rvec)
% obs_index = sparse(rvec, nvec, tvec, obs_index_linear)
% Plot twist! MATLAB does not support sparse 3D matrices. That is why Mark
% was using cells.
% The dumb thing is to fill in the matrix that says "observation(r,i,t) is
% index d" manually with a for loop. This is painful but may actually not
% be _that_ terrible.
