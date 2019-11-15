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
% single-commodity flow, leveraging CVX.
% Syntax:
%  [flows, effective_science, delivered_science, bandwidths, dual_bandwidth, dual_memory] = communication_optimizer(swarm, bandwidth_model,data_scaling_factor)
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

% We will never need more than this memory
max_memory = sum(sum(swarm.Observation.flow));

% Compute bandwidths
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
% Scale the prblem
bandwidths_and_memories = bandwidths_and_memories/data_scaling_factor;
observation_flows = swarm.Observation.flow/data_scaling_factor;


% Pose the problem
cvx_begin
    variable effective_science(N,K) nonnegative
    variable flows(K,N,N) nonnegative
    variable delivered_science(K) nonnegative
    dual variable dual_bandwidth_and_memory
    
    maximize sum(sum(swarm.Observation.priority.*effective_science))
    %maximize sum(delivered_science)
    
    subject to
    % Continuity for all except carrier
    for k=1:K-1
        for j=1:N
            if j~=N
                sum(flows(k,:,j)) + effective_science(j,k) == sum(flows(k+1,j,:))
            else
                sum(flows(k,:,j)) + effective_science(j,k) == sum(flows(k+1,j,:))+delivered_science(k+1);
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
        flows(k,N,N) == 0;
    end
    
    % Do not violate bandwidth and memory constraints

    dual_bandwidth_and_memory: flows<=bandwidths_and_memories;
    
    % Empty memory at the end
    for i=1:1:N
        if ~isnan(swarm.Parameters.available_memory(i)) && ~isinf(swarm.Parameters.available_memory(i))
            flows(K,i,i) == 0;
        end
    end
    
    % Don't do more science than is available
    effective_science<=observation_flows;
    
cvx_end

swarm.Communication.flow = flows*data_scaling_factor;
swarm.Communication.effective_source_flow = effective_science*data_scaling_factor;
swarm.Communication.bandwidths_and_memories = bandwidths_and_memories*data_scaling_factor;
swarm.Communication.dual_bandwidths_and_memories = dual_bandwidth_and_memory;  % Effective_flow is reduced by data_scaling_factor, but so is the cost.

goal = sum(sum(swarm.Observation.priority.*swarm.Communication.effective_source_flow));

% Blueprint for recovering science_delivered. 
flows_to_carrier = flows(:,:,end);
flows_from_carrier = squeeze(flows(:,end,:));
delivered_science_recovered = zeros(K,1);
delivered_science_recovered(2:end) = sum(flows_to_carrier(1:end-1,:),2)-sum(flows_from_carrier(2:end,:),2)+effective_science(end,1:end-1)';

%assert(norm(delivered_science_recovered-delivered_science)<1e-3)
