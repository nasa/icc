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

function [flows, effective_science, delivered_science, bandwidths_and_memories, dual_bandwidth_and_memory] = communication_optimizer(spacecraft, bandwidth_model)

% How to best get data from a number of science spacecraft to a carrier
% s/c, given a network bandwidth model. Solves the problem as a
% single-commodity flow, leveraging CVX.
% Syntax:
%  [flows, effective_science, delivered_science, bandwidths, dual_bandwidth, dual_memory] = communication_optimizer(spacecraft, bandwidth_model)
% Inputs:
% * spacecraft, a struct with four fields.
% - time is a vector of length K. It contains the time at which the s/c
% orbits are sampled.
% - orbits is a list of N vectors of length K.
% orbits{i} is the vector of locations of spacecraft i.
% orbits{i}(:, k) is the location of s/c i at time time(k).
% - science is a matrix of size N-by-K.
% science(i,k) is the amount of science (in bits) produced by
% s/c k at time i.
% - priority is a matrix of size N-by-K.
% priority(i,k) is the value of one bit of science produced by
% s/c k at time i.
% - memory, a vector of size N. Memory(i) is the memory of s/c i.
% * bandwidth_model is a function. bandwidth_model takes in a pair of
% locations x1, x2 and returns a bandwidth (in bits) between the
% spacecraft. If the bandwidth is not specified, a quadratic model
% providing 250 kbps at 100 km is used.
% Outputs:
% * flows, a matrix of size K by N by N.
%   flows(k,i,j) contains the data flow from s/c i to s/c j at time k.
% * effective_science
% * delivered_science
% * bandwidths

if nargin<2
    bandwidth_model = @(x1,x2) min(250000 * (100000/norm(x2-x1,2))^2, 100*1e6); 
end

K = length(spacecraft.time);
N = length(spacecraft.orbits);

% We will never need more than this memory
max_memory = sum(sum(spacecraft.science));

% Compute bandwidths
bandwidths_and_memories = zeros(K,N,N);
for k=1:K-1
    for i=1:N
        for j=1:N
            if j~=i
                bandwidths_and_memories(k,i,j) = bandwidth_model(spacecraft.orbits{i}(:,k), spacecraft.orbits{j}(:,k))* (spacecraft.time(k+1)-spacecraft.time(k));
            elseif ~isnan(spacecraft.memory(i)) && ~isinf(spacecraft.memory(i))
                bandwidths_and_memories(k,i,j) = spacecraft.memory(i);
            else
                bandwidths_and_memories(k,i,j) = max_memory;
            end
        end
    end
end

% Pose the problem
cvx_begin quiet
    variable effective_science(N,K) nonnegative
    variable flows(K,N,N) nonnegative
    variable delivered_science(K) nonnegative
    dual variable dual_bandwidth_and_memory
    
    maximize sum(sum(spacecraft.priority.*effective_science))
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
    flows(1,:,:) == 0
    
%     % Continuity for carrier
%     for k=1:K
%        sum(flows(k,:,N)) == delivered_science(k);  % Carrier does not transmit or generate science
%     end
%     % No need for carrier to memorize
%     for k=1:K
%         flows(k,N,N) == 0;
%     end
    
    % Do not violate bandwidth and memory constraints

    dual_bandwidth_and_memory: flows<=bandwidths_and_memories;
    
    % Empty memory at the end
    for i=1:1:N
        if ~isnan(spacecraft.memory(i)) && ~isinf(spacecraft.memory(i))
            flows(K,i,i) == 0;
        end
    end
    
    % Don't do more science than is available
    effective_science<=spacecraft.science;
    
cvx_end