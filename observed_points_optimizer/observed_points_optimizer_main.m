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

function [map, observation_flow, observed_points, priority, reward] = observed_points_optimizer_main(varargin)
%OBSERVED_POINTS_OPTIMIZER  Determines which points the spacecraft will
%observe, given their orbits.
%
%   Syntax: [map, observation_flow, observed_points, priority, reward] = observed_points_optimizer_main(SmallBodyParameters, time_vector, sc_obs_type, orbits, *observed_points) 
%    *optional input 
%
%   Inputs: 
%    - SmallBodyParameters: Is a struct containing information about the 
%      asteroid. 
%    - time_vector [seconds]: [1 x K] vector containing the times at which the s/c
%      orbits are sampled.
%    - sc_obs_type: [1 x N] vector containing the observation type indicies
%      which indicate the observation related capabilities of the s/c 
%       - sc_obs_type(n) = 0 for carrier spacecraft
%       - sc_obs_type(n) = 1 for all other spacecraft 
%    - orbits: [K x 6 x N] array containing the best orbits of the s/c
%    - *observed_points: [N x K] array indicating the points previously
%      observed by the s/c, if determined.
%       - observed_points(n,k) = -1 if the observation status of s/c n at
%       time k is undetermined. Only undetermined points will be evaluated
%       by this function.
%       - if this input is not passed into the function, all points will be
%       given an undetermined status 
% 
%   Outputs: 
%    - map: [N x V] array 
%       - map(n,v) indicates the number of times s/c n has observed 
%         vertex v
%    - observation_flow [bits/second]: [N x K] array
%       - observation_flow(n,k) contains the data taken in by s/c
%         n at time k 
%    - observed_points: [N x K] array 
%       - observed_points(n,k) contains either the point observed by 
%         s/c n at time k if an observation is made, or 0 otherwise
%    - priority [reward/(bit/second)]: [N x K] array
%       - priority(n,k) is the reward (value) accociated with 
%         observation_flow(n,k)
%    - reward: sum(sum(priority.*observation_flow))

%% Define Parameters

bits_per_point = 8*0.4*1e9; % 0.4GB, data collected at each point

%% Setup

% Inputs 
SmallBodyParameters = varargin{1};
time_vector = varargin{2};
sc_obs_type = varargin{3};
orbits = varargin{4};
K = length(time_vector); % number of time steps
N = size(orbits,3); % number of spacecraft
if nargin < 5
    observed_points = -1.*ones(N,K); % non-recursive use 
else
    observed_points = varargin{5}; % recursive use 
end

% Load Sun State from time_vector
sun_state_array = zeros(K, 3);

% Propagate Asteroid State
theta_SB = zeros(1, K);
theta_SB(1) = SmallBodyParameters.rotation_at_t0;
for i_time = 1:(K-1)
    theta_SB(i_time+1) = theta_SB(i_time) + SmallBodyParameters.rotation_rate*(time_vector(i_time+1)-time_vector(i_time));
end

%% Get Set of Feasible Observation Points at Each Timestep
observable_points = cell(K,N);
for i_time = 1:K
    % Update the vertices on the asteroid to match the current state
    time_since_epoch = time_vector(i_time)-time_vector(1);
    R = rotation_matrix_at_t( time_since_epoch, SmallBodyParameters.rotation_rate, SmallBodyParameters.rotation_at_t0 );
    asteroid_vertices = transpose(R*transpose(SmallBodyParameters.ShapeModel.Vertices));
    
    for i_sc = 1:N
        if observed_points(i_sc, i_time) == -1 % only consider points that have not been evaluated yet
            if sc_obs_type(i_sc) == 0
                observable_points{i_time, i_sc} = []; % carrier spacecraft not allowed to observe
            else
                observable_points{i_time, i_sc} = get_observable_points(SmallBodyParameters.radius, asteroid_vertices, orbits(i_time, 1:3, i_sc ), sun_state_array(i_time, :), sc_obs_type(i_sc)) ;
            end
        end
    end
end

%% Choose Points to Observe from Feasible Set
for i_sc = 1:N
    for i_time = 1:K
        if observed_points(i_sc, i_time) == -1 % only consider points that have not been evaluated yet
            %% Choose observation point
            if isempty(observable_points{i_time, i_sc})
                observed_points(i_sc, i_time) = 0;
            else
                observed_points(i_sc, i_time) = observable_points{i_time, i_sc}(1); % just take first point in feasible set  
            end
        end
    end
end

%% Define Priority - i.e. observation reward / bit
priority = get_coverage_reward(observed_points, sc_obs_type);

%% Fill out the observation flow and map, using info from observed_points
observation_flow = zeros(N,K);
map = zeros(N,size(SmallBodyParameters.ShapeModel.Vertices,1));
for i_sc = 1:N
    for i_time = 1:K
        % Store data flow
        observation_flow(i_sc, i_time) = bits_per_point*sign(observed_points(i_sc, i_time)); % either equal to zero or bits_per_point
        
        % Update map
        for i_pt = observed_points(i_sc, i_time)
            if observed_points~=0 % if a point was observed, add it to the map
                map(i_sc, i_pt) = map(i_sc, i_pt)+1;
            end
        end
    end
end

%% Define Total Reward for each Orbit
reward = zeros(i_sc, 1);
for i_sc = 1:N
    reward(i_sc,1) = sum(observation_flow(i_sc, :).*priority(i_sc, :));
end

end

