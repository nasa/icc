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

function [swarm_map, swarm_observation_flow, swarm_observed_points, swarm_priority, swarm_reward, swarm_orbits] = monte_carlo_coverage_optmizer(SmallBodyParameters, time_vector, sc_obs_type, n_trial_orbits)
%MONTE_CARLO_COVERAGE_OPTIMIZER  Generates a set of random orbits, and
%assigns one of these orbits to each instrument carrying spacecraft in the
%swarm. 
%   The orbits are chosen sequentially (one spacecraft after another) 
%   such that the overall coverage reward is maximized on each iteration. 
%
%   Syntax: [swarm_map, swarm_observation_flow, swarm_observed_points, swarm_priority, swarm_reward, swarm_orbits] = monte_carlo_coverage_optmizer(SmallBodyParameters, time_vector, sc_obs_type, n_trial_orbits)
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
%    - n_trial_orbits: number of trial orbits to used in the Monte Carlo
% 
%   Outputs: 
%    - swarm_map: [N x V] array 
%       - swarm_map(n,v) indicates the number of times s/c n has
%         observed vertex v
%    - swarm_observation_flow [bits/second]: [N x K] array
%       - observation_flow(n,k) contains the data taken in by s/c
%         n at time k 
%    - swarm_observed_points: [N x K] array 
%       - swarm_observed_points(n,k) contains the point observed by s/c
%         n at time k 
%    - swarm_priority [reward/(bit/second)]: [N x K] array
%       - swarm_priority(n,k) is the reward (value) accociated with 
%         observation_flow(n,k)
%    - swarm_reward: sum(sum(priority.*observation_flow))
%    - swarm_orbits: [K x 6 x N] array containing the best orbits found in
%      the Monte Carlo

n_spacecraft = length(sc_obs_type);
num_points = size(SmallBodyParameters.ShapeModel.Vertices,1);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Integrate Trajectories for Trial Orbits                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ErosGravity = SphericalHarmonicsGravityIntegrator(SmallBodyParameters.SphericalModel,SmallBodyParameters.rotation_rate, 0, SmallBodyParameters.ShapeModel, @ode23tb);

% Initialize other spacecraft states (i.e. swarm state)
sc_initial_state_array = initialize_random_orbits(n_trial_orbits, SmallBodyParameters);

% Integrate for instrument spacecraft
trial_orbits = zeros( length(time_vector), 6, n_spacecraft);
for i_sc = 1:n_trial_orbits
    [~, abs_traj, ~] = ErosGravity.integrate(time_vector, sc_initial_state_array(i_sc, :), 'absolute' );
    trial_orbits(:, :, i_sc) = abs_traj;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Find Best Observation Points and Rewards for Given Orbits         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize the information for the optimal swarm orbits - we will fill
% these out sequentially,
K = length(time_vector);
swarm_map = zeros(n_spacecraft, num_points ) ;
swarm_observation_flow = zeros(n_spacecraft, K);
swarm_observed_points = -1.*ones(n_spacecraft, K);
swarm_priority = zeros(n_spacecraft, K);
swarm_reward = zeros(n_spacecraft, 1);
swarm_orbits = zeros(K, 6, n_spacecraft);

best_orbits = [];

% Add best orbits to swarm one spacecraft at a time
for i_sc = 1:n_spacecraft
    if sc_obs_type(i_sc) ~= 0 % if not carrier spacecraft
        % Get Rewards Given the Map of Previous Agents and Record Best
        interim_observed_points = swarm_observed_points(1:i_sc,:);
        best_reward = -1;
        for j_orbit = 1:n_trial_orbits
            if ~ismember(j_orbit,best_orbits) % if the orbit is not already being used
                new_orbit_set = trial_orbits(:,:, [best_orbits, j_orbit]); % Add j_orbit to the set of best orbits found so far.
                [map, observation_flow, observed_points, priority, reward] = observed_points_optimizer_main(SmallBodyParameters, time_vector, sc_obs_type(1:i_sc), new_orbit_set, interim_observed_points); % get reward
                if sum(reward)>sum(best_reward)
                    i_best_orbit = j_orbit;
                    best_reward = reward;
                    best_map = map;
                    best_observation_flow = observation_flow;
                    best_observed_points  = observed_points;
                    best_priority = priority;
                end
            end
        end
       
        % Store Best Orbit in the Swarm Arrays
        swarm_map(i_sc,:) = best_map(i_sc,:) ;
        swarm_observation_flow(i_sc,:) = best_observation_flow(i_sc,:);
        swarm_observed_points(i_sc,:) = best_observed_points(i_sc,:);
        swarm_priority(i_sc,:) = best_priority(i_sc,:);
        swarm_reward(i_sc,:) = best_reward(i_sc,:);
        swarm_orbits(:,:,i_sc) = trial_orbits(:,:, i_best_orbit);
        best_orbits = [best_orbits, i_best_orbit]; % store the index of the best orbit;
    else
        swarm_observed_points(i_sc,:) = zeros(1,K); % indicate no observation is made for carrier 
    end
end

end

