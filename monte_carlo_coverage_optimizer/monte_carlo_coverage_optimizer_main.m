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

function Swarm = monte_carlo_coverage_optimizer_main(AsteroidModel, Swarm, n_trial_orbits)
%MONTE_CARLO_COVERAGE_OPTIMIZER  Generates a set of random orbits, and
%assigns one of these orbits to each instrument carrying spacecraft in the
%swarm. (does not set carrier orbit)

%   The orbits are chosen sequentially (one spacecraft after another)
%   such that the overall coverage reward is maximized on each iteration.
%
%   Syntax: Swarm = monte_carlo_coverage_optimizer_main(AsteroidModel, Swarm, n_trial_orbits)
%
%   Inputs:
%    - Asteroid_Model
%    - Swarm
%    - n_trial_orbits: number of trial orbits to used in the Monte Carlo
%
%   Outputs:
%    - Swarm

N = Swarm.get_num_spacecraft();
best_reward = 0;

%% Generate Trial Swarms
trial_swarm = cell(1,n_trial_orbits); % cell array of Swarm copies
trial_initial_states = initialize_random_orbits(n_trial_orbits, AsteroidModel); % set of orbit ICs that we will choose from
best_orbits_set = []; % indices of best orbits from trial_initial_states
iter = 0;

max_iter = sum((n_trial_orbits-N+1):n_trial_orbits);
h = waitbar(iter/max_iter,'Finding orbits...');

for i_sc = setdiff(1:N, Swarm.get_indicies_of_type(0))
    reset_reward = true;
    sc_same_type = intersect(1:i_sc, Swarm.get_indicies_of_type(Swarm.Parameters.types{i_sc})); % all SC <= i_sc of same type
    
    for i_orbit = setdiff(1:n_trial_orbits,best_orbits_set)
        trial_swarm{i_orbit} = Swarm.copy(); % copy Swarm, which includes the best trajectories for spacecraft 1:(i_sc-1)
        trial_swarm{i_orbit}.integrate_trajectory(i_sc, AsteroidModel, trial_initial_states(i_orbit,:), 'absolute'); % add in one of the trial orbits
        trial_swarm{i_orbit} = observed_points_optimizer_main(AsteroidModel, trial_swarm{i_orbit}, sc_same_type, i_sc); % observe the asteroid, and update the coverage reward to include the new orbit
        % above 3 lines could be made paralell using parfor
        
        if (trial_swarm{i_orbit}.get_coverage_reward() > best_reward) || (reset_reward==true) % keep track of the best trial swarm
            reset_reward = false;
            best_reward = trial_swarm{i_orbit}.get_coverage_reward();
            best_swarm = trial_swarm{i_orbit}.copy();
            best_orbit = i_orbit;
        end
        waitbar(iter/max_iter,h,'Finding orbits...');
        iter = iter+1;
    end
    best_orbits_set = [best_orbits_set, best_orbit]; %#ok<AGROW>
    Swarm = best_swarm.copy(); % Swarm now contains the best trajectories for spacecraft 1:i_sc
end

close(h)
end

