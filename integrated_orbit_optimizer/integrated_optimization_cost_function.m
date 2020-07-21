%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%  Wrapper function. Given the parameters of a set of orbits, returns the %
%  amount of science delivered to the base station by calling             %
%  observation_and_communication_optimizer.                               %
%  For use with integrated_optimization_driver.                           %  
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

function [goal, gradient, dgoal_dic, dk_dbandwidth, dbandwidth_dlocation, dk_dlocation, dlocation_dic] = integrated_optimization_cost_function(swarm, sc_initial_condition_vector, initial_condition_scaling_factor, gravity_model, bandwidth_parameters, trajectory_bounds, optimize_carrier, verbose)

% Cost function for GBO.
% Inputs
% - Swarm, a structure describing the current state of the system
% - Initial conditions for all orbits relay orbits (to be specified by
%    optimizer, possibly in a scaled form)
% - Location scaling factor (a horrible hack)
% - Gravity model
% - Bandwidth parameters, a struct with three fields: 
%  - reference_bandwidth, the bandwidth at a given reference_distance
%  - reference_distance
%  - max_bandwidth, a maximum bandwidth for close-by ops
% Outputs:
% - Goal, the amount of science delivered
% - Gradient, the gradient of the amt of science delivered wrt the the
%    relevant variables

%% Default bandwidth model
if nargin<8
    verbose=false;
end
if nargin<7
    optimize_carrier=false;
end

if nargin<6
    trajectory_bounds.max_distance_m = 120000;
    trajectory_bounds.min_distance_m = 15000;
end

if nargin<5
    bandwidth_parameters.reference_bandwidth = 250000;
    bandwidth_parameters.reference_distance = 100000;
    bandwidth_parameters.max_bandwidth = 100*1e6;
end

% addpath(genpath('../utilities'))
% addpath('../network_flow_communication_optimizer')

%% Unpack initial conditions and sanity check them
N = swarm.get_num_spacecraft();

assert(N * 6 == length(sc_initial_condition_vector), "ERROR: relay orbit index dimensions and initial conditions vector dimensions do not agree")
assert(length(initial_condition_scaling_factor) == 6, "ERROR: initial conditions scaling factor should be a vector of length 6")

% Unpack the initial conditions - they have to be in a vector to be
% compatible with algorithms in the opt toolbox
sc_initial_condition = reshape(sc_initial_condition_vector, [6, N]);
% Apply scaling factor
for sc = 1:N
    sc_initial_condition(:,sc) = sc_initial_condition(:,sc)./initial_condition_scaling_factor;
end

%% Integrate traejctories, check for collisions, and for flying too far
if verbose
    fprintf(" CF: integrating trajectories ")
end
% try

%     swarm.parallel_integrate_trajectories(gravity_model, sc_initial_condition', 'absolute');
% catch ME
%     if (strcmp(ME.identifier, 'SBDT:harmonic_gravity:inside_radius'))
%         %warning("ERROR: something went wrong with integrating the trajectory. Inspect the warnings above.")
%         warning("ERROR: Trajectory dipped inside reference radius. Returning inf.");
%         goal = inf;
%         gradient = nan;
%         return
%     else
%         rethrow(ME);
%     end
% end
% if ~swarm.all_trajectories_set || swarm.collision_with_asteroid(gravity_model)
%     warning("ERROR: Trajectory dipped inside reference radius or crashed. Returning inf.");
%     goal = inf;
%     gradient = nan;
%     return
% else
%     warning("All trajectories outside reference radius, continuing.");
% end
carrier_index = swarm.get_indicies_of_type(0);
if ~optimize_carrier
    assert(length(carrier_index)==1, "Multiple carriers! This will not work");
end
for i_sc = 1:N
    % If optimize_carrier is off and this is the carrier, do not touch the
    % carrier orbit. Count on it having been populated outside.
    if optimize_carrier || i_sc ~= carrier_index
        if verbose
            fprintf(" %d, ", i_sc);
        end
        try
            swarm.integrate_trajectory(i_sc, gravity_model, sc_initial_condition(:, i_sc)', 'absolute');
            distances = vecnorm(swarm.abs_trajectory_array(:,1:3,i_sc),2,2);
            if max(distances)> trajectory_bounds.max_distance_m
                warning("ERROR: Trajectory %d went too far from asteroid (max distance %f m, bound %f m). Returning inf.", i_sc, max(distances), trajectory_bounds.max_distance_m);
                goal = inf;
                gradient = nan;
                return
            end
            if min(distances) < trajectory_bounds.min_distance_m
                warning("ERROR: Trajectory %d went too close to asteroid (min distance %f m, bound %f m). Returning inf.", i_sc, min(distances), trajectory_bounds.min_distance_m);
                goal = inf;
                gradient = nan;
                return
            end
        catch ME
            if (strcmp(ME.identifier, 'SBDT:harmonic_gravity:inside_radius'))
                warning("ERROR: Trajectory %d dipped inside reference radius. Returning inf.", i_sc);
                goal = inf;
                gradient = nan;
                return
            else
                disp(ME.message);
                warning("Weird error while integrating, returning inf and moving on");
                goal = inf;
                gradient = nan;
                return
                %rethrow(ME);
            end
        end
    else
        assert(~ismember(carrier_index, swarm.unset_trajectories), "ERROR: carrier trajectory is not optimized, it should be set outside");
    end
end
if verbose
    fprintf("\n");
end

%% build bandwidth model
%bandwidth_model = @(x1,x2) min(bandwidth_parameters.reference_bandwidth * (bandwidth_parameters.reference_distance/norm(x2-x1,2))^2, bandwidth_parameters.max_bandwidth*1e6); 

spherical_asteroid_parameters.max_radius = gravity_model.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_parameters.min_radius = gravity_model.BodyModel.shape.maxRadius*1e3;

occlusion_test =  @(x1, x2) is_occluded(x1, x2, spherical_asteroid_parameters);
bandwidth_model = @(x1, x2) quadratic_comm_model(x1, x2, bandwidth_parameters,occlusion_test);

%% Call inner-loop optimizer

% Numerical conditioning is critical to get a good solution from the
% communication optimizer. Mosek 9 will deal with ill-conditioned problems,
% but just about every free solver (and Mosek 8) will report infeasibility
% if appropriate scaling is not applied.
% Here, we attempt to make a guess at the "mean" information flow by
% taking the mean data rate, excluding zeros. We use the mean, as opposed
% to the median, because we do _not_ want to throw out outlier instruments
% generating a disproportionate amount of data.
if verbose
    disp(" CF: call optimizer")
end
[swarm, goal] = observation_and_communication_optimizer(gravity_model, swarm, bandwidth_model, NaN, verbose);
if verbose
    disp(" CF: gradient")
end

if nargout>1  % compute gradient
    [dgoal_dic, dk_dbandwidth, dbandwidth_dlocation, dk_dlocation, dlocation_dic] = compute_integrated_gradient(swarm, bandwidth_parameters,spherical_asteroid_parameters);
    gradient = zeros(size(sc_initial_condition_vector));
    for sc = 1:N
        if optimize_carrier || sc ~= carrier_index
            % If we do not optimize the carrier, the gradient of the goal
            % wrt the carrier is zero.
            offset = 6*(sc-1);
            assert(all(size(dgoal_dic{sc}) == size(initial_condition_scaling_factor')), "ERROR: scaling factor shape is wrong")
            gradient(1+offset:6+offset) = dgoal_dic{sc}./initial_condition_scaling_factor';
        end
    end
        % Minimize
    gradient = -gradient;
end

% Minimize
goal = -goal;
if verbose
    disp(" CF: done")
end
end